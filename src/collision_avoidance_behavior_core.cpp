// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       collision_avoidance_behavior_core.cpp
 *  \brief      CollisionAvoidanceBehaviorCore — cs4home Core component
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "collision_avoidance_behavior/collision_avoidance_behavior_core.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace collision_avoidance_behavior
{

CollisionAvoidanceBehaviorCore::CollisionAvoidanceBehaviorCore(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: cs4home_core::Core("collision_avoidance_behavior_core", parent)
{
}

bool CollisionAvoidanceBehaviorCore::configure()
{
  own_id_ = parent_->get_namespace();
  if (!own_id_.empty() && own_id_.front() == '/') {
    own_id_ = own_id_.substr(1);
  }

  // Plugin
  loader_ = std::make_unique<
    pluginlib::ClassLoader<collision_avoidance_base::CollisionAvoidanceBase>>(
    "collision_avoidance_behavior",
    "collision_avoidance_base::CollisionAvoidanceBase");

  parent_->declare_parameter("plugin_name", "pairwise_path_lock_plugin::Plugin");
  const std::string default_plugin =
    parent_->get_parameter("plugin_name").as_string();
  if (!load_plugin(default_plugin.empty() ?
    "pairwise_path_lock_plugin::Plugin" : default_plugin))
  {
    return false;
  }

  // CA gateway client
  auto self = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(parent_);
  ca_client_ = std::make_shared<ca_structure::CAGatewayClient>(self);

  ca_client_->register_module<as2_msgs::msg::CAPathLockRequest>(
    "ca_lock_request", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockRequest & msg, const std::string & sender) {
      if (plugin_) {plugin_->on_request(msg, sender);}
    });

  ca_client_->register_module<as2_msgs::msg::CAPathLockGrant>(
    "ca_lock_grant", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockGrant & msg, const std::string & sender) {
      if (plugin_) {plugin_->on_grant(msg, sender);}
    });

  ca_client_->register_module<as2_msgs::msg::CAPathLockRelease>(
    "ca_lock_release", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockRelease & msg, const std::string & sender) {
      if (plugin_) {plugin_->on_release(msg, sender);}
    });

  plugin_->initialize(parent_.get(), ca_client_, own_id_);

  // GoTo sub-action client
  go_to_client_ = rclcpp_action::create_client<as2_msgs::action::GoToWaypoint>(
    parent_, "GoToBehavior");

  const std::string go_to_base =
    std::string(parent_->get_namespace()) + "/GoToBehavior";
  goto_pause_client_ = parent_->create_client<std_srvs::srv::Trigger>(
    go_to_base + "/_behavior/pause");
  goto_resume_client_ = parent_->create_client<std_srvs::srv::Trigger>(
    go_to_base + "/_behavior/resume");

  // Action server
  const std::string action_name =
    std::string(parent_->get_namespace()) + "/CollisionAvoidanceBehavior";

  action_server_ = rclcpp_action::create_server<CollisionAvoidanceAction>(
    parent_,
    action_name,
    [this](const rclcpp_action::GoalUUID & uuid, auto goal) {
      return handle_goal(uuid, goal);
    },
    [this](auto gh) {return handle_cancel(gh);},
    [this](auto gh) {handle_accepted(gh);});

  RCLCPP_INFO(parent_->get_logger(),
    "CollisionAvoidanceBehaviorCore configured (id=%s).", own_id_.c_str());
  return true;
}

bool CollisionAvoidanceBehaviorCore::activate()  {return true;}

bool CollisionAvoidanceBehaviorCore::deactivate()
{
  if (run_timer_) {
    run_timer_->cancel();
    run_timer_.reset();
  }
  return true;
}

// ── Action server callbacks ─────────────────────────────────────────────────

rclcpp_action::GoalResponse CollisionAvoidanceBehaviorCore::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const CollisionAvoidanceAction::Goal> goal)
{
  return do_activate(goal) ?
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
    rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse CollisionAvoidanceBehaviorCore::handle_cancel(
  std::shared_ptr<GoalHandleT> /*goal_handle*/)
{
  do_deactivate();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CollisionAvoidanceBehaviorCore::handle_accepted(std::shared_ptr<GoalHandleT> goal_handle)
{
  goal_handle_ = goal_handle;
  run_timer_ = parent_->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {do_run();});
}

// ── Behavior logic ──────────────────────────────────────────────────────────

bool CollisionAvoidanceBehaviorCore::do_activate(
  std::shared_ptr<const CollisionAvoidanceAction::Goal> goal)
{
  if (goal->path.empty()) {
    RCLCPP_ERROR(parent_->get_logger(), "on_activate: path is empty");
    return false;
  }

  released_ = false;
  current_phase_ = REQUESTING_LOCK;
  motion_rejected_ = motion_succeeded_ = motion_aborted_ = go_to_paused_ = false;
  go_to_goal_handle_ = nullptr;

  if (!goal->plugin_name.empty() && goal->plugin_name != current_plugin_name_) {
    if (!load_plugin(goal->plugin_name)) {return false;}
    plugin_->initialize(parent_.get(), ca_client_, own_id_);
  }

  double safety_dist = (goal->safety_distance > 0.0f) ?
    static_cast<double>(goal->safety_distance) : 1.5;

  auto full_path = goal->path;
  auto pose_msg  = afferent_->get_msg<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose");
  if (pose_msg) {
    as2_msgs::msg::PoseStampedWithID p;
    p.id   = "current_position";
    p.pose = *pose_msg;
    full_path.insert(full_path.begin(), p);
  }

  stored_path_       = full_path;
  stored_safety_dist_ = safety_dist;

  const auto peers = ca_client_->get_known_peers();
  plugin_->start_acquisition(full_path, peers, ++req_id_counter_, safety_dist);
  return true;
}

bool CollisionAvoidanceBehaviorCore::do_deactivate()
{
  if (go_to_goal_handle_) {
    go_to_client_->async_cancel_goal(go_to_goal_handle_);
    go_to_goal_handle_ = nullptr;
  }
  if (!released_) {
    plugin_->release();
    released_ = true;
  }
  current_phase_ = RELEASING_LOCK;
  motion_aborted_ = true;
  return false;
}

bool CollisionAvoidanceBehaviorCore::do_pause()
{
  bool ok = true;
  if (current_phase_ == EXECUTING_MOTION) {
    ok = call_trigger(goto_pause_client_);
    if (ok) {go_to_paused_ = true;}
  }
  if (!released_) {
    plugin_->release();
    released_ = true;
  }
  return ok;
}

bool CollisionAvoidanceBehaviorCore::do_resume()
{
  released_ = false;
  current_phase_ = REQUESTING_LOCK;
  const auto peers = ca_client_->get_known_peers();
  plugin_->start_acquisition(stored_path_, peers, ++req_id_counter_, stored_safety_dist_);
  return true;
}

void CollisionAvoidanceBehaviorCore::do_run()
{
  if (!goal_handle_ || !goal_handle_->is_active()) {return;}

  auto feedback = std::make_shared<CollisionAvoidanceAction::Feedback>();
  auto result   = std::make_shared<CollisionAvoidanceAction::Result>();
  auto s = plugin_->status();

  feedback->state             = s.state;
  feedback->lock_held         = s.lock_held;
  feedback->pending_peers     = s.pending_peers;
  feedback->conflicting_peers = s.conflicting_peers;
  feedback->deferred_count    = s.deferred_count;
  feedback->motion_in_progress = (current_phase_ == EXECUTING_MOTION);
  goal_handle_->publish_feedback(feedback);

  if (current_phase_ == REQUESTING_LOCK) {
    if (!s.lock_held) {return;}

    current_phase_ = HOLDING_LOCK;

    if (go_to_paused_) {
      go_to_paused_ = false;
      current_phase_ = call_trigger(goto_resume_client_) ?
        EXECUTING_MOTION : RELEASING_LOCK;
      if (current_phase_ == RELEASING_LOCK) {motion_aborted_ = true;}
      return;
    }

    const auto & goal = goal_handle_->get_goal();
    const bool has_motion =
      (goal->goal_pose.pose.position.x != 0.0 ||
      goal->goal_pose.pose.position.y != 0.0 ||
      goal->goal_pose.pose.position.z != 0.0);

    if (has_motion) {
      auto go_to_goal = as2_msgs::action::GoToWaypoint::Goal();
      go_to_goal.target_pose.header = goal->goal_pose.header;
      go_to_goal.target_pose.point.x = goal->goal_pose.pose.position.x;
      go_to_goal.target_pose.point.y = goal->goal_pose.pose.position.y;
      go_to_goal.target_pose.point.z = goal->goal_pose.pose.position.z;
      go_to_goal.max_speed = goal->max_speed > 0.0f ? goal->max_speed : 1.0f;
      go_to_goal.yaw.mode  = as2_msgs::msg::YawMode::KEEP_YAW;

      auto opts = rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SendGoalOptions();
      opts.goal_response_callback =
        std::bind(&CollisionAvoidanceBehaviorCore::go_to_response_cbk, this,
        std::placeholders::_1);
      opts.result_callback =
        std::bind(&CollisionAvoidanceBehaviorCore::go_to_result_cbk, this,
        std::placeholders::_1);
      go_to_client_->async_send_goal(go_to_goal, opts);
      current_phase_ = EXECUTING_MOTION;
    } else {
      current_phase_ = RELEASING_LOCK;
      motion_succeeded_ = true;
    }
    return;
  }

  if (current_phase_ == EXECUTING_MOTION) {
    if (motion_rejected_ || motion_aborted_ || motion_succeeded_) {
      current_phase_ = RELEASING_LOCK;
    }
    return;
  }

  if (current_phase_ == RELEASING_LOCK) {
    if (!released_) {
      plugin_->release();
      released_ = true;
    }
    if (s.state == "IDLE") {
      result->collision_avoidance_success = motion_succeeded_;
      run_timer_->cancel();
      if (motion_succeeded_) {
        goal_handle_->succeed(result);
      } else {
        goal_handle_->abort(result);
      }
      do_execution_end();
    }
  }
}

void CollisionAvoidanceBehaviorCore::do_execution_end()
{
  if (!released_) {
    plugin_->release();
    released_ = true;
  }
  goal_handle_.reset();
}

// ── Helpers ─────────────────────────────────────────────────────────────────

bool CollisionAvoidanceBehaviorCore::load_plugin(const std::string & plugin_name)
{
  try {
    plugin_ = loader_->createSharedInstance(plugin_name);
    current_plugin_name_ = plugin_name;
    RCLCPP_INFO(parent_->get_logger(), "Plugin loaded: %s", plugin_name.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(parent_->get_logger(), "Failed to load plugin '%s': %s",
      plugin_name.c_str(), e.what());
    return false;
  }
}

bool CollisionAvoidanceBehaviorCore::call_trigger(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
{
  if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    return false;
  }
  return future.get()->success;
}

void CollisionAvoidanceBehaviorCore::go_to_response_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr & gh)
{
  if (!gh) {
    motion_rejected_ = true;
  } else {
    go_to_goal_handle_ = gh;
  }
}

void CollisionAvoidanceBehaviorCore::go_to_result_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult & r)
{
  switch (r.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      motion_succeeded_ = r.result->go_to_success;
      if (!motion_succeeded_) {motion_aborted_ = true;}
      break;
    default:
      motion_aborted_ = true;
      break;
  }
}

}  // namespace collision_avoidance_behavior
