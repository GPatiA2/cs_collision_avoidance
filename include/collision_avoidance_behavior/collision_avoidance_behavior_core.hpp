// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_CORE_HPP_
#define COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_CORE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "cs4home_core/Core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"

#include "ca_structure/ca_gateway_client.hpp"

#include "as2_msgs/action/collision_avoidance.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_msgs/msg/ca_path_lock_request.hpp"
#include "as2_msgs/msg/ca_path_lock_grant.hpp"
#include "as2_msgs/msg/ca_path_lock_release.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "collision_avoidance_behavior/collision_avoidance_base.hpp"

namespace collision_avoidance_behavior
{

using CollisionAvoidanceAction = as2_msgs::action::CollisionAvoidance;

class CollisionAvoidanceBehaviorCore : public cs4home_core::Core
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CollisionAvoidanceBehaviorCore)

  using GoalHandleT = rclcpp_action::ServerGoalHandle<CollisionAvoidanceAction>;

  explicit CollisionAvoidanceBehaviorCore(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure() override;
  bool activate()  override;
  bool deactivate() override;

private:
  std::shared_ptr<ca_structure::CAGatewayClient> ca_client_;

  std::unique_ptr<pluginlib::ClassLoader<collision_avoidance_base::CollisionAvoidanceBase>>
  loader_;
  std::shared_ptr<collision_avoidance_base::CollisionAvoidanceBase> plugin_;
  std::string current_plugin_name_;

  rclcpp_action::Server<CollisionAvoidanceAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleT> goal_handle_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SharedPtr go_to_client_;
  rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr go_to_goal_handle_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr goto_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr goto_resume_client_;

  std::string own_id_;
  uint32_t req_id_counter_{0};
  bool released_{false};

  std::vector<as2_msgs::msg::PoseStampedWithID> stored_path_;
  double stored_safety_dist_{0.0};

  enum MotionPhase {
    IDLE_PHASE, REQUESTING_LOCK, HOLDING_LOCK, EXECUTING_MOTION, RELEASING_LOCK
  };
  MotionPhase current_phase_{IDLE_PHASE};
  bool motion_rejected_{false};
  bool motion_succeeded_{false};
  bool motion_aborted_{false};
  bool go_to_paused_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CollisionAvoidanceAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleT> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleT> goal_handle);

  bool do_activate(std::shared_ptr<const CollisionAvoidanceAction::Goal> goal);
  bool do_deactivate();
  bool do_pause();
  bool do_resume();
  void do_run();
  void do_execution_end();

  bool load_plugin(const std::string & plugin_name);
  bool call_trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);

  void go_to_response_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr &);
  void go_to_result_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult &);
};

}  // namespace collision_avoidance_behavior

#endif  // COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_CORE_HPP_
