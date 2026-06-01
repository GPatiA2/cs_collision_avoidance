// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       collision_avoidance_behavior.cpp
 *  \brief      CollisionAvoidanceBehavior CognitiveModule — on_configure wires components
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "collision_avoidance_behavior/collision_avoidance_behavior.hpp"

#include <memory>

namespace collision_avoidance_behavior
{

CollisionAvoidanceBehavior::CollisionAvoidanceBehavior()
: cs4home_core::CognitiveModule("CollisionAvoidanceBehavior")
{
}

CollisionAvoidanceBehavior::CallbackReturnT
CollisionAvoidanceBehavior::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto self = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this());

  // Afferent: subscribes to self_localization/pose (ONDEMAND)
  afferent_ = std::make_shared<CollisionAvoidanceBehaviorAfferent>(self);
  afferent_->configure();

  // Core: plugin, CA client, action server
  auto core = std::make_shared<CollisionAvoidanceBehaviorCore>(self);
  core->set_afferent(afferent_);
  if (!core->configure()) {
    RCLCPP_ERROR(get_logger(), "CollisionAvoidanceBehaviorCore::configure() failed");
    return CallbackReturnT::FAILURE;
  }
  core_ = core;

  RCLCPP_INFO(get_logger(), "CollisionAvoidanceBehavior configured.");
  return CallbackReturnT::SUCCESS;
}

}  // namespace collision_avoidance_behavior
