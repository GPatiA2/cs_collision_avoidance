// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_AFFERENT_HPP_
#define COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_AFFERENT_HPP_

#include "cs4home_core/Afferent.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace collision_avoidance_behavior
{

class CollisionAvoidanceBehaviorAfferent : public cs4home_core::Afferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CollisionAvoidanceBehaviorAfferent)

  explicit CollisionAvoidanceBehaviorAfferent(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure() override;
};

}  // namespace collision_avoidance_behavior

#endif  // COLLISION_AVOIDANCE_BEHAVIOR__COLLISION_AVOIDANCE_BEHAVIOR_AFFERENT_HPP_
