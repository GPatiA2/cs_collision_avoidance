// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#include "collision_avoidance_behavior/collision_avoidance_behavior_afferent.hpp"

namespace collision_avoidance_behavior
{

CollisionAvoidanceBehaviorAfferent::CollisionAvoidanceBehaviorAfferent(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: cs4home_core::Afferent("collision_avoidance_behavior_afferent", parent)
{
}

bool CollisionAvoidanceBehaviorAfferent::configure()
{
  // ONDEMAND subscription: Core reads the latest pose before lock acquisition
  // to prepend the drone's current position to the path.
  return create_subscriber("self_localization/pose", "geometry_msgs/msg/PoseStamped");
}

}  // namespace collision_avoidance_behavior
