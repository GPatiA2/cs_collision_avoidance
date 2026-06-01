// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       collision_avoidance_behavior.hpp
 *  \brief      CollisionAvoidanceBehavior CognitiveModule (cs4home version)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#pragma once

#include <memory>

#include "cs4home_core/CognitiveModule.hpp"

#include "collision_avoidance_behavior/collision_avoidance_behavior_afferent.hpp"
#include "collision_avoidance_behavior/collision_avoidance_behavior_core.hpp"

namespace collision_avoidance_behavior
{

class CollisionAvoidanceBehavior : public cs4home_core::CognitiveModule
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CollisionAvoidanceBehavior)
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CollisionAvoidanceBehavior();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
};

}  // namespace collision_avoidance_behavior
