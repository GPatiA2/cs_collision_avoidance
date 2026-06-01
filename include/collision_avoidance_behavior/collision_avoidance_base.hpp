// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       collision_avoidance_base.hpp
 *  \brief      Collision avoidance plugin base (cs4home version — uses ca_structure)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <as2_msgs/msg/pose_stamped_with_id.hpp>
#include <as2_msgs/action/collision_avoidance.hpp>
#include <as2_msgs/msg/ca_path_lock_request.hpp>
#include <as2_msgs/msg/ca_path_lock_grant.hpp>
#include <as2_msgs/msg/ca_path_lock_release.hpp>
#include "ca_structure/ca_gateway_client.hpp"

namespace collision_avoidance_base
{

struct Status
{
  std::string state;
  bool lock_held{false};
  std::vector<std::string> pending_peers;
  std::vector<std::string> conflicting_peers;
  uint32_t deferred_count{0};
};

class CollisionAvoidanceBase
{
public:
  virtual ~CollisionAvoidanceBase() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<ca_structure::CAGatewayClient> ca_client,
    const std::string & own_id) = 0;

  virtual void start_acquisition(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & path,
    const std::vector<std::string> & peer_snapshot,
    uint32_t req_id,
    double safety_distance) = 0;

  virtual void release() = 0;

  virtual void on_request(
    const as2_msgs::msg::CAPathLockRequest & req,
    const std::string & sender) = 0;

  virtual void on_grant(
    const as2_msgs::msg::CAPathLockGrant & grant,
    const std::string & sender) = 0;

  virtual void on_release(
    const as2_msgs::msg::CAPathLockRelease & rel,
    const std::string & sender) = 0;

  virtual Status status() const = 0;
};

}  // namespace collision_avoidance_base
