# collision_avoidance_behavior

Distributed path-lock collision avoidance behavior for multi-agent systems built with [Aerostack2](https://github.com/aerostack2/aerostack2), using pairwise mutual-exclusion locks over [as2_ca](https://github.com/CoreSenseEU/collective_awareness_structure) to prevent drones from occupying conflicting flight paths simultaneously.

## Overview

Each drone runs an independent `CollisionAvoidanceBehavior` node. When a drone wants to fly a path, it first acquires a distributed lock by sending lock requests to all known peers. Peers evaluate whether the requested path conflicts with their own held or requested path (using minimum polyline distance) and either grant the lock immediately or defer the grant until they release. Only once all peers have granted the lock does the drone begin its motion via the `GoToWaypoint` action. On completion it releases the lock, allowing deferred peers to proceed.

```
Drone A                                    Drone B
┌────────────────────────────┐            ┌────────────────────────────┐
│  CollisionAvoidanceBehavior│            │  CollisionAvoidanceBehavior│
│                            │ LockRequest│                            │
│  REQUESTING ───────────────┼───────────►│  (conflict check)          │
│                            │   Grant    │                            │
│  HOLDING   ◄───────────────┼────────────│                            │
│                            │            │                            │
│  GoToBehavior (motion)     │            │  REQUESTING (deferred)     │
│                            │  Release   │                            │
│  RELEASING ───────────────►│            │  HOLDING → motion          │
└────────────────────────────┘            └────────────────────────────┘
```

The collision-detection strategy is provided by a **plugin** (`pairwise_path_lock_plugin` by default), making it straightforward to swap in alternative algorithms.

## Installation

```bash
cd ~/cs_test_ws/src
git clone <this-repository>
cd ~/cs_test_ws
source ~/aerostack2_ws/install/setup.bash
colcon build --packages-select collision_avoidance_behavior
```

Dependencies (provided by Aerostack2):

- `as2_core`, `as2_behavior`, `as2_msgs`, `as2_ca`
- `pluginlib`, `rclcpp`, `rclcpp_action`, `geometry_msgs`, `std_srvs`
- `Eigen3`

## CollisionAvoidanceBehavior node

Manages the full lock lifecycle: plugin loading, lock acquisition (with peer negotiation via `as2_ca`), optional motion via `GoToWaypoint`, and lock release. Supports pause/resume — the lock is released on pause and re-acquired before motion resumes.

### Running the node

```bash
ros2 launch collision_avoidance_behavior collision_avoidance_behavior_launch.py \
  namespace:=drone0 \
  plugin_name:="pairwise_path_lock_plugin::Plugin"
```

Or directly:

```bash
ros2 run collision_avoidance_behavior collision_avoidance_behavior_node --ros-args \
  -r __ns:=/drone0 \
  -p plugin_name:="pairwise_path_lock_plugin::Plugin" \
  -p safety_distance:=1.5
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `plugin_name` | `pairwise_path_lock_plugin::Plugin` | Pluginlib class name of the collision strategy to load |
| `safety_distance` | `1.5` | Minimum path separation in metres below which paths are considered conflicting |

### Action interface

| Action | Type | Description |
|---|---|---|
| `CollisionAvoidanceBehavior` | `as2_msgs/action/CollisionAvoidance` | Main behavior action |

**Goal fields:**

| Field | Type | Description |
|---|---|---|
| `path` | `PoseStampedWithID[]` | Intended flight path (drone's current pose is prepended automatically) |
| `goal_pose` | `PoseStamped` | Motion target sent to `GoToWaypoint` once the lock is held; omit for lock-only use |
| `safety_distance` | `float32` | Per-request override for the safety distance (0 → use node parameter) |
| `max_speed` | `float32` | Max speed forwarded to `GoToWaypoint` (0 → 1 m/s default) |
| `plugin_name` | `string` | Per-request plugin override; empty → keep current plugin |

**Result fields:**

| Field | Type | Description |
|---|---|---|
| `collision_avoidance_success` | `bool` | `true` if the lock was held and motion completed successfully |

**Feedback fields:**

| Field | Type | Description |
|---|---|---|
| `state` | `string` | Plugin state: `IDLE`, `REQUESTING`, `HOLDING`, `RELEASING` |
| `lock_held` | `bool` | Whether the lock is currently held |
| `pending_peers` | `string[]` | Peers whose grant has not yet been received |
| `conflicting_peers` | `string[]` | Peers with a conflicting path currently held |
| `deferred_count` | `uint32` | Number of peer requests that this drone has deferred |
| `motion_in_progress` | `bool` | Whether `GoToWaypoint` is currently executing |

### Inter-agent messages (via as2_ca)

| Type key | ROS message | Direction | Description |
|---|---|---|---|
| `ca_lock_request` | `as2_msgs/msg/CAPathLockRequest` | Requester → Peers | Announces intended path and requests grants |
| `ca_lock_grant` | `as2_msgs/msg/CAPathLockGrant` | Peer → Requester | Grants the lock for a specific request ID |
| `ca_lock_release` | `as2_msgs/msg/CAPathLockRelease` | Holder → Peers | Signals that the lock has been released |

### Data flow

```
Action goal
      │
      ▼
  start_acquisition(path, peers, req_id, safety_dist)
      │
      ├──(as2_ca)──► CAPathLockRequest → each peer
      │
      │  [peer evaluates conflict, responds with Grant or defers]
      │
      ▼
  on_grant() × N peers → HOLDING
      │
      ▼
  GoToWaypoint goal (if goal_pose != zero)
      │
      ▼
  motion complete
      │
      ▼
  release() ──(as2_ca)──► CAPathLockRelease → all peers
      │
      ▼
  plugin state == IDLE → action SUCCESS
```

## Plugins

### pairwise_path_lock_plugin

Event-driven distributed mutual-exclusion using lexicographic priority for tie-breaking. When two drones request conflicting paths simultaneously, the drone with the lexicographically smaller namespace proceeds first; the other defers its grant until the first releases.

**Conflict detection:** `path_geometry::min_polyline_distance` computes the minimum 3-D Euclidean distance between two piecewise-linear paths (segment-to-segment for all pairs). A conflict is declared when this distance falls below `safety_distance`.

**State machine per node:**

```
IDLE ──start_acquisition──► REQUESTING ──all grants received──► HOLDING
                                                                      │
                                                              release()│
                                                                      ▼
                          IDLE ◄──flush deferred grants────── RELEASING
```

## Plugin API

### CollisionAvoidanceBase

| Method | Description |
|---|---|
| `initialize(node, ca_client, own_id)` | Attach to a node and the CA gateway client |
| `start_acquisition(path, peers, req_id, safety_dist)` | Begin lock acquisition for the given path |
| `release()` | Release the held lock and notify all peers |
| `on_request(req, sender)` | Handle an incoming `CAPathLockRequest` from a peer |
| `on_grant(grant, sender)` | Handle an incoming `CAPathLockGrant` from a peer |
| `on_release(rel, sender)` | Handle an incoming `CAPathLockRelease` from a peer |
| `status()` | Return the current plugin state as a `Status` struct |

### path_geometry

| Function | Description |
|---|---|
| `min_polyline_distance(a, b)` | Minimum 3-D Euclidean distance between two piecewise-linear paths; single-pose paths are treated as points |
