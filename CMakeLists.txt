cmake_minimum_required(VERSION 3.8)
project(collision_avoidance_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rclcpp_cascade_lifecycle
  lifecycle_msgs
  pluginlib
  cs4home_core
  ca_structure
  as2_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  eigen3_cmake_module
)

foreach(DEP ${DEPS})
  find_package(${DEP} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
list(APPEND DEPS Eigen3)

# ── Plugin shared library ─────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/path_geometry.cpp
  plugins/pairwise_path_lock_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# ── Behavior node (Afferent + Core + CognitiveModule) ────────────────────────
add_executable(collision_avoidance_behavior_node
  src/collision_avoidance_behavior_afferent.cpp
  src/collision_avoidance_behavior_core.cpp
  src/collision_avoidance_behavior.cpp
  src/collision_avoidance_behavior_node.cpp
  src/path_geometry.cpp
)
target_include_directories(collision_avoidance_behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(collision_avoidance_behavior_node ${DEPS})
target_link_libraries(collision_avoidance_behavior_node ${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}
  collision_avoidance_behavior_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp src/path_geometry.cpp)
  target_include_directories(test_path_geometry PUBLIC include)
  ament_target_dependencies(test_path_geometry as2_msgs nav_msgs geometry_msgs Eigen3)

  ament_add_gtest(test_plugin_logic test/test_plugin_logic.cpp
    plugins/pairwise_path_lock_plugin.cpp src/path_geometry.cpp)
  target_include_directories(test_plugin_logic PUBLIC include)
  ament_target_dependencies(test_plugin_logic ${DEPS})
endif()

ament_package()
