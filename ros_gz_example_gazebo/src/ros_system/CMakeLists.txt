add_library(RosSystem
  SHARED
  RosSystem.cpp)

target_link_libraries(RosSystem
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

ament_target_dependencies(RosSystem
  "rclcpp"
  "std_msgs")

target_include_directories(RosSystem
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS RosSystem DESTINATION lib/${PROJECT_NAME})
