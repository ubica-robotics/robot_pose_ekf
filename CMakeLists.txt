cmake_minimum_required(VERSION 3.8)
project(robot_pose_ekf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ubica_rclcpp_utils REQUIRED)
# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

set(ros_dependencies geometry_msgs rosidl_default_generators nav_msgs rclcpp rosidl_default_generators sensor_msgs tf2 tf2_geometry_msgs tf2_ros ubica_rclcpp_utils)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetStatus.srv"
)

include_directories(
  include
)

add_executable(odom_estimation_node 
                       src/odom_estimation_node.cpp 
                       src/odom_estimation.cpp
                       src/nonlinearanalyticconditionalgaussianodo.cpp)
ament_target_dependencies(odom_estimation_node ${ros_dependencies})
target_link_libraries(odom_estimation_node ${BFL_LIBRARIES})
rosidl_target_interfaces(odom_estimation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
set_target_properties(odom_estimation_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

install(TARGETS odom_estimation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY test/data
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS 
    nodes/wtf.py
    DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(rosbag2 REQUIRED)

  ament_add_gtest_executable(test_${PROJECT_NAME} test/test_robot_pose_ekf.cpp)
  ament_target_dependencies(test_${PROJECT_NAME} ${ros_dependencies})
  target_link_libraries(test_${PROJECT_NAME} ${BFL_LIBRARIES})

  ament_add_gtest_executable(test_${PROJECT_NAME}_zero_covariance test/test_robot_pose_ekf_zero_covariance.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_zero_covariance ${ros_dependencies})
  target_link_libraries(test_${PROJECT_NAME}_zero_covariance ${BFL_LIBRARIES})

  add_ros_test(test/launch/test_robot_pose_ekf.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/launch/test_robot_pose_ekf_zero_covariance.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()


ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
