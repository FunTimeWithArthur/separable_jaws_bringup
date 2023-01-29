cmake_minimum_required(VERSION 3.8)
project(separable_jaws_bringup)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# executable: joint_trajectory_controller_test
add_executable(joint_trajectory_controller_test_node
        test/joint_trajectory_controller_test.cpp
)
ament_target_dependencies(joint_trajectory_controller_test_node
        rclcpp
        rclcpp_action
        control_msgs
)

# install application, test executables
install(TARGETS joint_trajectory_controller_test_node
        DESTINATION lib/${PROJECT_NAME}
)

# install config files, launch scripts
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        control_msgs
        rclcpp
        rclcpp_action
)

ament_package()
