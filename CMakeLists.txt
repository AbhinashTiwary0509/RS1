cmake_minimum_required(VERSION 3.8)
project(smart_factory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Add Qt dependencies
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Enable automoc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


add_executable(MasterControlNodeExe src/MasterControlNode.cpp src/SimpleGUI.cpp)

target_include_directories(MasterControlNodeExe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(MasterControlNodeExe rclcpp std_msgs sensor_msgs nav_msgs rclcpp_action nav2_msgs)

target_link_libraries(MasterControlNodeExe
    Qt5::Widgets
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
# ament_target_dependencies(minimal_opencv_ros2_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS
    MasterControlNodeExe
    DESTINATION lib/${PROJECT_NAME}
 )

 install(DIRECTORY launch models worlds urdf
  DESTINATION share/${PROJECT_NAME}/
)
 
# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()
