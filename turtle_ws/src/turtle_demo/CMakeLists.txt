cmake_minimum_required(VERSION 3.8)
project(turtle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(behaviortree_cpp)
find_package(nrg_utility_behaviors)
find_package(turtle_behaviors REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    std_msgs
    geometry_msgs
    turtlesim    
    behaviortree_cpp
    nrg_utility_behaviors
    turtle_behaviors
)

add_executable(turtle_broadcaster src/turtle_broadcaster.cpp)
ament_target_dependencies(turtle_broadcaster ${THIS_PACKAGE_DEPS})

add_executable(turtle_listener src/turtle_listener.cpp)
ament_target_dependencies(turtle_listener ${THIS_PACKAGE_DEPS})

add_executable(turtle_BT src/turtle_BT.cpp)
ament_target_dependencies(turtle_BT ${THIS_PACKAGE_DEPS})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(TARGETS
    turtle_broadcaster
    turtle_listener
    turtle_BT
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
