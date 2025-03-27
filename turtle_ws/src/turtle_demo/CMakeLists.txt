cmake_minimum_required(VERSION 3.8)
project(turtle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(turtle_behaviors REQUIRED)

set(THIS_PACKAGE_DEPS
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    turtlesim
    behaviortree_ros2
    btcpp_ros2_interfaces
    turtle_behaviors
)

add_executable(turtle_broadcaster src/turtle_broadcaster.cpp)
ament_target_dependencies(turtle_broadcaster ${THIS_PACKAGE_DEPS})

add_executable(turtle_listener src/turtle_listener.cpp)
ament_target_dependencies(turtle_listener ${THIS_PACKAGE_DEPS})

add_executable(turtle_test src/turtle_test.cpp)
ament_target_dependencies(turtle_test ${THIS_PACKAGE_DEPS})

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

install(TARGETS
    turtle_broadcaster
    turtle_listener
    turtle_test
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
