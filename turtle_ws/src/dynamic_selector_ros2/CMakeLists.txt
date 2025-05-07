cmake_minimum_required(VERSION 3.8)
project(dynamic_selector_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nrg_utility_behaviors REQUIRED)

# Include directory
install(DIRECTORY include/  DESTINATION include/)
ament_export_include_directories(include)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
src/smart_input_node.cpp
src/decision_module.cpp
src/dynamic_selector.cpp
)

ament_target_dependencies(${PROJECT_NAME}
behaviortree_cpp
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ds_test src/ds_test.cpp src/smart_input_node.cpp src/decision_module.cpp src/dynamic_selector.cpp)
ament_target_dependencies(ds_test
behaviortree_cpp
std_msgs
nrg_utility_behaviors
)

target_link_libraries(ds_test behaviortree_cpp)

install(
  TARGETS ds_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
