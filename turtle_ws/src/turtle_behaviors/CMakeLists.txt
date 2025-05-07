cmake_minimum_required(VERSION 3.8)
project(turtle_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs)

# Declare the source files
set(BEHAVIOR_SOURCES
  src/target_within_range.cpp
  src/chase_target.cpp
  src/scan_search.cpp
)

include_directories(include)

# Create turtle_behaviors library
add_library(${PROJECT_NAME} SHARED ${BEHAVIOR_SOURCES})

ament_target_dependencies(${PROJECT_NAME}
behaviortree_cpp
geometry_msgs
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Install include files
install(DIRECTORY include/  DESTINATION include/)
ament_export_include_directories(include)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
