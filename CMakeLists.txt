cmake_minimum_required(VERSION 3.8)
project(virtual_objects_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED))
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(virtual_objects_merger src/virtual_objects_merger.cpp
					src/virtual_objects_merger_node.cpp
					)
ament_target_dependencies(virtual_objects_merger rclcpp  
                autoware_auto_perception_msgs tf2_ros tf2 tf2_geometry_msgs)

install(TARGETS
virtual_objects_merger
DESTINATION lib/${PROJECT_NAME}
)



ament_package()
