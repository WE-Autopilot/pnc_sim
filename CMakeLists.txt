cmake_minimum_required(VERSION 3.8)
project(ap1_pnc_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# FIND PACKAGES
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ap1_msgs REQUIRED)

# --------------------------------------------------
# INCLUDE DIRECTORIES
# --------------------------------------------------
include_directories(
  include
)

# --------------------------------------------------
# BUILD EXECUTABLE
# --------------------------------------------------
add_executable(pnc_sim_node
  src/main.cpp
  src/simple_vehicle_sim_node.cpp
)

ament_target_dependencies(pnc_sim_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs     
  ap1_msgs
)

# --------------------------------------------------
# INSTALL
# --------------------------------------------------
install(TARGETS pnc_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
