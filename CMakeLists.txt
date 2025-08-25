cmake_minimum_required(VERSION 3.8)
project(lid_odom)


add_compile_options(-O3)
#add_compile_options(-g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)

# Create the library
add_library(${PROJECT_NAME}_lib SHARED
  src/Pipeline.cpp
  src/Registration.cpp
  src/VoxelMap.cpp
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  Sophus::Sophus
  TBB::tbb
)

# Create the executable node
add_executable(${PROJECT_NAME}_node src/lid_odom_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
)

# Install headers
install(
  DIRECTORY include/ src/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Install library and executables
install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node 
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  Sophus
  TBB
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
