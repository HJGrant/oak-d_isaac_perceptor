cmake_minimum_required(VERSION 3.8)
project(my_oakd_launch)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# If you're using GCC or Clang and an older version that requires linking filesystem explicitly:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Link filesystem explicitly for GCC versions < 9 or older Clang versions
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0.0")
    target_link_libraries(extract_voxel_map stdc++fs)
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nvblox_ros_common REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(isaac_ros_managed_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_nitros_camera_info_type REQUIRED)
find_package(isaac_ros_gxf REQUIRED)

# Create the executable
add_executable(extract_voxel_map src/extract_voxel_map.cpp)
ament_target_dependencies(extract_voxel_map rclcpp rosbag2_transport nvblox_msgs)

# Install the executable
install(TARGETS
  extract_voxel_map
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
