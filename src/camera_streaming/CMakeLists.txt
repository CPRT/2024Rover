cmake_minimum_required(VERSION 3.5)
project(camera_streaming)

# Set the C++ standard to C++14 (or higher, if needed)
set(CMAKE_CXX_STANDARD 14)

# Find dependencies (ROS2, GStreamer, and PkgConfig)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

# Use pkg-config to find the necessary GStreamer libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

# Include the necessary directories for GStreamer
include_directories(
  include/${PROJECT_NAME}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
)


# Link directories for the necessary libraries
link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  ${GST_VIDEO_LIBRARY_DIRS}
)

# Add the executable target
add_executable(camera_streaming src/webrtc_node.cpp)

# Link with the required libraries
target_link_libraries(camera_streaming
  rclcpp::rclcpp
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
)

# Ensure that the pkg-config variables are used
pkg_check_modules(GLIB REQUIRED glib-2.0)

# Set up any necessary ROS2 specific configurations
ament_target_dependencies(camera_streaming rclcpp interfaces)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the executable (optional, depending on your needs)
install(TARGETS camera_streaming DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install configuration files (if any)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()