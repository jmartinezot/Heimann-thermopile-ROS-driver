cmake_minimum_required(VERSION 3.5)
project(heiman)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Uncomment the following section if your package has services, actions, or messages
# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage1.msg"
#   "msg/YourMessage2.msg"
#   "srv/YourService.srv"
# )

# Include C++ directories
include_directories(include)

# Declare a C++ executable
add_executable(controlandpublishfromHTPA src/controlandpublishfromHTPA.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(controlandpublishfromHTPA rclcpp std_msgs)

# Install executables and/or libraries
install(TARGETS
  controlandpublishfromHTPA
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Uncomment the following section if your package has services, actions, or messages
# rosidl_target_interfaces(initialize_htpa
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

ament_package()




