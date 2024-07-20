cmake_minimum_required(VERSION 3.8)
project(yaskawa_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(motoros2_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrajExecutor.action"
  DEPENDENCIES std_msgs trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


##### Generate the executables #### 
add_executable(executeTraj_action src/executeTraj_action.cpp)

target_include_directories(executeTraj_action PUBLIC include)

ament_target_dependencies(executeTraj_action
ament_cmake
rclcpp
trajectory_msgs
rclcpp_action
control_msgs
sensor_msgs
motoros2_interfaces
)

target_link_libraries(executeTraj_action "${cpp_typesupport_target}") 


##### Install directories and targets #### 
install(
  DIRECTORY include action src launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
    executeTraj_action
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
