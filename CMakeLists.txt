cmake_minimum_required(VERSION 3.5)
project(rt2_assignment1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/Command.srv"
	"srv/RandomPosition.srv"
	"srv/Position.srv"
	DEPENDENCIES std_msgs
)

add_library(random_service SHARED
  src/position_service.cpp)

add_library(fsm_component SHARED
  src/state_machine.cpp)
  
rosidl_target_interfaces(random_service
	${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(fsm_component
	${PROJECT_NAME} "rosidl_typesupport_cpp")
  
rclcpp_components_register_node(random_service
  PLUGIN "rt2_assignment1::StateMachine"
  EXECUTABLE position_service) 

rclcpp_components_register_node(fsm_component
  PLUGIN "rt2_assignment1::FSM"
  EXECUTABLE comp_FSM)

# The executable can be run as standalone node while the "random_service" is the component
  
ament_target_dependencies(random_service rclcpp rclcpp_components std_msgs)
ament_target_dependencies(fsm_component rclcpp rclcpp_components std_msgs)

install(TARGETS
  random_service
  fsm_component
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib) #This lib has to be added for the components only
  
install(
	FILES rt2_mapping.yaml
	DESTINATION share/${PROJECT_NAME}
)
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
