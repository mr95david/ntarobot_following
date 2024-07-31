cmake_minimum_required(VERSION 3.8)
project(nodes_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(primer_nodocpp src/primer_nodocpp.cpp)
target_include_directories(primer_nodocpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(primer_nodocpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Agregar ejecutable del nodo creado
add_executable(simple_lifecycle_node src/cyclelife_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

# Agregar ejecutable del nodo creado
add_executable(follow_node src/follow_node.cpp)
ament_target_dependencies(follow_node rclcpp nav2_msgs rclcpp_action)

# Agregar ejecutable del nodo creado
add_executable(click_point src/click_point.cpp)
ament_target_dependencies(click_point rclcpp geometry_msgs)

# Agregar ejecutable del nodo creado
add_executable(human_follow src/human_follow.cpp)
ament_target_dependencies(human_follow rclcpp geometry_msgs nav2_msgs rclcpp_action)

install(TARGETS 
  primer_nodocpp
  simple_lifecycle_node
  follow_node
  click_point
  human_follow
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
