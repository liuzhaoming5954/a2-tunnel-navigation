cmake_minimum_required(VERSION 3.16)
project(a2_platform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(A2_ENABLE_UNITREE_SDK "Build the optional receive-only Unitree SDK adapter" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(a2_interfaces REQUIRED)

add_library(a2_state_core src/state_validation.cpp)
target_include_directories(a2_state_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(a2_state_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(readonly_node src/readonly_node.cpp)
target_link_libraries(readonly_node a2_state_core)
ament_target_dependencies(readonly_node rclcpp rcl_interfaces diagnostic_msgs a2_interfaces)
target_compile_options(readonly_node PRIVATE -Wall -Wextra -Wpedantic)

if(A2_ENABLE_UNITREE_SDK)
  find_package(unitree_sdk2 REQUIRED)
  target_sources(readonly_node PRIVATE src/unitree_state_source.cpp)
  target_link_libraries(readonly_node unitree_sdk2)
  target_compile_definitions(readonly_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
endif()

install(TARGETS a2_state_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS readonly_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state test/test_state.cpp)
  target_link_libraries(test_state a2_state_core)
endif()
ament_package()
