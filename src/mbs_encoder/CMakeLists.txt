cmake_minimum_required(VERSION 3.8)
project(mbs_encoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MbsState.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_core SHARED
  src/serial_port.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)

add_executable(mbs_encoder_node apps/mbs_encoder_node.cpp)
target_link_libraries(mbs_encoder_node ${PROJECT_NAME}_core "${cpp_typesupport_target}")
ament_target_dependencies(mbs_encoder_node rclcpp std_msgs)
target_compile_definitions(mbs_encoder_node PRIVATE
  MBS_ENCODER_SOURCE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

add_executable(mbs_set_zero apps/mbs_set_zero.cpp)
target_link_libraries(mbs_set_zero ${PROJECT_NAME}_core)
target_compile_definitions(mbs_set_zero PRIVATE
  MBS_ENCODER_SOURCE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mbs_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mbs_set_zero
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads rosidl_default_runtime)

ament_package()
