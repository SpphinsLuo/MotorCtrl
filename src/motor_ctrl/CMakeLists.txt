# motor_ctrl — CMakeLists.txt
# 电机控制底层：USB-CAN、达妙 MIT、Motorevo HA60、轨迹和安全控制。
cmake_minimum_required(VERSION 3.8)
project(motor_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mbs_encoder REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/usb_can_adapter.cpp
  src/bus_cycle.cpp
  src/dm_mit_host.cpp
  src/p3_runtime.cpp
  src/safety.cpp
  src/hip_parallel_kin.cpp
  src/quintic_traj.cpp
  src/algo_api.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${mbs_encoder_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

add_executable(hw_smoke_dual_bus apps/hw_smoke_dual_bus.cpp)
target_link_libraries(hw_smoke_dual_bus ${PROJECT_NAME})

add_executable(p1_dm_communicate apps/p1_dm_communicate.cpp)
target_link_libraries(p1_dm_communicate ${PROJECT_NAME})

add_executable(p1_evo_communicate apps/p1_evo_communicate.cpp)
target_link_libraries(p1_evo_communicate ${PROJECT_NAME})

add_executable(p2_dm_test apps/p2_dm_test.cpp)
target_link_libraries(p2_dm_test ${PROJECT_NAME})

add_executable(p2_evo_test apps/p2_evo_test.cpp)
target_link_libraries(p2_evo_test ${PROJECT_NAME})

add_executable(p3_right6_sync apps/p3_right6_sync.cpp)
target_link_libraries(p3_right6_sync ${PROJECT_NAME})

add_executable(p3_dual12_sync apps/p3_dual12_sync.cpp)
target_link_libraries(p3_dual12_sync ${PROJECT_NAME})

add_executable(p3_fault_inject apps/p3_fault_inject.cpp)
target_link_libraries(p3_fault_inject ${PROJECT_NAME})

add_executable(p4_state_dump apps/p4_state_dump.cpp)
target_link_libraries(p4_state_dump ${PROJECT_NAME})
target_include_directories(p4_state_dump PRIVATE ${mbs_encoder_INCLUDE_DIRS})
ament_target_dependencies(p4_state_dump rclcpp sensor_msgs std_msgs)
target_link_libraries(p4_state_dump
  mbs_encoder::mbs_encoder__rosidl_typesupport_cpp)

add_executable(p4_hip_kin_self_test apps/p4_hip_kin_self_test.cpp)
target_link_libraries(p4_hip_kin_self_test ${PROJECT_NAME})

add_executable(p5_safety_self_test apps/p5_safety_self_test.cpp)
target_link_libraries(p5_safety_self_test ${PROJECT_NAME})

add_executable(p6_traj_track apps/p6_traj_track.cpp)
target_link_libraries(p6_traj_track ${PROJECT_NAME})
target_include_directories(p6_traj_track PRIVATE ${mbs_encoder_INCLUDE_DIRS})
ament_target_dependencies(p6_traj_track rclcpp sensor_msgs std_msgs)
target_link_libraries(p6_traj_track
  mbs_encoder::mbs_encoder__rosidl_typesupport_cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS hw_smoke_dual_bus p1_dm_communicate p1_evo_communicate
        p2_dm_test p2_evo_test p3_right6_sync p3_dual12_sync
        p3_fault_inject p4_state_dump p4_hip_kin_self_test
        p5_safety_self_test p6_traj_track
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY udev
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads mbs_encoder)

ament_package()
