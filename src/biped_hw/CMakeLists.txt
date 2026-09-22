# biped_hw — CMakeLists.txt
# ament/CMake 构建：共享库 + P0 冒烟、P1 达妙/Motorevo 使能、
# P2 达妙 MIT 阶梯 / Motorevo 伺服跟踪、P3 同周期同步 / 双膝慢摆、
# MBS RS485 编码器接收 mbs_encoder_recv、P4 p4_state_dump，并安装 config/udev。
cmake_minimum_required(VERSION 3.8)
project(biped_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/usb_can_adapter.cpp
  src/bus_cycle.cpp
  src/p3_runtime.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_executable(hw_smoke_dual_bus apps/hw_smoke_dual_bus.cpp)
target_link_libraries(hw_smoke_dual_bus ${PROJECT_NAME})

# Same source, two defaults: id6 is the P1 first-gate (left knee DM10422).
add_executable(p1_left_dm_id3 apps/p1_left_dm_id3.cpp)
target_link_libraries(p1_left_dm_id3 ${PROJECT_NAME})
target_compile_definitions(p1_left_dm_id3 PRIVATE P1_DEFAULT_CAN_ID=3)

add_executable(p1_left_dm_id6 apps/p1_left_dm_id3.cpp)
target_link_libraries(p1_left_dm_id6 ${PROJECT_NAME})
target_compile_definitions(p1_left_dm_id6 PRIVATE P1_DEFAULT_CAN_ID=6)

add_executable(p1_left_evo_id8 apps/p1_left_evo_id8.cpp)
target_link_libraries(p1_left_evo_id8 ${PROJECT_NAME})

add_executable(p2_dm_mit_ladder apps/p2_dm_mit_ladder.cpp)
target_link_libraries(p2_dm_mit_ladder ${PROJECT_NAME})

add_executable(p2_evo_servo_track apps/p2_evo_servo_track.cpp)
target_link_libraries(p2_evo_servo_track ${PROJECT_NAME})

add_executable(p3_left6_sync apps/p3_left6_sync.cpp)
target_link_libraries(p3_left6_sync ${PROJECT_NAME})

add_executable(p3_dual12_sync apps/p3_dual12_sync.cpp)
target_link_libraries(p3_dual12_sync ${PROJECT_NAME})

add_executable(p3_fault_inject apps/p3_fault_inject.cpp)
target_link_libraries(p3_fault_inject ${PROJECT_NAME})

add_executable(p3_dual_knee_spin apps/p3_dual_knee_spin.cpp)
target_link_libraries(p3_dual_knee_spin ${PROJECT_NAME})

add_executable(mbs_encoder_recv apps/mbs_encoder_recv.cpp)
target_link_libraries(mbs_encoder_recv ${PROJECT_NAME})
target_compile_definitions(mbs_encoder_recv PRIVATE
  BIPED_HW_SOURCE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

add_executable(p4_state_dump apps/p4_state_dump.cpp)
target_link_libraries(p4_state_dump ${PROJECT_NAME})
ament_target_dependencies(p4_state_dump rclcpp sensor_msgs)
target_compile_definitions(p4_state_dump PRIVATE
  BIPED_HW_SOURCE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS hw_smoke_dual_bus p1_left_dm_id3 p1_left_dm_id6 p1_left_evo_id8
        p2_dm_mit_ladder p2_evo_servo_track p3_left6_sync p3_dual12_sync p3_fault_inject
        p3_dual_knee_spin mbs_encoder_recv p4_state_dump
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

ament_package()
