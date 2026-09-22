cmake_minimum_required(VERSION 3.8)
project(motor_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(motorevo_PV_test src/motorevo_PV_test.cpp)

add_executable(single_motor_ctrl src/single_motor_ctrl.cpp)

add_executable(single_motor_ctrl_right src/single_motor_ctrl_right.cpp)

add_executable(test01_ssj src/test01_ssj.cpp)

ament_target_dependencies(motorevo_PV_test
rclcpp
std_msgs

)

ament_target_dependencies(single_motor_ctrl
rclcpp
std_msgs

)

ament_target_dependencies(single_motor_ctrl_right
rclcpp
std_msgs

)

ament_target_dependencies(test01_ssj
rclcpp
std_msgs

)

install(TARGETS
motorevo_PV_test
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
single_motor_ctrl
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
single_motor_ctrl_right
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
test01_ssj
DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
