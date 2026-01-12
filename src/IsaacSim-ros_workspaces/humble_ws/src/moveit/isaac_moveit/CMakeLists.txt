cmake_minimum_required(VERSION 3.5)
project(isaac_moveit)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 패키지 의존성 탐색
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # 추가: 코드 내 tf2 사용 시 필요

# --- Executable: pick_and_place ---
add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  tf2_geometry_msgs
)

# --- Executable: pick_and_place_tm12 ---
add_executable(pick_and_place_tm12 src/pick_and_place_tm12.cpp)
target_include_directories(pick_and_place_tm12 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pick_and_place_tm12
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  tf2_geometry_msgs
)

# --- 설치(Install) 설정 ---

# 1. 실행 파일 설치
install(TARGETS
  pick_and_place
  pick_and_place_tm12
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 설정 및 리소스 폴더 설치 (여기가 핵심 수정 사항입니다)
install(
  DIRECTORY 
    tm_config
    tm_description   # 이 줄이 빠져 있어서 에러가 났던 것입니다
    rviz2
    launch
  DESTINATION share/${PROJECT_NAME}
)

# 3. 추가적인 의존성 전파
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
)

ament_package()
