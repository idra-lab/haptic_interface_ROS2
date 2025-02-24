cmake_minimum_required(VERSION 3.5)
project(haptic_control)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fopenmp)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raptor_api_interfaces REQUIRED)
find_package(haption_raptor_api REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vf_test test/vf_test.cpp)
  target_include_directories(vf_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(vf_test Eigen3::Eigen Open3D::Open3D qpoases::qpoases gomp)
endif()

### QP Oasis
set(QPOASES_INSTALL_DIR ${CMAKE_BINARY_DIR}/qpoases_install)
include(ExternalProject)
ExternalProject_Add(
  external_qpoases
  URL https://github.com/coin-or/qpOASES/archive/refs/tags/releases/3.2.1.tar.gz
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_DIR}
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
  -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
  -DBUILD_SHARED_LIBS=OFF
  -DQPOASES_BUILD_EXAMPLES=OFF
)
add_library(qpoases_helper INTERFACE)
add_dependencies(qpoases_helper external_qpoases)
target_include_directories(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/include)
target_link_libraries(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/lib/libqpOASES.a)
add_library(qpoases::qpoases ALIAS qpoases_helper)
##########

### Open3D 0.18.0
# if Open3d is not found, then do not build the haptic_control node
if(Open3D_FOUND)
  message(STATUS "Open3D found. Building haptic_control node.")
else()
  message(STATUS "Open3D not found. Downloading...")
  include(FetchContent)
  FetchContent_Declare(
    open3d
    URL https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
  )
  FetchContent_GetProperties(open3d)
  if(NOT open3d_POPULATED)
    FetchContent_Populate(open3d)
  endif()
  find_package(Open3D REQUIRED HINTS ${open3d_SOURCE_DIR})
  message(STATUS "Source dir: ${open3d_SOURCE_DIR}")
endif()
##########
add_executable(sample_teleoperation src/sample_teleoperation.cpp)
ament_target_dependencies(sample_teleoperation rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs)
include_directories(include)
target_link_libraries(sample_teleoperation Eigen3::Eigen)

add_executable(haptic_control src/haptic_control_base.cpp)
ament_target_dependencies(haptic_control rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs visualization_msgs)
include_directories(include)
target_link_libraries(haptic_control Eigen3::Eigen Open3D::Open3D qpoases::qpoases gomp)

add_executable(keyboard_teleoperation test/keyboard_teleoperation.cpp)
ament_target_dependencies(keyboard_teleoperation rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs visualization_msgs)
include_directories(include)
target_link_libraries(keyboard_teleoperation Eigen3::Eigen Open3D::Open3D qpoases::qpoases gomp)



install(TARGETS ${This} haptic_control sample_teleoperation keyboard_teleoperation
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
