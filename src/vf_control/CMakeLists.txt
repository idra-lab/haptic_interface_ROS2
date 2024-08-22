cmake_minimum_required(VERSION 3.5)
project(vf_control)


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
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raptor_api_interfaces REQUIRED)
find_package(haption_raptor_api REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D QUIET)
if (Open3D_FOUND)
    message(STATUS "Found Open3D installed locally")
else()
    message(STATUS "Open3D not found, downloading...")
    include(FetchContent)
    FetchContent_Declare(
        Open3D
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL "https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz"
    )
    FetchContent_MakeAvailable(Open3D)
    find_package(Open3D REQUIRED HINTS ${open3d_SOURCE_DIR})
endif()

set (QPOASES_INSTALL_DIR ${CMAKE_BINARY_DIR}/qpoases_install)
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




# message(STATUS "Open3D include dir: ${Open3D_INCLUDE_DIRS}")

add_executable(vf_control src/vf_control.cpp)
add_executable(vf_control_test src/vf_control_test.cpp)
add_executable(mesh_test src/mesh_test.cpp)

ament_target_dependencies(vf_control rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(vf_control_test rclcpp tf2_geometry_msgs visualization_msgs raptor_api_interfaces)

include_directories(vf_control PUBLIC include)
include_directories(mesh_test PUBLIC include)


# target_include_directories(mesh_test ${Open3D_INCLUDE_DIRS})
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# ${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
# ${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
# )

target_link_libraries(vf_control Eigen3::Eigen Open3D::Open3D qpoases::qpoases)
target_link_libraries(vf_control_test Eigen3::Eigen Open3D::Open3D qpoases::qpoases)
target_link_libraries(mesh_test Eigen3::Eigen Open3D::Open3D)

install(TARGETS ${This} vf_control
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${This} mesh_test vf_control_test
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
