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

message(STATUS "Open3D include dir: ${Open3D_INCLUDE_DIRS}")

add_executable(vf_control src/vf_control.cpp)

ament_target_dependencies(vf_control rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs visualization_msgs)
include_directories(vf_control PUBLIC include)

# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# ${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
# ${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
# )

target_link_libraries(vf_control Eigen3::Eigen Open3D::Open3D)

install(TARGETS ${This} vf_control
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
