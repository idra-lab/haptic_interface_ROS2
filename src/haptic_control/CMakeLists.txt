cmake_minimum_required(VERSION 3.5)
set(This haptic_control)
project(haptic_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raptor_api_interfaces REQUIRED)
find_package(haption_raptor_api REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(virtual_fixture_msgs REQUIRED)
# find eigen
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(${This} src/main_haptic_control.cpp src/haptic_control.cpp)
add_executable(virtual_fixture_control src/main_vf_control.cpp src/virtual_fixture_control.cpp)

ament_target_dependencies(${This} rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs)
ament_target_dependencies(virtual_fixture_control rclcpp raptor_api_interfaces haption_raptor_api tf2_ros tf2_geometry_msgs virtual_fixture_msgs)
include_directories(${This} PUBLIC include)
include_directories(virtual_fixture_control PUBLIC include)

include_directories(${This} PUBLIC
include_directories(virtual_fixture_control PUBLIC)

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

if(MSVC)  
else()
	execute_process(COMMAND /usr/bin/ldd --version
		COMMAND head -n 1
		COMMAND /usr/bin/awk "{print $NF}"
		OUTPUT_STRIP_TRAILING_WHITESPACE
		OUTPUT_VARIABLE GLIBC)
	message("GLIBC=${GLIBC}")
    target_compile_definitions(${This} PUBLIC PROTOCOL_FB LINUX _OS_UNIX)
    set(CMAKE_C_FLAGS_RELEASE "-Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "-Wall")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()


install(TARGETS ${This} virtual_fixture_control
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_link_libraries(haptic_control Eigen3::Eigen)
ament_package()
