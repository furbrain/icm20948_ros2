cmake_minimum_required(VERSION 3.8)
project(icm20948_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

add_compile_definitions(ICM_20948_USE_DMP)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the ICM20948_I2CDev library subdirectory
add_subdirectory(ICM20948_I2CDev)

add_executable(icm20948_dpm
  src/icm20948_dpm.cpp
)

add_executable(icm20948_raw
  src/icm20948_raw.cpp
)

target_include_directories(icm20948_raw PUBLIC
  include/
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(icm20948_raw icm20948_i2cdev)
target_compile_features(icm20948_raw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(icm20948_raw rclcpp sensor_msgs)

target_include_directories(icm20948_dpm PUBLIC
  include/
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(icm20948_dpm icm20948_i2cdev)
target_compile_features(icm20948_dpm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(icm20948_dpm rclcpp sensor_msgs)

install(TARGETS icm20948_raw icm20948_dpm
  DESTINATION lib/${PROJECT_NAME})

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
