cmake_minimum_required(VERSION 3.22)
project(glim_odometry_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(glim REQUIRED)
find_package(GTest REQUIRED)
enable_testing()
add_executable(test_odometry_ct_vgicp test_odometry_ct_vgicp.cpp)
# Use checkout headers: the current standalone export adds an extra /glim
# to its include prefix, while the installed headers keep the glim/ directory.
target_include_directories(test_odometry_ct_vgicp PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(test_odometry_ct_vgicp PRIVATE glim::glim GTest::gtest)
add_test(NAME odometry_ct_vgicp COMMAND test_odometry_ct_vgicp
  "${CMAKE_CURRENT_SOURCE_DIR}/../config" "$<TARGET_FILE:glim::odometry_estimation_ct_vgicp>")
