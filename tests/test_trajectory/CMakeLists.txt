add_executable(testTrajectory main.cpp)
target_link_libraries(testTrajectory ${DATA_LIB}  Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestTrajectory testTrajectory)
