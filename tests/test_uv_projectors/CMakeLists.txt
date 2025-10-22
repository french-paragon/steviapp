add_executable(testUVProjection main.cpp)
target_link_libraries(testUVProjection ${UTILS_LIB} StereoVision::stevi Qt${QT_VERSION_MAJOR}::Test ${DATA_LIB} ${SOLVING_LIB} ${CERES_LIBRARIES})
add_test(TestUVProjection testUVProjection)
