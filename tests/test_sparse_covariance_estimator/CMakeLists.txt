add_executable(test_sparse_cov_estimator main.cpp)

target_link_libraries(test_sparse_cov_estimator StereoVision::stevi ${TESTUTILS_LIB} ${UTILS_LIB} Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Test ${DATA_LIB} ${SOLVING_LIB} ${CERES_LIBRARIES})

add_test(TestSparseCovEstimator test_sparse_cov_estimator)
