add_executable(benchmark_sparse_cov_estimator main.cpp)

target_link_libraries(benchmark_sparse_cov_estimator StereoVision::stevi ${TESTUTILS_LIB} ${UTILS_LIB} Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Test ${DATA_LIB} ${SOLVING_LIB} ${CERES_LIBRARIES})

