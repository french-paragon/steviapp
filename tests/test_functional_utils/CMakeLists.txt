add_executable(testFunctionalUtils main.cpp)
target_link_libraries(testFunctionalUtils ${UTILS_LIB}  Qt${QT_VERSION_MAJOR}::Test ${CERES_LIBRARIES})
add_test(TestFunctionalUtils testFunctionalUtils)
