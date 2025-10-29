add_executable(testTestUtils main.cpp)
target_link_libraries(testTestUtils Qt${QT_VERSION_MAJOR}::Test ${TESTUTILS_LIB}) #no more targets, ensure that TESTUTILS_LIB pull all dependancies
add_test(TestTestUtils testTestUtils)
