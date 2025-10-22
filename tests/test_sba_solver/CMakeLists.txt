add_executable(testSBASolver main.cpp
    test_sba_data.qrc)
target_link_libraries(testSBASolver StereoVision::stevi ${UTILS_LIB} Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Test ${DATA_LIB} ${SOLVING_LIB} ${CERES_LIBRARIES})
add_test(TestSBASolver testSBASolver)
