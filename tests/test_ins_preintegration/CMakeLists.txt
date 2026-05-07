add_executable(test_ins_preintegration main.cpp)

target_link_libraries(test_ins_preintegration StereoVision::stevi Qt${QT_VERSION_MAJOR}::Test ${SOLVING_LIB})
