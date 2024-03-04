
add_executable(testCore testCore.cpp)
target_link_libraries(testCore Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestCore testCore)

add_executable(testReprojection testProjection.cpp)
target_link_libraries(testReprojection Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestReprojection testReprojection)

add_executable(testRotations testRotations.cpp)
target_link_libraries(testRotations Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestRotations testRotations)


add_executable(testPointCloudAlignement testPointCloudAlignement.cpp)
target_link_libraries(testPointCloudAlignement Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestPointCloudAlignement testPointCloudAlignement)


add_executable(testLenseDistortion testLenseDistortion.cpp)
target_link_libraries(testLenseDistortion Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestLenseDistortion testLenseDistortion)
