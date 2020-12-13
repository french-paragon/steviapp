
add_executable(testCore testCore.cpp)
target_link_libraries(testCore ${GEOMETRY_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestCore testCore)

add_executable(testReprojection testProjection.cpp)
target_link_libraries(testReprojection ${GEOMETRY_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestReprojection testReprojection)

add_executable(testRotations testRotations.cpp)
target_link_libraries(testRotations ${GEOMETRY_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestRotations testRotations)


add_executable(testPointCloudAlignement testPointCloudAlignement.cpp)
target_link_libraries(testPointCloudAlignement ${GEOMETRY_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestPointCloudAlignement testPointCloudAlignement)
