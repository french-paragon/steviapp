add_executable(testTerrainProjector testTerrainProjector.cpp)
target_link_libraries(testTerrainProjector StereoVision::stevi ${GEO_LIB} Qt${QT_VERSION_MAJOR}::Test StereoVision::stevi)
add_test(TestTerrainProjector testTerrainProjector)
