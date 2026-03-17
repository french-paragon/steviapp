add_executable(demo_spherical_pinhole_costs main.cpp)

target_link_libraries(demo_spherical_pinhole_costs StereoVision::stevi ${UTILS_LIB} ${DATA_LIB} ${SOLVING_LIB} ${CERES_LIBRARIES})
