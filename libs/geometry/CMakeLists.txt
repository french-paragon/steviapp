define_stereovisionapp_lib(GEOMETRY Geometry)

set(LIB_SRC pointcloudalignment.cpp
            pointcloudalignment.h
)

add_library(
    ${GEOMETRY_LIB}
    ${LIB_SRC}
)


target_link_libraries(${GEOMETRY_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
