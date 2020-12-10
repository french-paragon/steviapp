define_stereovisionapp_lib(GEOMETRY Geometry)

set(LIB_SRC core.cpp
            core.h
            geometricexception.cpp
            geometricexception.h
            pointcloudalignment.cpp
            pointcloudalignment.h
            rotations.cpp
            rotations.h
            alignement.cpp
            alignement.h
)

add_library(
    ${GEOMETRY_LIB}
    ${LIB_SRC}
)


target_link_libraries(${GEOMETRY_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
