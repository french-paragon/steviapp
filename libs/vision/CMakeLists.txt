define_stereovisionapp_lib(VISION Vision)

set(LIB_SRC vision_global.h
            imageio.cpp
            imageio.h
            interpolation.cpp
            interpolation.h
            lensdistortionsmap.cpp
            lensdistortionsmap.h
)

add_library(
    ${VISION_LIB}
    ${LIB_SRC}
)

target_link_libraries(${VISION_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Gui)
target_link_libraries(${VISION_LIB} PRIVATE ${GEOMETRY_LIB})
