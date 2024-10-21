define_stereovisionapp_lib(VISION Vision)

set(LIB_SRC
            imageio.cpp
            checkboarddetector.cpp
            pointcloudio.cpp
)

set(LIB_HEADERS
    vision_global.h
    imageio.h
    checkboarddetector.h
    pointcloudio.h
    indexed_timed_sequence.h
    trajectoryImuPreIntegration.h
)

add_library(
    ${VISION_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)

target_link_libraries(${VISION_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Gui)
target_link_libraries(${VISION_LIB} PRIVATE ${PROCESSING_LIB} StereoVision::stevi)

install_stereovisionapp_sdklib(${VISION_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
