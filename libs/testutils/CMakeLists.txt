define_stereovisionapp_lib(TESTUTILS TestUtils)

set(LIB_SRC
     datablocks/generatedtrajectory.cpp
)

set(LIB_HEADERS
    datablocks/generatedtrajectory.h
)

add_library(
    ${TESTUTILS_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)

target_link_libraries(${TESTUTILS_LIB} PUBLIC Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Gui)
target_link_libraries(${TESTUTILS_LIB} PUBLIC ${PROCESSING_LIB} ${DATA_LIB} StereoVision::stevi)

install_stereovisionapp_sdklib(${TESTUTILS_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

foreach ( file ${LIB_HEADERS} )
    get_filename_component( dir ${file} DIRECTORY )
    install( FILES ${file} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/${dir} )
endforeach()
