define_stereovisionapp_lib(PROCESSING Processing)

set(LIB_SRC steppedprocess.cpp
)

set(LIB_HEADERS
    steppedprocess.h
)

add_library(
    ${PROCESSING_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)


target_link_libraries(${PROCESSING_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)

install_stereovisionapp_sdklib(${PROCESSING_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
