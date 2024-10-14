define_stereovisionapp_lib(UTILS Utils)

set(LIB_SRC
)

set(LIB_HEADERS
    statusoptionalreturn.h
)

add_library(
    ${UTILS_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)

install_stereovisionapp_sdklib(${UTILS_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
