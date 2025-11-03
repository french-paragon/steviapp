define_stereovisionapp_lib(UTILS Utils)

set(LIB_SRC
    editablefloatingpointblock.cpp
    optionlistvalue.cpp
)

set(LIB_HEADERS
    statusoptionalreturn.h
    functional.h
    inplace_vector.h
    editablefloatingpointblock.h
    optionlistvalue.h
)

add_library(
    ${UTILS_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)

install_stereovisionapp_sdklib(${UTILS_LIB})

target_link_libraries(${UTILS_LIB} Qt${QT_VERSION_MAJOR}::Core)

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
