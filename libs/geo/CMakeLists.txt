define_stereovisionapp_lib(GEO Geo)

set(LIB_SRC)

set(LIB_HEADERS
    geoRaster.h
    terrainProjector.h
    localframes.h)

add_library(
    ${GEO_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
)

target_link_libraries(${GEO_LIB} PUBLIC PROJ::proj)

install_stereovisionapp_sdklib(${GEO_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
