
set(QCUSTOMPLOT_LIB qcustomplot PARENT_SCOPE)
set(QCUSTOMPLOT_LIB qcustomplot)

set(LIB_SRC qcustomplot.cpp)

set(LIB_HEADERS qcustomplot.h)

add_library(
    ${QCUSTOMPLOT_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS})

target_link_libraries(${QCUSTOMPLOT_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::PrintSupport)

target_compile_definitions(${QCUSTOMPLOT_LIB} PRIVATE -DQCUSTOMPLOT_COMPILE_LIBRARY)

add_library(${STEVIAPP_SDK_NAME}::${QCUSTOMPLOT_LIB} ALIAS ${QCUSTOMPLOT_LIB})

target_include_directories(${QCUSTOMPLOT_LIB} PUBLIC
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${QCUSTOMPLOT_LIB}
  EXPORT ${STEVIAPP_SDK_EXPORTNAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
