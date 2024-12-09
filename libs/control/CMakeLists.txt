define_stereovisionapp_lib(CONTROL Control)

configure_file(application_config.h.in ${CONFIGURED_INCLUDES_DIR}/control/application_config.h)

set(LIB_SRC
        utils_functions.cpp
        application.cpp
        stereoappplugininterface.cpp
        mainwindow.cpp
	mainwindow.ui
        actionmanager.cpp
        datatableactions.cpp
        datatableactionmanager.cpp
        imagebaseactions.cpp
        imagebaseactionmanager.cpp
        camerabaseactionmanager.cpp
        landmarkbasedactions.cpp
        landmarkbaseactionmanager.cpp
        stereorigactions.cpp
        stereorigactionmanager.cpp
        angleconstrainactionmanager.cpp
        distanceconstrainactionmanager.cpp
        solversactions.cpp
        exportactions.cpp
        cameracalibrationactionmanager.cpp
        cameracalibrationactions.cpp
        localcoordinatesystembaseactionmanager.cpp
        localcoordinatesystemactions.cpp
        fixedstereosequenceactions.cpp
        fixedstereosequenceactionmanager.cpp
        trajectoryactions.cpp
        trajectoryactionmanager.cpp
        correspondencessetactionsmanager.cpp
        correspondencessetactions.cpp
        projectactions.cpp
)

set(LIB_HEADERS
    ${CONFIGURED_INCLUDES_DIR}/control/application_config.h
    utils_functions.h
    application.h
    stereoappplugininterface.h
    mainwindow.h
    actionmanager.h
    datatableactions.h
    datatableactionmanager.h
    imagebaseactions.h
    imagebaseactionmanager.h
    camerabaseactionmanager.h
    landmarkbasedactions.h
    landmarkbaseactionmanager.h
    stereorigactions.h
    stereorigactionmanager.h
    angleconstrainactionmanager.h
    distanceconstrainactionmanager.h
    solversactions.h
    exportactions.h
    cameracalibrationactionmanager.h
    cameracalibrationactions.h
    localcoordinatesystembaseactionmanager.h
    localcoordinatesystemactions.h
    fixedstereosequenceactions.h
    fixedstereosequenceactionmanager.h
    trajectoryactions.h
    trajectoryactionmanager.h
    correspondencessetactionsmanager.h
    correspondencessetactions.h
    projectactions.h
)

if (ADD_DEVEL_TOOLS)
    list (APPEND LIB_SRC cornerdetectionactions.cpp)
    list (APPEND LIB_HEADERS cornerdetectionactions.h)
endif()

add_library(
${CONTROL_LIB}
${LIB_SRC}
${LIB_HEADERS}
)

target_link_libraries(${CONTROL_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${CONTROL_LIB} PRIVATE exiv2)

target_link_libraries(${CONTROL_LIB} PRIVATE StereoVision::stevi ${DATA_LIB} ${VISION_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} Threads::Threads)

install_stereovisionapp_sdklib(${CONTROL_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )
