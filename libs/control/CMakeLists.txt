define_stereovisionapp_lib(CONTROL Control)

set(LIB_SRC
	application.h
	application.cpp
	mainwindow.cpp
	mainwindow.h
	mainwindow.ui
	actionmanager.cpp
	actionmanager.h
	imagebaseactions.cpp
	imagebaseactions.h
	imagebaseactionmanager.cpp
	imagebaseactionmanager.h
	camerabaseactionmanager.cpp
	camerabaseactionmanager.h
	landmarkbasedactions.cpp
	landmarkbasedactions.h
	landmarkbaseactionmanager.cpp
	landmarkbaseactionmanager.h
        stereorigactions.cpp
        stereorigactions.h
	stereorigactionmanager.cpp
	stereorigactionmanager.h
	angleconstrainactionmanager.cpp
	angleconstrainactionmanager.h
	distanceconstrainactionmanager.cpp
	distanceconstrainactionmanager.h
	solversactions.cpp
	solversactions.h
	exportactions.cpp
	exportactions.h
	cameracalibrationactionmanager.cpp
	cameracalibrationactionmanager.h
	cameracalibrationactions.cpp
	cameracalibrationactions.h
	localcoordinatesystembaseactionmanager.cpp
	localcoordinatesystembaseactionmanager.h
	localcoordinatesystemactions.cpp
	localcoordinatesystemactions.h
	fixedstereosequenceactionmanager.cpp
	fixedstereosequenceactionmanager.h
)

add_library(
${CONTROL_LIB}
${LIB_SRC}
)


target_link_libraries(${CONTROL_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${CONTROL_LIB} PRIVATE exiv2)

target_link_libraries(${CONTROL_LIB} PRIVATE ${STEREOVISION_LIB} ${DATA_LIB} ${VISION_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} Threads::Threads)
