define_stereovisionapp_lib(GUI Gui)

set(LIB_SRC imagewidget.cpp
            editor.cpp
            imageadapters/imagedatablockdisplayadapter.cpp
            imageadapters/imagedatadisplayadapter.cpp
            imagedisplayoverlays/imagelandmarksoverlay.cpp
            imagedisplayoverlays/pointssetoverlay.cpp
            imagedisplayoverlays/labelledpointsoverlay.cpp
            openGlDrawables/opengldrawablescenegrid.cpp
            openGlDrawables/opengldrawablelandmarkset.cpp
            openGlDrawables/opengldrawablecamerasset.cpp
            openGlDrawables/opengldrawablelocalcoordinatesystem.cpp
            openGlDrawables/opengldrawabletrajectory.cpp
            imageeditor.cpp
            imageeditor.ui
            imageviewer.cpp
            imagepointdetailseditor.cpp
            imagepointdetailseditor.ui
            landmarkpointdetailseditor.cpp
            landmarkpointdetailseditor.ui
            trajectorysequencevieweditor.cpp
            trajectorycomparisoneditor.cpp
            trajectoryalignementanalysiseditor.cpp
            localcoordinatesystempointdetailseditor.cpp
            localcoordinatesystempointdetailseditor.ui
            opengl3dsceneviewwidget.cpp
            sparsealignementviewer.cpp
            sparsealignementeditor.cpp
            sparsealignementeditor.ui
            lensdistortionviewer.cpp
            lenseditor.cpp
            lenseditor.ui
            gl_shaders.qrc
            stepprocessmonitorbox.cpp
            stepprocessmonitorbox.ui
            sparsesolverconfigdialog.cpp
            sparsesolverconfigdialog.ui
            sparsestereosolverconfigdialog.cpp
            sparsestereosolverconfigdialog.ui
            solutioninitconfigdialog.cpp
            solutioninitconfigdialog.ui
            rectifiedimageexportoptionsdialog.cpp
            rectifiedimageexportoptionsdialog.ui
            cameracalibrationeditor.cpp
            cameracalibrationeditor.ui
            cameracalibrationsparsealignementviewerinterface.cpp
            cameracalibrationsparsealignementeditor.cpp
            cameracalibrationsparsealignementeditor.ui
            cameracalibrationoptionsdialog.cpp
            cameracalibrationoptionsdialog.ui
            hexagonaltargetdetectionoptionsdialog.cpp
            hexagonaltargetdetectionoptionsdialog.ui
            fixedparametersoptionwidget.cpp
            fixedparametersoptionwidget.ui
            fixedstereosequenceeditor.cpp
            fixedstereosequenceeditor.ui
            stereosequenceexportoptiondialog.cpp
            stereosequenceexportoptiondialog.ui
            stereosequenceimageexportoptiondialog.cpp
            stereosequenceimageexportoptiondialog.ui
            datatablevieweditor.cpp
            dialogs/trajectoryeditpositionoptionsdialog.cpp
            dialogs/trajectoryeditorientationoptionsdialog.cpp
            dialogs/trajectoryeditaccelerometeroptionsdialog.cpp
            dialogs/trajectoryeditgyrooptionsdialog.cpp
            inputsWidgets/rigidbodytransforminputwidget.h
            inputsWidgets/rigidbodytransforminputwidget.cpp
)

set(LIB_HEADERS
    imagewidget.h
    editor.h
    imageeditor.h
    imageviewer.h
    imagepointdetailseditor.h
    landmarkpointdetailseditor.h
    localcoordinatesystempointdetailseditor.h
    opengl3dsceneviewwidget.h
    sparsealignementviewer.h
    sparsealignementeditor.h
    lensdistortionviewer.h
    lenseditor.h
    stepprocessmonitorbox.h
    sparsesolverconfigdialog.h
    sparsestereosolverconfigdialog.h
    solutioninitconfigdialog.h
    rectifiedimageexportoptionsdialog.h
    cameracalibrationeditor.h
    cameracalibrationsparsealignementviewerinterface.h
    cameracalibrationsparsealignementeditor.h
    cameracalibrationoptionsdialog.h
    hexagonaltargetdetectionoptionsdialog.h
    fixedparametersoptionwidget.h
    fixedstereosequenceeditor.h
    stereosequenceexportoptiondialog.h
    stereosequenceimageexportoptiondialog.h
    datatablevieweditor.h
    trajectorysequencevieweditor.h
    trajectorycomparisoneditor.h
    trajectoryalignementanalysiseditor.h
)

set(LIB_HEADERS_IMGADAPTERS
    imageadapters/imagedatablockdisplayadapter.h
    imageadapters/imagedatadisplayadapter.h
)

set(LIB_HEADERS_IMGDISPOVERLAYS
    imagedisplayoverlays/imagelandmarksoverlay.h
    imagedisplayoverlays/pointssetoverlay.h
    imagedisplayoverlays/labelledpointsoverlay.h
)

set(LIB_HEADERS_OPENGLDRAWABLES
    openGlDrawables/opengldrawablescenegrid.h
    openGlDrawables/opengldrawablelandmarkset.h
    openGlDrawables/opengldrawablecamerasset.h
    openGlDrawables/opengldrawablelocalcoordinatesystem.h
    openGlDrawables/opengldrawabletrajectory.h
)

set(LIB_HEADERS_DIALOG
    dialogs/trajectoryeditpositionoptionsdialog.h
    dialogs/trajectoryeditorientationoptionsdialog.h
    dialogs/trajectoryeditaccelerometeroptionsdialog.h
    dialogs/trajectoryeditgyrooptionsdialog.h
)

if (ADD_DEVEL_TOOLS)
    list (APPEND LIB_SRC cornerdetectortesteditor.cpp
        cornermatchingtesteditor.cpp)
    list (APPEND LIB_HEADERS cornerdetectortesteditor.h
        cornermatchingtesteditor.h)
endif()

add_library(
    ${GUI_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
    ${LIB_HEADERS_IMGADAPTERS}
    ${LIB_HEADERS_IMGDISPOVERLAYS}
    ${LIB_HEADERS_OPENGLDRAWABLES}
    ${LIB_HEADERS_DIALOG}
)


target_link_libraries(${GUI_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${GUI_LIB} PRIVATE ${DATA_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} ${QCUSTOMPLOT_LIB} StereoVision::qImageWidget)

install_stereovisionapp_sdklib(${GUI_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )

install(FILES ${LIB_HEADERS_IMGADAPTERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/imageadapters
  )

install(FILES ${LIB_HEADERS_IMGDISPOVERLAYS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/imagedisplayoverlays
  )

install(FILES ${LIB_HEADERS_OPENGLDRAWABLES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/openGlDrawables
  )

install(FILES ${LIB_HEADERS_DIALOG}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/dialogs
  )
