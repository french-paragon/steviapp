define_stereovisionapp_lib(GUI Gui)

set(LIB_SRC imagewidget.cpp
            editor.cpp
            imageadapters/imagedatablockdisplayadapter.cpp
            imagedisplayoverlays/imagelandmarksoverlay.cpp
            imageeditor.cpp
            imageeditor.ui
            imagepointdetailseditor.cpp
            imagepointdetailseditor.ui
            landmarkpointdetailseditor.cpp
            landmarkpointdetailseditor.ui
            localcoordinatesystempointdetailseditor.cpp
            localcoordinatesystempointdetailseditor.ui
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
)

set(LIB_HEADERS
    imagewidget.h
    editor.h
    imageeditor.h
    imagepointdetailseditor.h
    landmarkpointdetailseditor.h
    localcoordinatesystempointdetailseditor.h
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
)

set(LIB_HEADERS_IMGADAPTERS
    imageadapters/imagedatablockdisplayadapter.h
)

set(LIB_HEADERS_IMGDISPOVERLAYS
    imagedisplayoverlays/imagelandmarksoverlay.h
)
list (APPEND files ${files2})

add_library(
    ${GUI_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
    ${LIB_HEADERS_IMGADAPTERS}
    ${LIB_HEADERS_IMGDISPOVERLAYS}
)


target_link_libraries(${GUI_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${GUI_LIB} PRIVATE ${DATA_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} ${QIMAGEDISPLAYWIDGET_LIB})

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
