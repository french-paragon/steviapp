define_stereovisionapp_lib(GUI Gui)

set(LIB_SRC imagewidget.cpp
            imagewidget.h
            editor.cpp
            editor.h
            imageeditor.cpp
            imageeditor.h
            imageeditor.ui
            sparsealignementviewer.cpp
            sparsealignementviewer.h
            sparsealignementeditor.cpp
            sparsealignementeditor.h
            sparsealignementeditor.ui
            lensdistortionviewer.cpp
            lensdistortionviewer.h
            lenseditor.cpp
            lenseditor.h
            lenseditor.ui
            gl_shaders.qrc
            stepprocessmonitorbox.cpp
            stepprocessmonitorbox.h
            stepprocessmonitorbox.ui
            sparsesolverconfigdialog.cpp
            sparsesolverconfigdialog.h
            sparsesolverconfigdialog.ui
            solutioninitconfigdialog.cpp
            solutioninitconfigdialog.h
            solutioninitconfigdialog.ui
            rectifiedimageexportoptionsdialog.cpp
            rectifiedimageexportoptionsdialog.h
            rectifiedimageexportoptionsdialog.ui
)

add_library(
    ${GUI_LIB}
    ${LIB_SRC}
)


target_link_libraries(${GUI_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${GUI_LIB} PRIVATE ${DATA_LIB} ${PROCESSING_LIB})
