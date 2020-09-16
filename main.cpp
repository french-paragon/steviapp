#include "mainwindow.h"

#include <QApplication>

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"

#include "gui/imageeditor.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	StereoVisionApp::MainWindow w;

	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::LandmarkFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::ImageFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::CameraFactory(&a));

	w.installEditor(new StereoVisionApp::ImageEditorFactory(&a));

	w.show();
	return a.exec();
}
