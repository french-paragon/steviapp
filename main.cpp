#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"

#include "gui/imageeditor.h"
#include "gui/sparsealignementeditor.h"
#include "gui/lenseditor.h"

#include "control/imagebaseactionmanager.h"
#include "control/camerabaseactionmanager.h"
#include "control/landmarkbaseactionmanager.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	//surface setup

	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	format.setStencilBufferSize(8);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setSamples(4);
	QSurfaceFormat::setDefaultFormat(format);

	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::LandmarkFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::ImageFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::CameraFactory(&a));

	StereoVisionApp::ProjectFactory* pF = &StereoVisionApp::ProjectFactory::defaultProjectFactory();

	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::ImageBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::LandmarkBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::CameraBaseActionManager(pF));

	StereoVisionApp::MainWindow w;

	w.installEditor(new StereoVisionApp::ImageEditorFactory(&a));
	w.installEditor(new StereoVisionApp::SparseAlignementEditorFactory(&a));
	w.installEditor(new StereoVisionApp::LensEditorFactory(&a));

	if (argc > 1) {
		w.openProject(QString(argv[1]));
	}

	w.show();
	return a.exec();
}
