#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"
#include "datablocks/stereorig.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"
#include "datablocks/cameracalibration.h"
#include "datablocks/localcoordinatesystem.h"

#include "gui/imageeditor.h"
#include "gui/imagepointdetailseditor.h"
#include "gui/landmarkpointdetailseditor.h"
#include "gui/localcoordinatesystempointdetailseditor.h"
#include "gui/sparsealignementeditor.h"
#include "gui/lenseditor.h"
#include "gui/cameracalibrationeditor.h"
#include "gui/cameracalibrationsparsealignementeditor.h"

#include "control/imagebaseactionmanager.h"
#include "control/camerabaseactionmanager.h"
#include "control/landmarkbaseactionmanager.h"
#include "control/stereorigactionmanager.h"
#include "control/angleconstrainactionmanager.h"
#include "control/distanceconstrainactionmanager.h"
#include "control/cameracalibrationactionmanager.h"
#include "control/localcoordinatesystembaseactionmanager.h"

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
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::StereoRigFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::LocalCoordinateSystemFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::AngleConstrainFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::DistanceConstrainFactory(&a));
	StereoVisionApp::ProjectFactory::defaultProjectFactory().addType(new StereoVisionApp::CameraCalibrationFactory(&a));

	StereoVisionApp::ProjectFactory* pF = &StereoVisionApp::ProjectFactory::defaultProjectFactory();

	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::ImageBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::LandmarkBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::CameraBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::StereoRigActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::LocalCoordinateSystemBaseActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::AngleConstrainActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::DistanceConstrainActionManager(pF));
	StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary().registerDatablockActionManager(new StereoVisionApp::CameraCalibrationActionManager(pF));

	StereoVisionApp::MainWindow w;

	w.installEditor(new StereoVisionApp::ImageEditorFactory(&a));
	w.installEditor(new StereoVisionApp::ImagePointDetailsEditorFactory(&a));
	w.installEditor(new StereoVisionApp::LandmarkPointDetailsEditorFactory(&a));
	w.installEditor(new StereoVisionApp::LocalCoordinateSystemPointDetailsEditorFactory(&a));
	w.installEditor(new StereoVisionApp::SparseAlignementEditorFactory(&a));
	w.installEditor(new StereoVisionApp::LensEditorFactory(&a));
	w.installEditor(new StereoVisionApp::CameraCalibrationEditorFactory(&a));
	w.installEditor(new StereoVisionApp::CameraCalibrationSparseAlignementEditorFactory(&a));

	if (argc > 1) {
		w.openProject(QString(argv[1]));
	}

	w.show();
	return a.exec();
}
