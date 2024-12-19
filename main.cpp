#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#include "control/application.h"
#include "control/mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

#include "control/application_config.h"

#include "datablocks/project.h"
#include "datablocks/datatable.h"
#include "datablocks/landmark.h"
#include "datablocks/trajectory.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"
#include "datablocks/correspondencesset.h"
#include "datablocks/stereorig.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"
#include "datablocks/cameracalibration.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/fixedcolorstereosequence.h"
#include "datablocks/fixedstereopluscolorsequence.h"

#include "gui/datatablevieweditor.h"
#include "gui/imageeditor.h"
#include "gui/imageviewer.h"
#include "gui/imagepointdetailseditor.h"
#include "gui/landmarkpointdetailseditor.h"
#include "gui/localcoordinatesystempointdetailseditor.h"
#include "gui/sparsealignementeditor.h"
#include "gui/lenseditor.h"
#include "gui/cameracalibrationeditor.h"
#include "gui/cameracalibrationsparsealignementeditor.h"
#include "gui/fixedstereosequenceeditor.h"
#include "gui/trajectorysequencevieweditor.h"
#include "gui/trajectoryoptanalysiseditor.h"
#include "gui/trajectoryalignementanalysiseditor.h"

#ifdef STEVIAPP_DEVEL_TOOLS
#include "gui/cornerdetectortesteditor.h"
#include "gui/cornermatchingtesteditor.h"
#endif

#include "control/datatableactionmanager.h"
#include "control/imagebaseactionmanager.h"
#include "control/camerabaseactionmanager.h"
#include "control/landmarkbaseactionmanager.h"
#include "control/stereorigactionmanager.h"
#include "control/angleconstrainactionmanager.h"
#include "control/distanceconstrainactionmanager.h"
#include "control/cameracalibrationactionmanager.h"
#include "control/localcoordinatesystembaseactionmanager.h"
#include "control/fixedstereosequenceactionmanager.h"
#include "control/trajectoryactionmanager.h"
#include "control/correspondencessetactionsmanager.h"

#include "sparsesolver/modularsbasolver.h"
#include "sparsesolver/sbamodules/trajectorybasesbamodule.h"
#include "sparsesolver/sbamodules/landmarkssbamodule.h"
#include "sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "sparsesolver/sbamodules/localcoordinatesystemsbamodule.h"
#include "sparsesolver/sbamodules/correspondencessetsbamodule.h"

#include <QDebug>

#include <gdal/gdal.h>

#include <glog/logging.h>

namespace py = pybind11;

void runScript(QString const& scriptPath, QStringList argv = {}) {
	py::scoped_interpreter guard{};

	// Evaluate in scope of main module
	py::object scope = py::module::import("__main__").attr("__dict__");
	py::list args;

	for(QString const& arg : argv) {
		args.append(arg.toStdString());
	}

    if (argv.size() > 0) {
        py::module::import("sys").add_object("argv", args);
    }

	py::eval_file(scriptPath.toStdString(), scope);
}

int main(int argc, char *argv[])
{

    //configure libraries
    GDALAllRegister();
    google::InitGoogleLogging(argv[0]);

	StereoVisionApp::StereoVisionApplication a(argc, argv);

	QObject::connect(&a, &StereoVisionApp::StereoVisionApplication::scriptFileExecututionRequested, &runScript);

	//surface setup

	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	format.setStencilBufferSize(8);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setSamples(4);
	QSurfaceFormat::setDefaultFormat(format);

    //app interfaces
    StereoVisionApp::SBASolverModulesInterface* sbasolvermodulesinterface = new StereoVisionApp::SBASolverModulesInterface(&a);
    a.registerAdditionalInterface(StereoVisionApp::SBASolverModulesInterface::AppInterfaceName, sbasolvermodulesinterface);

    StereoVisionApp::TrajectoryBaseSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);
    StereoVisionApp::LandmarksSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);
    StereoVisionApp::ImageAlignementSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);
    StereoVisionApp::LocalCoordinateSystemSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);
    StereoVisionApp::CorrespondencesSetSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();
    StereoVisionApp::ActionManagersLibrary& aml = StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary();

    pF.addType(new StereoVisionApp::DataTableFactory(&a));
    pF.addType(new StereoVisionApp::LandmarkFactory(&a));
    pF.addType(new StereoVisionApp::TrajectoryFactory(&a));
    pF.addType(new StereoVisionApp::ImageFactory(&a));
    pF.addType(new StereoVisionApp::CameraFactory(&a));
    pF.addType(new StereoVisionApp::CorrespondencesSetFactory(&a));
    pF.addType(new StereoVisionApp::StereoRigFactory(&a));
    pF.addType(new StereoVisionApp::LocalCoordinateSystemFactory(&a));
    pF.addType(new StereoVisionApp::AngleConstrainFactory(&a));
    pF.addType(new StereoVisionApp::DistanceConstrainFactory(&a));
    pF.addType(new StereoVisionApp::CameraCalibrationFactory(&a));
    pF.addType(new StereoVisionApp::CameraCalibrationFactory(&a));
    pF.addType(new StereoVisionApp::FixedColorStereoSequenceFactory(&a));
    pF.addType(new StereoVisionApp::FixedStereoPlusColorSequenceFactory(&a));

    aml.registerDatablockActionManager(new StereoVisionApp::DataTableActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::ImageBaseActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::LandmarkBaseActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::TrajectoryActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::CameraBaseActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::StereoRigActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::LocalCoordinateSystemBaseActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::AngleConstrainActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::DistanceConstrainActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::CameraCalibrationActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::FixedColorStereoSequenceActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::FixedStereoPlusColorSequenceActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::CorrespondencesSetActionsManager(&pF));

    a.loadRessources();

	StereoVisionApp::MainWindow* w = a.mainWindow();

	if (w != nullptr) {
        w->installEditor(new StereoVisionApp::DataTableViewEditorFactory(&a));
        w->installEditor(new StereoVisionApp::ImageEditorFactory(&a));
        w->installEditor(new StereoVisionApp::ImageViewerFactory(&a));
		w->installEditor(new StereoVisionApp::ImagePointDetailsEditorFactory(&a));
		w->installEditor(new StereoVisionApp::LandmarkPointDetailsEditorFactory(&a));
		w->installEditor(new StereoVisionApp::LocalCoordinateSystemPointDetailsEditorFactory(&a));
		w->installEditor(new StereoVisionApp::SparseAlignementEditorFactory(&a));
        w->installEditor(new StereoVisionApp::TrajectorySequenceViewEditorFactory(&a));
        w->installEditor(new StereoVisionApp::TrajectoryOptAnalysisEditorFactory(&a));
        w->installEditor(new StereoVisionApp::TrajectoryAlignementAnalysisEditorFactory(&a));
		w->installEditor(new StereoVisionApp::LensEditorFactory(&a));
		w->installEditor(new StereoVisionApp::CameraCalibrationEditorFactory(&a));
		w->installEditor(new StereoVisionApp::CameraCalibrationSparseAlignementEditorFactory(&a));
		w->installEditor(new StereoVisionApp::FixedStereoSequenceEditorFactory(&a));

        #ifdef STEVIAPP_DEVEL_TOOLS
        w->installEditor(new StereoVisionApp::CornerDetectorTestEditorFactory(&a));
        w->installEditor(new StereoVisionApp::CornerMatchingTestEditorFactory(&a));
        #endif
	}

	//create an empty project to start
    a.newEmptyProject();

	return a.exec();
}
