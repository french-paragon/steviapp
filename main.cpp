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
#include "datablocks/cameras/pushbroompinholecamera.h"
#include "datablocks/image.h"
#include "datablocks/correspondencesset.h"
#include "datablocks/stereorig.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"
#include "datablocks/cameracalibration.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/mounting.h"
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
#include "gui/trajectorycomparisoneditor.h"
#include "gui/trajectoryalignementanalysiseditor.h"
#include "gui/cornermatchingeditor.h"

#ifdef STEVIAPP_DEVEL_TOOLS
#include "gui/cornerdetectortesteditor.h"
#endif

#include "control/datatableactionmanager.h"
#include "control/imagebaseactionmanager.h"
#include "control/camerabaseactionmanager.h"
#include "control/pushbroomcameraactionmanager.h"
#include "control/landmarkbaseactionmanager.h"
#include "control/stereorigactionmanager.h"
#include "control/angleconstrainactionmanager.h"
#include "control/distanceconstrainactionmanager.h"
#include "control/cameracalibrationactionmanager.h"
#include "control/localcoordinatesystembaseactionmanager.h"
#include "control/mountingactionsmanager.h"
#include "control/fixedstereosequenceactionmanager.h"
#include "control/trajectoryactionmanager.h"
#include "control/correspondencessetactionsmanager.h"

#include "sparsesolver/modularsbasolver.h"
#include "sparsesolver/sbamodules/trajectorybasesbamodule.h"
#include "sparsesolver/sbamodules/landmarkssbamodule.h"
#include "sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "sparsesolver/sbamodules/localcoordinatesystemsbamodule.h"
#include "sparsesolver/sbamodules/mountingssbamodule.h"
#include "sparsesolver/sbamodules/correspondencessetsbamodule.h"

#include <QDebug>
#include <QMessageBox>

#include <glog/logging.h>

#ifdef ADVANCED_LOGGING

#include <QDir>
#include <QStandardPaths>

#include <cpptrace/cpptrace.hpp>
#include <cpptrace/formatting.hpp>

#include <fstream>

#include <signal.h>

void segfaultHandler(int sig) {

     StereoVisionApp::StereoVisionApplication* app =  StereoVisionApp::StereoVisionApplication::GetAppInstance();

    QStringList paths = QStandardPaths::standardLocations(QStandardPaths::ConfigLocation);
    QString path = paths.first();
    QDir dir(path);

    dir.mkpath(StereoVisionApp::StereoVisionApplication::appName());
    dir.cd(StereoVisionApp::StereoVisionApplication::appName());

    const char* logDirName = "logs";

    dir.mkpath(logDirName);
    dir.cd(logDirName);

    QString stacktrace_log_file_name = dir.absoluteFilePath(QString("segfault_trace_%1.log")
                                                                .arg(QDateTime::currentDateTime().toString("yyyy_MM_dd_hh_mm_ss_zzz")));

    cpptrace::stacktrace trace = cpptrace::generate_trace();

    std::cerr << "Application " << StereoVisionApp::StereoVisionApplication::appName() << " segfaulted with error code " << sig << std::endl;

    std::fstream logFile;
    logFile.open(stacktrace_log_file_name.toStdString(), std::ios::out);
    if (logFile.is_open()) {

        if (app != nullptr) {

            logFile << "App: " <<  StereoVisionApp::StereoVisionApplication::appName() << "\n";

            StereoVisionApp::Project* currentProject = app->getCurrentProject();
            QString currentProjectPath = "";
            if (currentProject != nullptr) {
                currentProjectPath = currentProject->source();
            }

            if (currentProjectPath.isEmpty()) {
                logFile << "No project open" << "\n";
            } else {
                logFile << "With opened project: " << qPrintable(currentProjectPath) << "\n";
            }
        }

        trace.print(logFile);
        logFile.flush();
        logFile.close();
        std::cerr << "stack saved to: " << stacktrace_log_file_name.toStdString() << std::endl;
    } else {
        trace.print(std::cerr);
    }

    exit(1);
}
#endif

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

QtMessageHandler originalMessageHandler = nullptr;

void messagesHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QString message = qFormatLogMessage(type, context, msg);

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw != nullptr) { //we have a gui
        //invoke the method in the main windows' thread.
        QMetaObject::invokeMethod(mw, [mw, type, msg] () {
            if (type == QtInfoMsg) {
                QMessageBox::information(mw, StereoVisionApp::MainWindow::tr("Information"), msg);
            } else if (type == QtWarningMsg) {
                return; //skip warning messages for now, they clutter the interface.
                //QMessageBox::warning(mw, StereoVisionApp::MainWindow::tr("Warning!"), msg);
            } else if (type == QtCriticalMsg) {
                QMessageBox::critical(mw, StereoVisionApp::MainWindow::tr("Critical"), msg);
            } else if (type == QtFatalMsg) {
                QMessageBox::critical(mw, StereoVisionApp::MainWindow::tr("Fatal"), msg);
            }
        });
    }

    if (originalMessageHandler) {
        (*originalMessageHandler)(type, context, msg);
    }
}

int main(int argc, char *argv[])
{

#ifndef NDEBUG //debug mode enabled
    std::cout << "Steviapp - debug mode" << std::endl;
#endif

    #ifdef ADVANCED_LOGGING
    signal(SIGSEGV, segfaultHandler);   // install our handler
    #endif

    //configure libraries
    google::InitGoogleLogging(argv[0]);

    //logging and message handling
    originalMessageHandler = qInstallMessageHandler(messagesHandler);

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
    StereoVisionApp::MountingsSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);
    StereoVisionApp::CorrespondencesSetSBAModule::registerDefaultModuleFactory(sbasolvermodulesinterface);

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();
    StereoVisionApp::ActionManagersLibrary& aml = StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary();

    pF.addType(new StereoVisionApp::DataTableFactory(&a));
    pF.addType(new StereoVisionApp::LandmarkFactory(&a));
    pF.addType(new StereoVisionApp::TrajectoryFactory(&a));
    pF.addType(new StereoVisionApp::ImageFactory(&a));
    pF.addType(new StereoVisionApp::CameraFactory(&a));
    pF.addType(new StereoVisionApp::PushBroomPinholeCameraFactory(&a));
    pF.addType(new StereoVisionApp::CorrespondencesSetFactory(&a));
    pF.addType(new StereoVisionApp::StereoRigFactory(&a));
    pF.addType(new StereoVisionApp::LocalCoordinateSystemFactory(&a));
    pF.addType(new StereoVisionApp::MountingFactory(&a));
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
    aml.registerDatablockActionManager(new StereoVisionApp::PushBroomCameraActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::StereoRigActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::LocalCoordinateSystemBaseActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::MountingActionsManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::AngleConstrainActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::DistanceConstrainActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::CameraCalibrationActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::FixedColorStereoSequenceActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::FixedStereoPlusColorSequenceActionManager(&pF));
    aml.registerDatablockActionManager(new StereoVisionApp::CorrespondencesSetActionsManager(&pF));

    Q_INIT_RESOURCE(gl_shaders);
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
        w->installEditor(new StereoVisionApp::TrajectoryComparisonEditorFactory(&a));
        w->installEditor(new StereoVisionApp::TrajectoryAlignementAnalysisEditorFactory(&a));
        w->installEditor(new StereoVisionApp::LensEditorFactory(&a));
        w->installEditor(new StereoVisionApp::PushBroomLenseEditorFactory(&a));
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
