#include "solversactions.h"

#include "datablocks/project.h"
#include "mainwindow.h"
#include "application.h"

#include "gui/solutioninitconfigdialog.h"
#include "gui/sparsesolverconfigdialog.h"
#include "gui/sparsestereosolverconfigdialog.h"
#include "gui/stepprocessmonitorbox.h"
#include "gui/sparsealignementeditor.h"

#include "sparsesolver/graphroughbundleadjustementsolver.h"
#include "sparsesolver/graphsbasolver.h"
#include "sparsesolver/ceressbasolver.h"
#include "sparsesolver/graphstereorigsolver.h"
#include "sparsesolver/sbagraphreductor.h"
#include "sparsesolver/sbainitializer.h"
#include "sparsesolver/modularsbasolver.h"
#include "sparsesolver/sbamodules/trajectorybasesbamodule.h"
#include "sparsesolver/sbamodules/localcoordinatesystemsbamodule.h"
#include "sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "sparsesolver/sbamodules/landmarkssbamodule.h"
#include "sparsesolver/sbamodules/correspondencessetsbamodule.h"

#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/stereorig.h"

#include <QThread>
#include <QMessageBox>
#include <QTextStream>

#include <eigen3/Eigen/Geometry>

namespace StereoVisionApp {

bool resetSolution(Project* p, MainWindow* w) {

	if (p == nullptr) {
		return false;
	}

	if (!p->hasSolution()) {
		return true;
	}

	if (w != nullptr) {
		QMessageBox::StandardButton b = QMessageBox::question(w,
															  QObject::tr("Clear solution"),
															  QObject::tr("The previous solution will be lost, continue ?"),
															  QMessageBox::Yes|QMessageBox::No,
															  QMessageBox::No);

		if (b != QMessageBox::Yes) {
			return false;
		}
	}

	p->clearOptimized();

	return true;
}

void solveCoarse(Project* p, MainWindow* w, int pnStep) {

	bool useSparseOptimizer = true;
	bool initWithCurrentSolution = false;
	int nSteps = pnStep;

	FixedParameters fixedParameters = FixedParameter::CameraInternal|FixedParameter::StereoRigs;

	if (w != nullptr) {

		SparseSolverConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Sparse optimizer options"));
		d.enableComputeUncertaintyOption(false);

		d.setFixedParameters(fixedParameters);

		d.exec();

		if (!d.shouldRun()) {
			return;
		}

		useSparseOptimizer = d.useSparseOptimizer();
		nSteps = d.numberOfSteps();
		initWithCurrentSolution = d.initWithCurrentSol();

		fixedParameters = d.getFixedParameters();
	}

	if (nSteps <= 0) {
		return;
	}

	GraphRoughBundleAdjustementSolver* solver = new GraphRoughBundleAdjustementSolver(p, useSparseOptimizer, initWithCurrentSolution);

	solver->setOptimizationSteps(nSteps);
	solver->setFixedParametersFlag(fixedParameters);

	QThread* t = new QThread();

	solver->moveToThread(t);
	QObject::connect(solver, &QObject::destroyed, t, &QThread::quit);
	QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

	if (w != nullptr) {

		QObject::connect(solver, &SteppedProcess::finished, w, &MainWindow::openSparseViewer, Qt::QueuedConnection);

		StepProcessMonitorBox* box = new StepProcessMonitorBox(w);
		box->setWindowFlag(Qt::Dialog);
		box->setWindowModality(Qt::WindowModal);
		box->setWindowTitle(QObject::tr("Sparse optimization"));

		box->setProcess(solver);

		QObject::connect(box, &QObject::destroyed, solver, &QObject::deleteLater);

		box->show();

	} else {

		solver->deleteWhenDone(true);
	}

	t->start();
	solver->run();
}

void initSolution(Project* p, MainWindow* w) {

	qint64 initial_frame = -1;

	FixedParameters fixedParameters = FixedParameter::CameraInternal|FixedParameter::StereoRigs;

	if (w != nullptr) {

		SolutionInitConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Init solution options"));

		d.setProject(p);

		d.setFixedParameters(fixedParameters);

		d.exec();

		if (d.result() == QDialog::Rejected) {
			return;
		}

		initial_frame = d.selectedStartingImage();

		fixedParameters = d.getFixedParameters();
	}

	if (!resetSolution(p, w)) {
		return;
	}

	SBAGraphReductor selector(3,2,true,true);
	SBAInitializer* initializer;

	bool useStereoRigInitializer = false;

	QVector<qint64> rigIds = p->getIdsByClass(StereoRig::staticMetaObject.className());

	for(qint64 id : rigIds) {
		StereoRig* rig = qobject_cast<StereoRig*>(p->getById(id));

		if (rig != nullptr) {
			QVector<qint64> subitems = rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

			bool found = false;

			for (qint64 id : subitems) {
				ImagePair* imp = qobject_cast<ImagePair*>(rig->getById(id));

				Image* img1 = qobject_cast<Image*>(p->getById(imp->idImgCam1()));
				Image* img2 = qobject_cast<Image*>(p->getById(imp->idImgCam2()));

				if (img1 != nullptr and img2 != nullptr) {
					auto lms1 = img1->getAttachedLandmarksIds();
					auto lms2 = img2->getAttachedLandmarksIds();

					QSet<qint64> s1(lms1.begin(), lms1.end());
					QSet<qint64> s2(lms2.begin(), lms2.end());

					s1.intersect(s2);

					if (s1.size() > 4) {
						found = true;
						break;
					}
				}
			}

			if (found) {
				useStereoRigInitializer = true;
				break;
			}
		}
	}

	if (useStereoRigInitializer) {
		initializer = new StereoRigInitializer(initial_frame, false, true, true);
	} else {
		initializer = new EightPointsSBAMultiviewInitializer(initial_frame, true, true);
	}

	initializer->setPreOptimizedFixedParameters(fixedParameters);

	SBAGraphReductor::elementsSet selection = selector(p, false);

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		QMessageBox::warning(w, "Initialization impossible", "No point nor image selected");
		return;
	}

	auto initial_setup = initializer->computeInitialSolution(p, selection.pts, selection.imgs);

	delete initializer;

	for (auto & map_id : initial_setup.points) {

		qint64 const& id = map_id.first;

		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm != nullptr) {

			floatParameterGroup<3> pos = lm->optPos();
			pos.value(0) = initial_setup.points[id].x();
			pos.value(1) = initial_setup.points[id].y();
			pos.value(2) = initial_setup.points[id].z();
			pos.setIsSet();
			lm->setOptPos(pos);

			lm->setOptimizationStep(DataBlock::Initialized);
		}

	}

	for (auto & map_id : initial_setup.cams) {

		qint64 const& id = map_id.first;

		Image* im = qobject_cast<Image*>(p->getById(id));

		if (im != nullptr) {
			floatParameterGroup<3> pos = im->optPos();
			pos.value(0) = initial_setup.cams[id].t.x();
			pos.value(1) = initial_setup.cams[id].t.y();
			pos.value(2) = initial_setup.cams[id].t.z();
			pos.setIsSet();
			im->setOptPos(pos);

			Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(initial_setup.cams[id].R);

			floatParameterGroup<3> rot = im->optRot();
			rot.value(0) = r.x();
			rot.value(1) = r.y();
			rot.value(2) = r.z();
			rot.setIsSet();
			im->setOptRot(rot);

			im->setOptimizationStep(DataBlock::Initialized);

		}

	}

	if (w != nullptr) {
		w->openSparseViewer();
	}
}

void initMonoStereoRigSolution(Project* p, MainWindow* w) {

	qint64 initial_frame = -1;
	FixedParameters fixed_parameters = NoFixedParameters;

	if (w != nullptr) {

		SolutionInitConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Init solution options"));

		d.setFixedParameters(fixed_parameters);

		d.setProject(p);

		d.exec();

		if (d.result() == QDialog::Rejected) {
			return;
		}

		initial_frame = d.selectedStartingImage();
		fixed_parameters = d.getFixedParameters();
	}

	if (!resetSolution(p, w)) {
		return;
	}

	if (initial_frame < 0) {
		return;
	}

	SBAGraphReductor selector(3,2,true,true);
	SBAInitializer* initializer = new StereoRigInitializer(initial_frame, true, true, false);

	SBAGraphReductor::elementsSet selection = selector(p, false);

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		QMessageBox::warning(w, "Initialization impossible", "No point nor image selected");
		return;
	}

	auto initial_setup = initializer->computeInitialSolution(p, selection.pts, selection.imgs);

	delete initializer;

	for (auto & map_id : initial_setup.points) {

		qint64 const& id = map_id.first;

		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm != nullptr) {

			floatParameterGroup<3> pos = lm->optPos();
			pos.value(0) = initial_setup.points[id].x();
			pos.value(1) = initial_setup.points[id].y();
			pos.value(2) = initial_setup.points[id].z();
			pos.setIsSet();
			lm->setOptPos(pos);

			lm->setOptimizationStep(DataBlock::Initialized);
		}

	}

	for (auto & map_id : initial_setup.cams) {

		qint64 const& id = map_id.first;

		Image* im = qobject_cast<Image*>(p->getById(id));

		if (im != nullptr) {
			floatParameterGroup<3> pos = im->optPos();
			pos.value(0) = initial_setup.cams[id].t.x();
			pos.value(1) = initial_setup.cams[id].t.y();
			pos.value(2) = initial_setup.cams[id].t.z();
			pos.setIsSet();
			im->setOptPos(pos);

			Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(initial_setup.cams[id].R);

			floatParameterGroup<3> rot = im->optRot();
			rot.value(0) = r.x();
			rot.value(1) = r.y();
			rot.value(2) = r.z();
			rot.setIsSet();
			im->setOptRot(rot);

			im->setOptimizationStep(DataBlock::Initialized);

		}

	}

	if (w != nullptr) {
		w->openSparseViewer();
	}
}

void checkBasicSBAModulesRegistration(SBASolverModulesInterface* interface) {

    if (!interface->hasSBAModule(TrajectoryBaseSBAModule::ModuleName)) {
        TrajectoryBaseSBAModule::registerDefaultModuleFactory(interface);
    }

    if (!interface->hasSBAModule(LandmarksSBAModule::ModuleName)) {
        LandmarksSBAModule::registerDefaultModuleFactory(interface);
    }

    if (!interface->hasSBAModule(ImageAlignementSBAModule::ModuleName)) {
        ImageAlignementSBAModule::registerDefaultModuleFactory(interface);
    }

    if (!interface->hasSBAModule(LocalCoordinateSystemSBAModule::ModuleName)) {
        LocalCoordinateSystemSBAModule::registerDefaultModuleFactory(interface);
    }

    if (!interface->hasSBAModule(CorrespondencesSetSBAModule::ModuleName)) {
        CorrespondencesSetSBAModule::registerDefaultModuleFactory(interface);
    }
}

void configureModularSBASolverInterfaces(SBASolverModulesInterface* interface, ModularSBASolver* solver) {


    ModularSBASolver::SBAModule* traj_module = interface->buildSBAModule(TrajectoryBaseSBAModule::ModuleName, solver);

    if (traj_module != nullptr) {
        solver->addModule(traj_module);
    }

    for (QString & moduleName : interface->installedSBAModules()) {

        if (moduleName == TrajectoryBaseSBAModule::ModuleName) {
            continue; //trajectory has already been registered (it is expected to be registered first).
        }

        ModularSBASolver::SBAModule* module = interface->buildSBAModule(moduleName, solver);

        if (module != nullptr) {
            solver->addModule(module);
        }
    }
}

void solveSparse(Project* p, MainWindow *w, int pnStep) {

    //initial checks

    StereoVisionApplication* application = StereoVisionApplication::GetAppInstance();

    if (application == nullptr) {
        return;
    }

    StereoVisionApp::Project* project = application->getCurrentProject();

    if (project == nullptr) {
        return;
    }

    QObject* interface = application->getAdditionalInterface(SBASolverModulesInterface::AppInterfaceName);

    if (interface == nullptr) { // SBASolverModulesInterface not installed yet

        interface = new SBASolverModulesInterface(application);

        application->registerAdditionalInterface(SBASolverModulesInterface::AppInterfaceName, interface);

    }

    SBASolverModulesInterface* sbamodulesinterface = qobject_cast<SBASolverModulesInterface*>(interface);

    if (sbamodulesinterface == nullptr) {
        return;
    }

    checkBasicSBAModulesRegistration(sbamodulesinterface);

    //gui input

	bool computeUncertainty = true;
	bool useSparseOptimizer = true;
    bool verbose = true;
	int nSteps = pnStep;

	FixedParameters fixedParameters = FixedParameter::CameraInternal|FixedParameter::StereoRigs;

	if (w != nullptr) {

		SparseSolverConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Sparse optimizer options"));
		d.enableUseCurrentSolutionOption(false);

		d.setFixedParameters(fixedParameters);

		d.exec();

		if (!d.shouldRun()) {
			return;
		}

		computeUncertainty = d.computeUncertainty();
		useSparseOptimizer = d.useSparseOptimizer();
		nSteps = d.numberOfSteps();

		fixedParameters = d.getFixedParameters();
	}

	if (nSteps <= 0) {
		return;
	}

    ModularSBASolver* solver = new ModularSBASolver(p, computeUncertainty, useSparseOptimizer, verbose);

	solver->setOptimizationSteps(nSteps);
	solver->setFixedParametersFlag(fixedParameters);

    QFileInfo projSourceInfos(project->source());

    QDir projDir = projSourceInfos.dir();
    solver->enableLogging(projDir.filePath("logging")); //TODO: add option in the dialog to select logs locations

    configureModularSBASolverInterfaces(sbamodulesinterface, solver);

	QThread* t = new QThread();

	solver->moveToThread(t);
	QObject::connect(solver, &QObject::destroyed, t, &QThread::quit);
	QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

	if (w != nullptr) {

		QObject::connect(solver, &SteppedProcess::finished, w, &MainWindow::openSparseViewer, Qt::QueuedConnection);

		StepProcessMonitorBox* box = new StepProcessMonitorBox(w);
		box->setWindowFlag(Qt::Dialog);
		box->setWindowModality(Qt::WindowModal);
		box->setWindowTitle(QObject::tr("Sparse optimization"));

		box->setProcess(solver);

		QObject::connect(box, &QObject::destroyed, solver, &QObject::deleteLater);

		box->show();

	} else {

		solver->deleteWhenDone(true);
	}

	t->start();
	solver->run();

}


void solveSparseHeadless(Project* p,
				 bool useCurrentSolutionAtStart,
				 bool useSparseOptimizer,
				 bool computeUncertainty,
				 int nSteps,
				 FixedParameters fixed) {

    StereoVisionApplication* application = StereoVisionApplication::GetAppInstance();

    if (application == nullptr) {
        return;
    }

    StereoVisionApp::Project* project = application->getCurrentProject();

    if (project == nullptr) {
        return;
    }

    QObject* interface = application->getAdditionalInterface(SBASolverModulesInterface::AppInterfaceName);

    if (interface == nullptr) { // SBASolverModulesInterface not installed yet

        interface = new SBASolverModulesInterface(application);

        application->registerAdditionalInterface(SBASolverModulesInterface::AppInterfaceName, interface);

    }

    SBASolverModulesInterface* sbamodulesinterface = qobject_cast<SBASolverModulesInterface*>(interface);

    if (sbamodulesinterface == nullptr) {
        return;
    }

    checkBasicSBAModulesRegistration(sbamodulesinterface);

	if (nSteps <= 0) {
		return;
	}

    ModularSBASolver* solver = new ModularSBASolver(p, computeUncertainty, useSparseOptimizer);

	solver->setOptimizationSteps(nSteps);
	solver->setFixedParametersFlag(fixed);

    QFileInfo projSourceInfos(project->source());

    QDir projDir = projSourceInfos.dir();
    solver->enableLogging(projDir.filePath("logging"));

    configureModularSBASolverInterfaces(sbamodulesinterface, solver);

	solver->run();

}

void solveSparseStereoRig(Project* p, MainWindow* w, int pnStep) {

	bool computeUncertainty = true;
	bool useSparseOptimizer = true;
	int nSteps = pnStep;

	std::optional<qint64> im1id = std::nullopt;
	std::optional<qint64> im2id = std::nullopt;

	if (w != nullptr) {

		SparseStereoSolverConfigDialog d(w);
		d.setProject(p);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Sparse stereo rig optimizer options"));

		d.exec();

		if (!d.shouldRun()) {
			return;
		}

		computeUncertainty = d.computeUncertainty();
		useSparseOptimizer = d.useSparseOptimizer();
		nSteps = d.numberOfSteps();

		im1id = d.selectedStartingImage1();
		im2id = d.selectedStartingImage2();
	}

	if (nSteps <= 0) {
		return;
	}

	if (im1id == im2id or !im1id.has_value() or !im2id.has_value()) {
		return;
	}

	GraphStereoRigSolver* solver = new GraphStereoRigSolver(p,
															im1id.value(),
															im2id.value(),
															computeUncertainty,
															useSparseOptimizer);

	solver->setOptimizationSteps(nSteps);

	QThread* t = new QThread();

	solver->moveToThread(t);
	QObject::connect(solver, &QObject::destroyed, t, &QThread::quit);
	QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

	if (w != nullptr) {

		QObject::connect(solver, &SteppedProcess::finished, w, &MainWindow::openSparseViewer, Qt::QueuedConnection);

		StepProcessMonitorBox* box = new StepProcessMonitorBox(w);
		box->setWindowFlag(Qt::Dialog);
		box->setWindowModality(Qt::WindowModal);
		box->setWindowTitle(QObject::tr("Sparse optimization"));

		box->setProcess(solver);

		QObject::connect(box, &QObject::destroyed, solver, &QObject::deleteLater);

		box->show();

	} else {

		solver->deleteWhenDone(true);
	}

	t->start();
	solver->run();
}

} //namespace StereoVisionApp
