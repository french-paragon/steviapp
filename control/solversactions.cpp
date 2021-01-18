#include "solversactions.h"

#include "datablocks/project.h"
#include "mainwindow.h"

#include "gui/solutioninitconfigdialog.h"
#include "gui/sparsesolverconfigdialog.h"
#include "gui/stepprocessmonitorbox.h"
#include "gui/sparsealignementeditor.h"

#include "sparsesolver/graphsbasolver.h"
#include "sparsesolver/sbagraphreductor.h"
#include "sparsesolver/sbainitializer.h"

#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include <QThread>
#include <QMessageBox>

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

void initSolution(Project* p, MainWindow* w) {

	qint64 initial_frame = -1;

	if (w != nullptr) {

		SolutionInitConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Init solution options"));

		d.setProject(p);

		d.exec();

		if (d.result() == QDialog::Rejected) {
			return;
		}

		initial_frame = d.selectedStartingImage();
	}

	if (!resetSolution(p, w)) {
		return;
	}

	SBAGraphReductor selector(3,2,true,true);
	EightPointsSBAMultiviewInitializer initializer(initial_frame, true, false);

	SBAGraphReductor::elementsSet selection = selector(p, false);

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		QMessageBox::warning(w, "Initialization impossible", "No point nor image selected");
		return;
	}

	auto initial_setup = initializer.computeInitialSolution(p, selection.pts, selection.imgs);

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

void solveSparse(Project* p, MainWindow *w, int pnStep) {

	bool computeUncertainty = true;
	bool useSparseOptimizer = true;
	int nSteps = pnStep;

	if (w != nullptr) {

		SparseSolverConfigDialog d(w);
		d.setModal(true);
		d.setWindowTitle(QObject::tr("Sparse optimizer options"));

		d.exec();

		if (!d.shouldRun()) {
			return;
		}

		computeUncertainty = d.computeUncertainty();
		useSparseOptimizer = d.useSparseOptimizer();
		nSteps = d.numberOfSteps();
	}

	if (nSteps <= 0) {
		return;
	}

	GraphSBASolver* solver = new GraphSBASolver(p, computeUncertainty, useSparseOptimizer);

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
