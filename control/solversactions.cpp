#include "solversactions.h"

#include "datablocks/project.h"
#include "mainwindow.h"

#include "gui/sparsesolverconfigdialog.h"
#include "gui/stepprocessmonitorbox.h"
#include "gui/sparsealignementeditor.h"

#include "sparsesolver/graphsbasolver.h"
#include "sparsesolver/sbagraphreductor.h"
#include "sparsesolver/sbainitializer.h"

#include "datablocks/landmark.h"
#include "datablocks/image.h"

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

	if (!resetSolution(p, w)) {
		return;
	}

	SBAGraphReductor selector(3,2,true,true);
	EightPointsSBAMultiviewInitializer initializer;

	SBAGraphReductor::elementsSet selection = selector(p);

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		QMessageBox::warning(w, "Initialization impossible", "No point nor image selected");
		return;
	}

	auto initial_setup = initializer.computeInitialSolution(p, selection.pts, selection.imgs);

	for (auto & map_id : initial_setup.points) {

		qint64 const& id = map_id.first;

		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm != nullptr) {

			floatParameter x = lm->optimizedX();
			x.setIsSet(initial_setup.points[id].x());
			lm->setOptimisedX(x);

			floatParameter y = lm->optimizedY();
			y.setIsSet(initial_setup.points[id].y());
			lm->setOptimisedY(y);

			floatParameter z = lm->optimizedZ();
			z.setIsSet(initial_setup.points[id].z());
			lm->setOptimisedZ(z);
		}

	}

	for (auto & map_id : initial_setup.cams) {

		qint64 const& id = map_id.first;

		Image* im = qobject_cast<Image*>(p->getById(id));

		if (im != nullptr) {
			floatParameter x = im->optXCoord();
			x.setIsSet(initial_setup.cams[id].t.x());
			im->setOptXCoord(x);

			floatParameter y = im->optYCoord();
			y.setIsSet(initial_setup.cams[id].t.y());
			im->setOptYCoord(y);

			floatParameter z = im->optZCoord();
			z.setIsSet(initial_setup.cams[id].t.z());
			im->setOptZCoord(z);

			Eigen::Vector3f r = initial_setup.cams[id].R.eulerAngles(0,1,2);

			x = im->optXRot();
			x.setIsSet(r.x()/M_PI*180.);
			im->setOptXRot(x);

			y = im->optYRot();
			y.setIsSet(r.y()/M_PI*180.);
			im->setOptYRot(y);

			z = im->optZRot();
			z.setIsSet(r.z()/M_PI*180.);
			im->setOptZRot(z);
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
