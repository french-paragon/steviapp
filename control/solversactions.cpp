#include "solversactions.h"

#include "datablocks/project.h"
#include "mainwindow.h"

#include "gui/sparsesolverconfigdialog.h"
#include "gui/stepprocessmonitorbox.h"

#include "sparsesolver/graphsbasolver.h"

#include <QThread>

namespace StereoVisionApp {

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
	QObject::connect(solver, &QObject::destroyed, t, &QObject::deleteLater);

	if (w != nullptr) {

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
