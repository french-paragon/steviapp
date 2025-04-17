#include "cameracalibrationactions.h"

#include "mainwindow.h"

#include "datablocks/project.h"
#include "datablocks/cameracalibration.h"

#include "gui/cameracalibrationeditor.h"
#include "gui/cameracalibrationsparsealignementeditor.h"
#include "gui/cameracalibrationoptionsdialog.h"
#include "gui/stepprocessmonitorbox.h"

//#include "sparsesolver/graphcameracalibrator.h"

#include <QThread>
#include <QDebug>

namespace StereoVisionApp {


void runCameraCalibration(Project* project, qint64 calibration) {

    /*if (project == nullptr) {
		return;
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	CameraCalibration* camcalib = project->getDataBlock<CameraCalibration>(calibration);

	if (camcalib == nullptr) {
		return;
	}

	if (camcalib->nSelectedCameras() <= 0) {
		camcalib->configureSelectedImages();
	}

	int nIter = 10;

	if (mw != nullptr) {

		CameraCalibrationOptionsDialog ccod(mw);
		ccod.setModal(true);
		ccod.setCheckboardCornerSize(camcalib->getGridSize());
		ccod.setNIterations(nIter);

		int code = ccod.exec();

		if (code != QDialog::Accepted) {
			return;
		}

		camcalib->setGridSize(ccod.checkboardCornerSize());
		nIter = ccod.nIterations();

	}


	GraphCameraCalibrator* solver = new GraphCameraCalibrator(camcalib, true);

	solver->setOptimizationSteps(nIter);

	QThread* t = new QThread();

	solver->moveToThread(t);
	QObject::connect(solver, &QObject::destroyed, t, &QThread::quit);
	QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

	if (mw != nullptr) {

		QObject::connect(solver, &SteppedProcess::finished, mw, [mw, camcalib] () {
			Editor* camCalibEditor = mw->openEditor(CameraCalibrationSparseAlignementEditor::staticMetaObject.className());

			if (camCalibEditor != nullptr) {
				CameraCalibrationSparseAlignementEditor* ccE = qobject_cast<CameraCalibrationSparseAlignementEditor*>(camCalibEditor);

				if (ccE != nullptr) {
					ccE->setCalibration(camcalib);
				}
			}

		}, Qt::QueuedConnection);

		StepProcessMonitorBox* box = new StepProcessMonitorBox(mw);
		box->setWindowFlag(Qt::Dialog);
		box->setWindowModality(Qt::WindowModal);
		box->setWindowTitle(QObject::tr("Camera optimization"));

		box->setProcess(solver);

		QObject::connect(box, &QObject::destroyed, solver, &QObject::deleteLater);

		box->show();

	} else {

		solver->deleteWhenDone(true);
	}

	t->start();
	solver->run();
    */

}


} // namespace StereoVisionApp
