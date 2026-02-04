#include "imagebaseactionmanager.h"

#include "control/application_config.h"

#include "mainwindow.h"
#include "gui/imageeditor.h"
#include "gui/imagepointdetailseditor.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/stereorig.h"
#include "datablocks/cameracalibration.h"

#include <QAction>
#include <QStandardPaths>
#include <QFileDialog>
#include <QMenu>
#include <QMessageBox>

#include "imagebaseactions.h"

#include "cornerdetectionactions.h"

namespace StereoVisionApp {

ImageBaseActionManager::ImageBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString ImageBaseActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::ImageBaseActionManager";
}
QString ImageBaseActionManager::itemClassName() const {
	return ImageFactory::imageClassName();
}

QList<QAction*> ImageBaseActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QString cn = itemClassName();

	QAction* add = new QAction(tr("Add Images"), parent);
	connect(add, &QAction::triggered, [w, p] () {
		QString filter = "Images(*.jpg *.jpeg *.png *.bmp *.tiff *.tif)";
		QString folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
		QStringList images = QFileDialog::getOpenFileNames(w, tr("Open Images"), folder, filter);

		QStringList failedImgs = addImages(images, p);

		if (!failedImgs.isEmpty()) {
			QString errorList = failedImgs.join("\n");
			QMessageBox::warning(w, tr("Failed to load some images"), errorList);
		}
	});

	QAction* exportRectified = new QAction(tr("Export rectified images"), parent);
	connect(exportRectified, &QAction::triggered, [w, p, cn] () {
		QList<qint64> imageIds = p->getIdsByClass(cn).toList();
		exportRectifiedImages(imageIds, p, w);
	});

	if (p->getIdsByClass(cn).size() == 0) {
		exportRectified->setEnabled(false);
	}

    #ifdef STEVIAPP_DEVEL_TOOLS

    QAction* testDetectCorners = new QAction(tr("Test detect corners"), parent);
    connect(testDetectCorners, &QAction::triggered, [] () {
        detectCornerInTestImage();
    });

    QAction* testMatchCorners = new QAction(tr("Test match corners"), parent);
    connect(testMatchCorners, &QAction::triggered, [] () {
        matchCornersInTestImagePair();
    });
    return {add, exportRectified, testDetectCorners, testMatchCorners};

    #endif

	return {add, exportRectified};

}
QList<QAction*> ImageBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

	Image* im = qobject_cast<Image*>(item);

	if (im == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QString cn = itemClassName();

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, im] () {

			Editor* e = mw->openEditor(ImageEditor::ImageEditorClassName);
			ImageEditor* ie = qobject_cast<ImageEditor*>(e);

			ie->setImage(im);

		});
		lst << edit;
	}

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Point details"), parent);
		connect(edit, &QAction::triggered, [mw, im] () {

			Editor* e = mw->openEditor(ImagePointDetailsEditor::staticMetaObject.className());
			ImagePointDetailsEditor* ie = qobject_cast<ImagePointDetailsEditor*>(e);

			ie->setImage(im);

		});
		lst << edit;
	}

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [im] () {
		Project* p = im->getProject();

		if (p != nullptr) {
			p->clearById(im->internalId());
		}
	});
	lst.append(remove);

	QAction* assignToCamera = createAssignToCameraAction(parent, im->getProject(), {im});

    lst.append(assignToCamera);

    QAction* exportRectified = new QAction(tr("Export rectified image"), parent);
    connect(exportRectified, &QAction::triggered, [mw, im] () {
        exportRectifiedImages({im->internalId()}, im->getProject(), mw);
    });
    lst.append(exportRectified);

    QAction* exportLandmarks = new QAction(tr("Export landmarks positions"), parent);
    connect(exportLandmarks, &QAction::triggered, [mw, im] () {
        exportImageLandmarksPositionsToCSV(im->internalId(), im->getProject(), mw);
    });
    lst.append(exportLandmarks);

	QAction* detectHexaTarget = new QAction(tr("Detect hexagonal targets"), parent);
	connect(detectHexaTarget, &QAction::triggered, [im] () {
		detectHexagonalTargets({im->internalId()}, im->getProject());
	});
	lst.append(detectHexaTarget);

	QAction* alignHexaTarget = new QAction(tr("Align hexagonal targets"), parent);
	connect(alignHexaTarget, &QAction::triggered, [im] () {
		orientHexagonalTargetsRelativeToCamera(im->internalId(), im->getProject());
	});
	lst.append(alignHexaTarget);

	QAction* alignImageOnLandmarks = new QAction(tr("Align image"), parent);
	connect(alignImageOnLandmarks, &QAction::triggered, [im] () {
		orientCamerasRelativeToObservedLandmarks(im->internalId(), im->getProject());
	});
	lst.append(alignImageOnLandmarks);

	QAction* addToCalibration = createAddToCalibrationAction(parent, im->getProject(), {im});

	lst.append(addToCalibration);


    #ifdef STEVIAPP_DEVEL_TOOLS

    QAction* testDetectCorners = new QAction(tr("Test detect corners"), parent);
    connect(testDetectCorners, &QAction::triggered, [im] () {
        detectCornerInImage(im->getProject(), im->internalId());
    });
    lst.append(testDetectCorners);

    #endif

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [im] () {
		im->clearOptimized();
	});
	lst.append(clearOptimized);

	return lst;

}
QList<QAction*> ImageBaseActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QVector<Image*> ims;
	QVector<qint64> imIds;
	ims.reserve(projectIndex.count());
	imIds.reserve(projectIndex.count());

	for (QModelIndex const& id : projectIndex) {
		qint64 imid = p->data(id, Project::IdRole).toInt();
		Image* im = qobject_cast<Image*>(p->getById(imid));

		if (im != nullptr) {
			ims.push_back(im);
			imIds.push_back(imid);
		}
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QString cn = itemClassName();

	QList<QAction*> lst;

	QAction* assignToCamera = createAssignToCameraAction(parent, p, ims);

	lst.append(assignToCamera);

	QAction* assignToStereoRig = createAssignToStereoRigAction(parent, p, ims);

	if (assignToStereoRig != nullptr) {
		lst.append(assignToStereoRig);
	}

    QAction* openImagesInCornerMatchingEditor = createOpenInCornerMatchingEditor(parent, p, imIds);

    if (openImagesInCornerMatchingEditor != nullptr) {
        lst.append(openImagesInCornerMatchingEditor);
    }


	QAction* exportRectified = new QAction(tr("Export rectified images"), parent);
	connect(exportRectified, &QAction::triggered, [w, p, imIds] () {
		exportRectifiedImages(imIds.toList(), p, w);
	});

	lst.append(exportRectified);


	QAction* exportForStereoRig = createExportRectifiedForRigAction(parent, p, imIds, w);

	if (exportForStereoRig != nullptr) {
		lst.append(exportForStereoRig);
	}

	QAction* detectHexaTarget = new QAction(tr("Detect hexagonal targets"), parent);
	connect(detectHexaTarget, &QAction::triggered, [p, imIds] () {
		detectHexagonalTargets(imIds.toList(), p);
	});
	lst.append(detectHexaTarget);


	QAction* printRelPose = new QAction(tr("Print relative poses"), parent);
	connect(printRelPose, &QAction::triggered, [p, imIds] () {
		QTextStream cout(stdout, QIODevice::WriteOnly);
		printImagesRelativePositions(cout, imIds, p);
	});

	lst.append(printRelPose);

	QAction* addToCalibration = createAddToCalibrationAction(parent, p, ims);

	lst.append(addToCalibration);

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [ims] () {
		for (Image* im : ims) {
			im->clearOptimized();
		}
	});
	lst.append(clearOptimized);

	return lst;

}


void ImageBaseActionManager::registerAppHeadlessActions(StereoVisionApplication* application) const {
    const char* ImageNamespace = "Image";

    application->registerHeadlessAction(ImageNamespace,"autoDetectTiePoints", autoDetectImagesTiePointsHeadless);

    const char* SparseMatchingNamespace = "SparseMatching";

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "configureModularHeadlessSparseMatchingPipeline",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(kwargs);
                                            Q_UNUSED(argv);
                                            return configureModularHeadlessSparseMatchingPipeline();
                                        });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "setupHeadlessHarrisCornerDetectorModule",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(argv);

                                            int lowPassRadius = 3;
                                            int nonMaximumSuppressionRadius = 3;
                                            int maxNCorners = 100;

                                            if (kwargs.contains("lowPassRadius")) {
                                                bool ok = true;
                                                lowPassRadius = kwargs["lowPassRadius"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument lowPassRadius has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("nonMaxSupprRadius")) {
                                                bool ok = true;
                                                nonMaximumSuppressionRadius = kwargs["nonMaxSupprRadius"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument nonMaxSupprRadius has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("maxNCorners")) {
                                                bool ok = true;
                                                maxNCorners = kwargs["maxNCorners"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument maxNCorners has invalid value (must be convertible to int)");
                                                }
                                            }

                                            return setupHeadlessHarrisCornerDetectorModule(lowPassRadius,
                                                                                           nonMaximumSuppressionRadius,
                                                                                           maxNCorners);
                                        });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "setupHeadlessHungarianCornerMatchModule",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(argv);

                                            int patchRadius = 3;
                                            int nSamples = 3;

                                            if (kwargs.contains("patchRadius")) {
                                                bool ok = true;
                                                patchRadius = kwargs["patchRadius"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument patchRadius has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("nSamples")) {
                                                bool ok = true;
                                                nSamples = kwargs["nSamples"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument nSamples has invalid value (must be convertible to int)");
                                                }
                                            }

                                            return setupHeadlessHungarianCornerMatchModule(patchRadius, nSamples);
                                        });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "setupHeadlessBestNCornerMatchModule",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(argv);

                                            int nMatches = 5;
                                            float maxRatio2Best = 1.2;

                                            if (kwargs.contains("nMatches")) {
                                                bool ok = true;
                                                nMatches = kwargs["nMatches"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument nMatches has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("maxRatio2Best")) {
                                                bool ok = true;
                                                maxRatio2Best = kwargs["maxRatio2Best"].toFloat(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument maxRatio2Best has invalid value (must be convertible to float)");
                                                }
                                            }

                                            return setupHeadlessBestNCornerMatchModule(nMatches, maxRatio2Best);
                                        });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "setupHeadlessRansacEpipolarInlinerSelectionModule",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(argv);

                                            int nRansacIterations = 500;
                                            float threshold = 0.01;

                                            if (kwargs.contains("nRansacIterations")) {
                                                bool ok = true;
                                                nRansacIterations = kwargs["nRansacIterations"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument nRansacIterations has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("threshold")) {
                                                bool ok = true;
                                                threshold = kwargs["threshold"].toFloat(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument threshold has invalid value (must be convertible to float)");
                                                }
                                            }

                                            return setupHeadlessRansacEpipolarInlinerSelectionModule(nRansacIterations, threshold);
                                        });


    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "setupHeadlessRansacPerspectiveInlinerSelectionModule",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(argv);

                                            int nRansacIterations = 500;
                                            bool enableMultiThresholding = false;
                                            float threshold = 50;
                                            float subthreshold = 5;

                                            if (kwargs.contains("nRansacIterations")) {
                                                bool ok = true;
                                                nRansacIterations = kwargs["nRansacIterations"].toInt(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument nRansacIterations has invalid value (must be convertible to int)");
                                                }
                                            }

                                            if (kwargs.contains("enableMultiThresholding")) {
                                                enableMultiThresholding = QVariant(kwargs["enableMultiThresholding"]).toBool();
                                            }

                                            if (kwargs.contains("threshold")) {
                                                bool ok = true;
                                                threshold = kwargs["threshold"].toFloat(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument threshold has invalid value (must be convertible to float)");
                                                }
                                            }

                                            if (kwargs.contains("subthreshold")) {
                                                bool ok = true;
                                                subthreshold = kwargs["subthreshold"].toFloat(&ok);

                                                if (!ok) {
                                                    return StatusOptionalReturn<void>::error("argument subthreshold has invalid value (must be convertible to float)");
                                                }
                                            }

                                            return setupHeadlessRansacPerspectiveInlinerSelectionModule(nRansacIterations, enableMultiThresholding, threshold, subthreshold);
                                        });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "addImageToHeadlessSparseMatching",
                                        [application] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(kwargs);

                                            if (argv.isEmpty()) {
                                                return StatusOptionalReturn<void>::error("missing image id, needed as one positional argument!)");
                                            }

                                            bool ok = true;
                                            qint64 imgId = argv[0].toInt(&ok);

                                            if (!ok) {
                                                return StatusOptionalReturn<void>::error("Invalid image id provided!)");
                                            }

                                            Project* proj = application->getCurrentProject();

                                            if (proj == nullptr) {
                                                return StatusOptionalReturn<void>::error("Missing current project in application!)");
                                            }

                                            return addImageToHeadlessSparseMatching(proj, imgId);
                                        });

        application->registerHeadlessAction(SparseMatchingNamespace,
                                            "writeHeadlessMatchingResultsToFile",
                                            [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(kwargs);

                                                if (argv.isEmpty()) {
                                                    return StatusOptionalReturn<void>::error("missing output file path argument, needed as one positional argument!)");
                                                }

                                                QString filePath = argv[0];

                                                return printHeadlessSparseMatchingResults(filePath);
                                            });

    application->registerHeadlessAction(SparseMatchingNamespace,
                                        "getHeadlessSparseMatchingResults",
                                        [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                                Q_UNUSED(kwargs);
                                                Q_UNUSED(argv);
                                            return getHeadlessSparseMatchingResults();
                                        });

        application->registerHeadlessAction(SparseMatchingNamespace,
                                            "runHeadlessMatching",
                                            [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                            Q_UNUSED(kwargs);
                                            Q_UNUSED(argv);
                                                return runHeadlessMatching();
                                            });

        application->registerHeadlessAction(SparseMatchingNamespace,
                                            "exportHeadlessSparseMatchingResultsView",
                                            [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {
                                                Q_UNUSED(kwargs);
                                                if (argv.isEmpty()) {
                                                    return StatusOptionalReturn<void>::error("missing output file path argument, needed as one positional argument!)");
                                                }

                                                QString outFilePath = argv[0];

                                                return exportHeadlessSparseMatchingResultsView(outFilePath);
                                            });


}

QAction* ImageBaseActionManager::createAssignToCameraAction(QObject* parent, Project* p, QVector<Image*> const& ims) const {

	QAction* assignToCamera = new QAction(tr("Assign to camera"), parent);
	QMenu* camMenu = new QMenu();
	connect(assignToCamera, &QObject::destroyed, camMenu, &QObject::deleteLater);

	QVector<qint64> camsIds = p->getIdsByClass(CameraFactory::cameraClassName());

	for(qint64 camId : camsIds) {

		Camera* c = qobject_cast<Camera*>(p->getById(camId));

		if (c != nullptr) {
			QAction* toCam = new QAction(c->objectName(), assignToCamera);
			connect(toCam, &QAction::triggered, [camId, ims] () {
				for (Image* im : ims) {
					im->assignCamera(camId);
				}
			});
			camMenu->addAction(toCam);
		}
	}
	assignToCamera->setMenu(camMenu);

	return assignToCamera;

}
QAction* ImageBaseActionManager::createAddToCalibrationAction(QObject* parent, Project* p, const QVector<Image *> &ims) const {

	QAction* addToCalibration = new QAction(tr("Add to Calibration"), parent);
	QMenu* calibMenu = new QMenu();
	connect(addToCalibration, &QObject::destroyed, calibMenu, &QObject::deleteLater);

	QVector<qint64> calibsIds = p->getIdsByClass(CameraCalibration::staticMetaObject.className());

	QList<qint64> imsIds;
	imsIds.reserve(ims.size());

	for (Image* img : ims) {
		imsIds.push_back(img->internalId());
	}

	for(qint64 calibId : calibsIds) {

		CameraCalibration* c = qobject_cast<CameraCalibration*>(p->getById(calibId));

		if (c != nullptr) {
			QAction* toCam = new QAction(c->objectName(), addToCalibration);
			connect(toCam, &QAction::triggered, [calibId, imsIds, p] () {
				addImagesToCalibration(imsIds, calibId, p);
			});
			calibMenu->addAction(toCam);
		}
	}
	addToCalibration->setMenu(calibMenu);

	return addToCalibration;

}

QAction* ImageBaseActionManager::createAssignToStereoRigAction(QObject* parent, Project* p, const QVector<Image *> &ims) const {

	bool ok = true;
	int countActions = 0;

	QAction* assignToStereoRig = new QAction(tr("Assign to stereo rig"), parent);

	if (ims.size() != 2) {
		assignToStereoRig->deleteLater();
		return nullptr;
	}

	QMenu* rigMenu = new QMenu();
	connect(assignToStereoRig, &QObject::destroyed, rigMenu, &QObject::deleteLater);

	QVector<qint64> rigsIds = p->getIdsByClass(StereoRigFactory::StereoRigClassName());

	for (qint64 rigId : rigsIds) {

		StereoRig* rig = qobject_cast<StereoRig*>(p->getById(rigId));

		if (rig != nullptr) {

			ImagePair* pairImg1 = rig->getPairForImage(ims[0]->internalId());
			ImagePair* pairImg2 = rig->getPairForImage(ims[1]->internalId());

			if ( pairImg1 != nullptr and
					pairImg2 != nullptr) {

				if (pairImg1 == pairImg2) {
					ok = false;
					break;
				}
			}

			if (pairImg1 != nullptr or
					pairImg2 != nullptr) {
				continue;
			}

			QAction* toRig = new QAction(rig->objectName(), assignToStereoRig);
			connect(toRig, &QAction::triggered, [rig, ims] () {
				rig->insertImagePair(ims[0]->internalId(), ims[1]->internalId());
			});
			rigMenu->addAction(toRig);
			countActions++;
		}

	}

	if (ok and countActions > 0) {
		assignToStereoRig->setMenu(rigMenu);
		return assignToStereoRig;
	} else {
		assignToStereoRig->deleteLater();
		return nullptr;
	}

	return nullptr;

}

QAction* ImageBaseActionManager::createOpenInCornerMatchingEditor(QObject* parent, Project* p, const QVector<qint64> &imIds) const {

    QAction* testMatchCorners = new QAction(tr("Add to corner matching editor"), parent);
    connect(testMatchCorners, &QAction::triggered, [p, imIds] () {
        addImages2MatchCornersEditor(p, imIds);
    });

    return testMatchCorners;
}

QAction* ImageBaseActionManager::createExportRectifiedForRigAction(QObject* parent, Project* p, const QVector<qint64> &ims, QWidget* w) const {

	bool ok = false;

	QAction* exportForStereoRig = new QAction(tr("Export rectified for rig"), parent);

	if (ims.size() != 2) {
		exportForStereoRig->deleteLater();
		return nullptr;
	}

	QMenu* rigMenu = new QMenu();
	connect(exportForStereoRig, &QObject::destroyed, rigMenu, &QObject::deleteLater);

	QVector<qint64> rigsIds = p->getIdsByClass(StereoRigFactory::StereoRigClassName());

	for (qint64 rigId : rigsIds) {

		StereoRig* rig = qobject_cast<StereoRig*>(p->getById(rigId));

		if (rig != nullptr) {

			ImagePair* pair = rig->getPairForImage(ims[0]);

			if (pair != nullptr) {

				if (pair->idImgCam1() == ims[1] or
						pair->idImgCam2() == ims[1]) {

					QAction* forRig = new QAction(rig->objectName(), exportForStereoRig);
					connect(forRig, &QAction::triggered, [rigId, ims, p, w] () {
						exportStereoRigRectifiedImages(ims.toList(), rigId, p, w);
					});
					rigMenu->addAction(forRig);
					ok = true;
				}
			}
		}

	}

	if (ok) {
		exportForStereoRig->setMenu(rigMenu);
		return exportForStereoRig;
	} else {
		exportForStereoRig->deleteLater();
		return nullptr;
	}

	return nullptr;
}

} // namespace StereoVisionApp
