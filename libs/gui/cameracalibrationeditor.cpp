#include "cameracalibrationeditor.h"
#include "ui_cameracalibrationeditor.h"

#include "datablocks/image.h"
#include "datablocks/cameracalibration.h"

#include "vision/checkboarddetector.h"

#include "stepprocessmonitorbox.h"

#include <QAction>
#include <QMenu>

#include <optional>

#include <QDebug>

namespace StereoVisionApp {

CameraCalibrationEditor::CameraCalibrationEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::CameraCalibrationEditor),
	_currentId(0),
	_currentCalib(nullptr)
{
	ui->setupUi(this);

	_checkBoardDrawer = new CheckboardPtsDrawable(this);
	ui->imageView->addDrawable(_checkBoardDrawer);

	_moveToNextImg = new QAction(tr("Next image"), this);
	_moveToPrevImg = new QAction(tr("Previous image"), this);

	_moveToNextImg->setShortcut(Qt::CTRL + Qt::Key_Tab);
	_moveToPrevImg->setShortcut(Qt::CTRL + Qt::Key_Shift + Qt::Key_Tab);

	ui->imageView->displayPoints(false);

	connect(ui->imageView, &ImageWidget::menuNonPointClick, this, &CameraCalibrationEditor::openContextMenu);
	connect(ui->imgsListView, &QListView::clicked, this, static_cast<void(CameraCalibrationEditor::*)(QModelIndex)>(&CameraCalibrationEditor::moveToImage));

	ui->imgsListView->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(ui->imgsListView, &QListView::customContextMenuRequested, this, &CameraCalibrationEditor::imgsListContextMenu);

	connect(_moveToNextImg, &QAction::triggered, this, &CameraCalibrationEditor::moveToNextImage);
	connect(_moveToPrevImg, &QAction::triggered, this, &CameraCalibrationEditor::moveToPreviousImage);

	connect(ui->detectCheckboardsButton, &QPushButton::clicked, this, &CameraCalibrationEditor::detectCheckBoardsAllImages);
}

CameraCalibrationEditor::~CameraCalibrationEditor()
{
	delete ui;
}

void CameraCalibrationEditor::setCalibration(CameraCalibration* calib) {

	if (calib == nullptr) {
		ui->imageView->setImage(nullptr);
		ui->imgsListView->setModel(nullptr);
		_checkBoardDrawer->setCalibration(nullptr);
		_currentCalib = nullptr;
		return;
	}

	ui->imgsListView->setModel(calib->imageList());
	QList<qint64> imgsIds = calib->loadedImages();

	_currentCalib = calib;

	_checkBoardDrawer->setCalibration(calib);

	if (imgsIds.size() > 0) {
		moveToImage(0);
	} else {
		ui->imageView->setImage(nullptr);
	}

}


void CameraCalibrationEditor::openContextMenu(QPoint const& pos) {

	QMenu cMenu;

	cMenu.addAction(_moveToNextImg);
	cMenu.addAction(_moveToPrevImg);

	cMenu.exec(ui->imageView->mapToGlobal(pos));
}

void CameraCalibrationEditor::moveToNextImage() {

	int row = _currentId;
	int nRows = ui->imgsListView->model()->rowCount();

	row++;

	if (row >= nRows) {
		row = 0;
	}

	moveToImage(row);
}
void CameraCalibrationEditor::moveToPreviousImage() {

	int row = _currentId;
	int nRows = ui->imgsListView->model()->rowCount();

	row--;

	if (row < 0) {
		row = nRows-1;
	}

	moveToImage(row);
}
void CameraCalibrationEditor::moveToImage(int row) {

	if (_currentCalib == nullptr) {
		return;
	}

	if (row != _currentId) {
		QModelIndex next = ui->imgsListView->model()->index(row, 0);

		if (next.isValid()) {
			qint64 id = next.data(Project::IdRole).toInt();

			Project* proj = _currentCalib->getProject();

			if (proj != nullptr) {
				Image* img = proj->getDataBlock<Image>(id);

				if (img != nullptr) {
					ui->imageView->setImage(img);
					_checkBoardDrawer->setImgId(id);
					_currentId = row;
				}

				ui->labelNCandidates->setText("");
				ui->labelNSelected->setText("");
				ui->labelGridSize->setText("");

				auto candidates = _currentCalib->getDetectedCandidates(_currentId);

				if (candidates.has_value()) {
					ui->labelNCandidates->setText(QString("%0").arg(candidates.value().size()));
				}

				auto filtered = _currentCalib->getFilteredCandidates(_currentId);

				if (filtered.has_value()) {
					ui->labelNSelected->setText(QString("%0").arg(filtered.value().size()));
				}

				auto grid = _currentCalib->getImageCorners(_currentId);

				if (grid.has_value()) {
					ui->labelGridSize->setText(QString("%0").arg(grid.value().size()));
				}
			}
		}
	}

}

void CameraCalibrationEditor::moveToImage(QModelIndex idx) {

	if (idx.isValid()) {
		if (idx.model() == ui->imgsListView->model()) {
			moveToImage(idx.row());
		}
	}

}


void CameraCalibrationEditor::detectCheckBoardsAllImages() {
	detectCheckBoards(-1);
}

void CameraCalibrationEditor::detectCheckBoards(qint64 id) {

	if (_currentCalib == nullptr) {
		return;
	}

	Project* proj = _currentCalib->getProject();

	if (proj == nullptr) {
		return;
	}

	QVector<qint64> idxs;

	if (id >= 0) {
		idxs.push_back(id);
	} else {
		auto loaded = _currentCalib->loadedImages();
		idxs = QVector<qint64>(loaded.begin(), loaded.end());
	}

	if (idxs.isEmpty()) {
		return;
	}

	CheckboardDetector * checkBoardDetector = new CheckboardDetector();


	checkBoardDetector->setLambda_threshold(ui->lambdaThresholdSpinBox->value());
	checkBoardDetector->setFilter_failure_threshold(ui->softThresholdSpinBox->value()/100.);
	checkBoardDetector->setFilter_error_threshold(ui->hardThresholdSpinBox->value()/100.);

	checkBoardDetector->setProject(proj);
	checkBoardDetector->addImagesToProcess(idxs);

	connect(checkBoardDetector, &CheckboardDetector::processedImage, _currentCalib, &CameraCalibration::setImageCorners, Qt::DirectConnection);
	connect(checkBoardDetector, &CheckboardDetector::finished, ui->imageView, static_cast<void(QWidget::*)()>(&QWidget::update));

	QThread* t = new QThread();

	checkBoardDetector->moveToThread(t);
	QObject::connect(checkBoardDetector, &QObject::destroyed, t, &QThread::quit);
	QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

	StepProcessMonitorBox* box = new StepProcessMonitorBox(this);
	box->setWindowFlag(Qt::Dialog);
	box->setWindowModality(Qt::WindowModal);
	box->setWindowTitle(QObject::tr("Detect checkboards"));

	box->setProcess(checkBoardDetector);

	QObject::connect(box, &QObject::destroyed, checkBoardDetector, &QObject::deleteLater);

	box->show();

	t->start();
	checkBoardDetector->run();

}

void  CameraCalibrationEditor::imgsListContextMenu(QPoint const& position) {

	QModelIndex idx = ui->imgsListView->indexAt(position);

	if (!idx.isValid()) {
		return;
	}

	bool ok;
	qint64 imgId = idx.data(Project::IdRole).toInt(&ok);

	if (!ok) {
		return;
	}

	QMenu menu;

	QAction* treatSingleImg = menu.addAction(tr("Detect checkboard"));

	connect(treatSingleImg, &QAction::triggered, this, [this, imgId] () {
		detectCheckBoards(imgId);
	});

	menu.exec(ui->imgsListView->mapToGlobal(position));

}


CheckboardPtsDrawable::CheckboardPtsDrawable(QWidget *parent):
	ImageWidgetDrawable(parent),
	_id(-1),
	_calib(nullptr)
{

}

void CheckboardPtsDrawable::setCalibration(CameraCalibration* calib) {
	_calib = calib;
}
void CheckboardPtsDrawable::setImgId(qint64 id) {
	_id = id;
}

void CheckboardPtsDrawable::paintItemImpl(QPainter* painter) const {

	if (painter == nullptr) {
		return;
	}

	if (_calib == nullptr) {
		return;
	}

	if (!_calib->hasImage(_id)) {
		return;
	}

	auto cand = _calib->getDetectedCandidates(_id);

	if (cand.has_value()) {

		QVector<StereoVision::discretCheckCornerInfos> corners = cand.value();

		QList<QLineF> arrows;
		arrows.reserve(corners.size());

		for (StereoVision::discretCheckCornerInfos const& corner : qAsConst(corners)) {
			QPointF pt1(corner.pix_coord_x+0.5, corner.pix_coord_y+0.5);
			QPointF pt2(corner.pix_coord_x+0.5+3*std::cos(corner.main_dir), corner.pix_coord_y+0.5+3*std::sin(corner.main_dir));
			arrows.push_back(QLineF(pt1, pt2));
		}

		drawLines(painter, arrows, QColor(200, 10, 10), 2);
	}

	auto sel = _calib->getFilteredCandidates(_id);

	if (sel.has_value()) {

		QVector<StereoVision::discretCheckCornerInfos> corners = sel.value();

		QList<QLineF> arrows;
		arrows.reserve(corners.size());

		for (StereoVision::discretCheckCornerInfos const& corner : qAsConst(corners)) {
			QPointF pt1(corner.pix_coord_x+0.5, corner.pix_coord_y+0.5);
			QPointF pt2(corner.pix_coord_x+0.5+3*std::cos(corner.main_dir), corner.pix_coord_y+0.5+3*std::sin(corner.main_dir));
			arrows.push_back(QLineF(pt1, pt2));
		}

		drawLines(painter, arrows, QColor(200, 200, 10), 2);
	}

	auto val = _calib->getImageCorners(_id);

	if (val.has_value()) {

		QVector<StereoVision::refinedCornerInfos> corners = val.value();

		QPolygonF points;
		points.reserve(corners.size());

		for (StereoVision::refinedCornerInfos const& corner : qAsConst(corners)) {
			points.push_back(QPointF(corner.pix_coord_x+0.5, corner.pix_coord_y+0.5));
		}

		drawPoints(painter, points, QColor(10, 200, 10), 5);
	}

}

CameraCalibrationEditorFactory::CameraCalibrationEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}

QString CameraCalibrationEditorFactory::TypeDescrName() const {
	return tr("Camera Calibration");
}
QString CameraCalibrationEditorFactory::itemClassName() const {
	return CameraCalibrationEditor::staticMetaObject.className();
}
Editor* CameraCalibrationEditorFactory::factorizeEditor(QWidget* parent) const {
	return new CameraCalibrationEditor(parent);
}

} // namespace StereoVisionApp
