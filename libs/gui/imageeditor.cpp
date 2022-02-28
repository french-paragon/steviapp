#include "imageeditor.h"
#include "ui_imageeditor.h"

#include <QMenu>
#include <QInputDialog>
#include <QAction>
#include <QSettings>
#include <QDebug>

#include "datablocks/image.h"
#include "datablocks/landmark.h"

namespace StereoVisionApp {

const QString ImageEditor::ImageEditorClassName = "StereoVisionApp::ImageEditor";

ImageEditor::ImageEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::ImageEditor),
	_current_image_id(-1),
	_current_landmark_id(-1)
{
	ui->setupUi(this);

	QSettings s;

	QString pixUncertaintyKey = "ImageEditor/initialImageLandmarkUncerainty";
	qreal pix_uncertainty = s.value(pixUncertaintyKey, QVariant(1)).toReal();
	ui->imageLandmarkSigmaSpinBox->setValue(pix_uncertainty);

	connect(ui->imageLandmarkSigmaSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [pixUncertaintyKey] (double pix_sigma) {
		QSettings s;
		s.setValue(pixUncertaintyKey, pix_sigma);
	});

	_moveToNextImg = new QAction(tr("Next image"), this);
	_moveToPrevImg = new QAction(tr("Previous image"), this);

	_moveToNextLandmark = new QAction(tr("Next landmark"), this);
	_moveToPrevLandmark = new QAction(tr("Previous landmark"), this);

	_moveToNextImg->setShortcut(Qt::CTRL + Qt::Key_Tab);
	_moveToPrevImg->setShortcut(Qt::CTRL + Qt::Key_Shift + Qt::Key_Tab);

	_moveToNextLandmark->setShortcut(Qt::CTRL + Qt::Key_Right);
	_moveToPrevLandmark->setShortcut(Qt::CTRL + Qt::Key_Left);

	connect(ui->imageDisplay, &ImageWidget::newPointClick, this, &ImageEditor::addPoint);
	connect(ui->imageDisplay, &ImageWidget::menuPointClick, this, &ImageEditor::openPointMenu);
	connect(ui->imageDisplay, &ImageWidget::menuNonPointClick, this, &ImageEditor::openNonPointMenu);

	connect(ui->imageComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ImageEditor::onCurrentImageIndexChanged);
	connect(ui->landmarkComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ImageEditor::onCurrentLandmarkIndexChanged);

	connect(_moveToNextImg, &QAction::triggered, this, &ImageEditor::moveToNextImage);
	connect(_moveToPrevImg, &QAction::triggered, this, &ImageEditor::moveToPreviousImage);

	connect(ui->nextImageButton, &QPushButton::clicked, this, &ImageEditor::moveToNextImage);
	connect(ui->previousImageButton, &QPushButton::clicked, this, &ImageEditor::moveToPreviousImage);

	connect(_moveToNextLandmark, &QAction::triggered, this, &ImageEditor::moveToNextLandmark);
	connect(_moveToPrevLandmark, &QAction::triggered, this, &ImageEditor::moveToPreviousLandmark);

	connect(ui->nextLandmarkButton, &QPushButton::clicked, this, &ImageEditor::moveToNextLandmark);
	connect(ui->previousLandmarkButton, &QPushButton::clicked, this, &ImageEditor::moveToPreviousLandmark);

	addAction(_moveToNextImg);
	addAction(_moveToPrevImg);

	addAction(_moveToNextLandmark);
	addAction(_moveToPrevLandmark);
}

ImageEditor::~ImageEditor()
{
	delete ui;
}

void ImageEditor::setImage(Image* img) {
	ui->imageDisplay->setImage(img);

	Project* ap = activeProject();

	if (img == nullptr) {
		if (ui->imageComboBox->currentIndex() != -1) {
			ui->imageComboBox->setCurrentIndex(-1);
		}
		return;
	}

	if (ap != img->getProject()) {
		setProject(img->getProject());
	}

	qint64 id = img->internalId();

	QModelIndex imIndex = ap->indexOfClassInstance(ImageFactory::imageClassName(), id);

	if (imIndex != QModelIndex()) {
		if (imIndex.row() != ui->imageComboBox->currentIndex()) {
			ui->imageComboBox->setCurrentIndex(imIndex.row());
		}
	}
}

void ImageEditor::afterProjectChange(Project* op) {

	if (op != nullptr) {
		disconnect(op, &Project::modelAboutToBeReset, this, &ImageEditor::clearCombobox);
		disconnect(op, &Project::modelReset, this, &ImageEditor::setComboboxIndices);
	}

	if (activeProject() != nullptr) {
		connect(activeProject(), &Project::modelAboutToBeReset, this, &ImageEditor::clearCombobox);
		connect(activeProject(), &Project::modelReset, this, &ImageEditor::setComboboxIndices, Qt::QueuedConnection);
	}

	if (activeProject() != op) {
		clearCombobox();
	}

	setComboboxIndices();

}
void ImageEditor::clearCombobox() {

	ui->landmarkComboBox->blockSignals(true);
	ui->landmarkComboBox->setCurrentIndex(-1);
	ui->landmarkComboBox->clear();
	ui->landmarkComboBox->blockSignals(false);

	ui->imageComboBox->blockSignals(true);
	ui->imageComboBox->setCurrentIndex(-1);
	ui->imageComboBox->clear();
	ui->imageComboBox->blockSignals(false);
}

void ImageEditor::setComboboxIndices() {

	if (activeProject() == nullptr) {
		return;
	}

	qint64 currentLmIndex = _current_landmark_id;
	qint64 currentImIndex = _current_image_id;

	ui->landmarkComboBox->setModel(activeProject());
	ui->landmarkComboBox->setRootModelIndex(activeProject()->indexOfClass(LandmarkFactory::landmarkClassName()));

	if (currentLmIndex != -1) {
		QVector<qint64> ids = activeProject()->getIdsByClass(Landmark::staticMetaObject.className());
		ui->landmarkComboBox->setCurrentIndex(ids.indexOf(currentLmIndex));
	} else {
		ui->landmarkComboBox->setCurrentIndex(-1);
	}

	ui->imageComboBox->setModel(activeProject());
	ui->imageComboBox->setRootModelIndex(activeProject()->indexOfClass(ImageFactory::imageClassName()));

	if (currentImIndex != -1) {
		QVector<qint64> ids = activeProject()->getIdsByClass(Image::staticMetaObject.className());
		ui->imageComboBox->setCurrentIndex(ids.indexOf(currentImIndex));
	} else {
		ui->imageComboBox->setCurrentIndex(-1);
	}
}

void ImageEditor::addPoint(QPointF const& imageCoordinates) {

	Project* p = activeProject();

	if (p == nullptr) {
		return;
	}

	Image* im = ui->imageDisplay->currentImage();

	if (im == nullptr) {
		return;
	}

	QModelIndex rootLm = ui->landmarkComboBox->rootModelIndex();
	QModelIndex itemIndex = p->index(ui->landmarkComboBox->currentIndex(), 0, rootLm);

	qint64 lmId;

	if (itemIndex == QModelIndex()) { //landmark does not exist yet.

		QString newLmName = QInputDialog::getText(this, tr("New landmark"), tr("Name"), QLineEdit::Normal, tr("New landmark"));

		if (newLmName.isEmpty()) {
			return;
		}

		lmId = p->createDataBlock(LandmarkFactory::landmarkClassName().toStdString().c_str());

		if (lmId < 0) {
			return;
		}

		Landmark* lm = qobject_cast<Landmark*>(p->getById(lmId));
		lm->setObjectName(newLmName);

		ui->landmarkComboBox->setCurrentIndex(p->countTypeInstances(LandmarkFactory::landmarkClassName())-1);

	} else {

		lmId = p->data(itemIndex, Project::IdRole).toInt();
	}

	ImageLandmark* lm = im->getImageLandmarkByLandmarkId(lmId);

	if (lm == nullptr) {
		bool isUncertain = ui->imageLandmarkSigmaSpinBox->value() >= 0.01;
		lm = im->getImageLandmark(im->addImageLandmark(imageCoordinates, lmId, isUncertain, ui->imageLandmarkSigmaSpinBox->value()));
	} else {
		lm->setImageCoordinates(imageCoordinates);
	}
}

void ImageEditor::openPointMenu(qint64 id, QPoint const& pos) {

	Image* im = ui->imageDisplay->currentImage();

	QMenu cMenu;

	QAction* rm = new QAction(tr("remove"), &cMenu);

	connect(rm, &QAction::triggered, [im, id] () {
		im->clearImageLandmark(id);
	});

	cMenu.addAction(rm);

	cMenu.exec(ui->imageDisplay->mapToGlobal(pos));

}
void ImageEditor::openNonPointMenu(QPoint const& pos) {

	QMenu cMenu;

	cMenu.addAction(_moveToNextImg);
	cMenu.addAction(_moveToPrevImg);
	cMenu.addAction(_moveToNextLandmark);
	cMenu.addAction(_moveToPrevLandmark);

	cMenu.exec(ui->imageDisplay->mapToGlobal(pos));

}

void ImageEditor::moveToNextImage() {

	int comboId = ui->imageComboBox->currentIndex();
	comboId += 1;

	if (comboId >= ui->imageComboBox->count()) {
		comboId = -1;
	}

	ui->imageComboBox->setCurrentIndex(comboId);

}
void ImageEditor::moveToPreviousImage() {

	int comboId = ui->imageComboBox->currentIndex();
	comboId -= 1;

	if (comboId == -1) {
		comboId = ui->imageComboBox->count();
	}

	ui->imageComboBox->setCurrentIndex(comboId);
}

void ImageEditor::moveToNextLandmark() {

	int comboId = ui->landmarkComboBox->currentIndex();
	comboId += 1;

	if (comboId >= ui->landmarkComboBox->count()) {
		comboId = -1;
	}

	ui->landmarkComboBox->setCurrentIndex(comboId);
}
void ImageEditor::moveToPreviousLandmark() {

	int comboId = ui->landmarkComboBox->currentIndex();

	if (comboId == -1) {
		comboId = ui->landmarkComboBox->count();
	}

	comboId -= 1;

	ui->landmarkComboBox->setCurrentIndex(comboId);
}

void ImageEditor::onCurrentImageIndexChanged(int id) {

	Project* p = activeProject();

	if (p == nullptr) {
		return;
	}

	QModelIndex imagesIndex = activeProject()->indexOfClass(ImageFactory::imageClassName());
	QModelIndex targetId = p->index(id, 0, imagesIndex);

	if (targetId == QModelIndex()) {
		setImage(nullptr);
		return;
	}

	qint64 imageId = p->data(targetId, Project::IdRole).toInt();

	Image* nImg = qobject_cast<Image*>(p->getById(imageId));

	if (nImg != nullptr) {
		_current_image_id = nImg->internalId();
	} else {
		_current_image_id = -1;
	}

	setImage(nImg);

}

void ImageEditor::onCurrentLandmarkIndexChanged() {


	Project* p = activeProject();

	if (p == nullptr) {
		return;
	}

	QModelIndex rootLm = p->indexOfClass(Landmark::staticMetaObject.className());
	QModelIndex itemIndex = p->index(ui->landmarkComboBox->currentIndex(), 0, rootLm);

	if (itemIndex == QModelIndex()) {
		return;
	} else {
		_current_landmark_id = itemIndex.data(Project::IdRole).toInt();
	}

}


ImageEditorFactory::ImageEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}

QString ImageEditorFactory::TypeDescrName() const {
	return tr("Image Editor");
}
QString ImageEditorFactory::itemClassName() const {
	return ImageEditor::ImageEditorClassName;
}
Editor* ImageEditorFactory::factorizeEditor(QWidget* parent) const {
	return new ImageEditor(parent);
}

} // namespace StereoVisionApp
