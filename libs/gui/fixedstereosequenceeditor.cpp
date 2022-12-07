#include "fixedstereosequenceeditor.h"
#include "ui_fixedstereosequenceeditor.h"

#include "datablocks/project.h"

#include "datablocks/image.h"

#include "datablocks/fixedcolorstereosequence.h"
#include "datablocks/fixedstereopluscolorsequence.h"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>

#include <QDebug>
#include <QFileDialog>

namespace StereoVisionApp {

FixedStereoSequenceEditor::FixedStereoSequenceEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::FixedStereoSequenceEditor),
	_sequence(nullptr)
{
	ui->setupUi(this);

	connect(ui->selectFolderButton, &QPushButton::clicked, this, [this] () {

		QString dir = QFileDialog::getExistingDirectory(this, tr("Image sequence directory"));

		if (!dir.isEmpty()) {
			loadImages(dir);
		}

	});

	connect(ui->leftViewSelect, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, static_cast<void(FixedStereoSequenceEditor::*)(int)>(&FixedStereoSequenceEditor::leftViewSelectionChanged));

	connect(ui->rgbViewSelect, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, static_cast<void(FixedStereoSequenceEditor::*)(int)>(&FixedStereoSequenceEditor::rgbViewSelectionChanged));

	connect(ui->rightViewSelect, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, static_cast<void(FixedStereoSequenceEditor::*)(int)>(&FixedStereoSequenceEditor::rightViewSelectionChanged));
}

FixedStereoSequenceEditor::~FixedStereoSequenceEditor()
{
	delete ui;
}

void FixedStereoSequenceEditor::setSequence(DataBlock* sequence) {

	if (_sequence != nullptr) {
		FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
		FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

		if (fixedColor != nullptr) {
			disconnect(fixedColor, &FixedColorStereoSequence::imgListChanged, this, &FixedStereoSequenceEditor::refreshDisplay);
		}

		if (fixedStereoPlusColor != nullptr) {
			disconnect(fixedStereoPlusColor, &FixedStereoPlusColorSequence::imgListChanged, this, &FixedStereoSequenceEditor::refreshDisplay);
		}

		ui->imgListView->setModel(nullptr);
	}

	_sequence = nullptr;

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(sequence);

	if (fixedColor != nullptr) {
		connect(fixedColor, &FixedColorStereoSequence::imgListChanged, this, &FixedStereoSequenceEditor::refreshDisplay);
		ui->imgListView->setModel(fixedColor->getImageList());
		ui->folderLineEdit->setText(fixedColor->baseFolder());

		ui->rgb_suffix_edit->setVisible(false);
		ui->rgb_suffix_label->setVisible(false);

		ui->rgbViewSelect->setVisible(false);
		ui->rgbViewSelectLabel->setVisible(false);

		Project* p = fixedColor->getProject();

		ui->leftViewSelect->setModel(p);
		ui->leftViewSelect->setRootModelIndex(p->indexOfClass(Image::staticMetaObject.className()));

		qint64 leftViewId = fixedColor->leftViewId();

		if (leftViewId != -1) {
			QVector<qint64> ids = fixedColor->getProject()->getIdsByClass(Image::staticMetaObject.className());
			ui->leftViewSelect->setCurrentIndex(ids.indexOf(leftViewId));
		} else {
			ui->leftViewSelect->setCurrentIndex(-1);
		}

		ui->rightViewSelect->setModel(p);
		ui->rightViewSelect->setRootModelIndex(p->indexOfClass(Image::staticMetaObject.className()));

		qint64 rightViewId = fixedColor->rightViewId();

		if (rightViewId != -1) {
			QVector<qint64> ids = fixedColor->getProject()->getIdsByClass(Image::staticMetaObject.className());
			ui->rightViewSelect->setCurrentIndex(ids.indexOf(rightViewId));
		} else {
			ui->rightViewSelect->setCurrentIndex(-1);
		}

		_sequence = sequence;
	}

	if (fixedStereoPlusColor != nullptr) {
		connect(fixedStereoPlusColor, &FixedStereoPlusColorSequence::imgListChanged, this, &FixedStereoSequenceEditor::refreshDisplay);
		ui->imgListView->setModel(fixedStereoPlusColor->getImageList());
		ui->folderLineEdit->setText(fixedStereoPlusColor->baseFolder());

		ui->rgb_suffix_edit->setVisible(true);
		ui->rgb_suffix_label->setVisible(true);

		ui->rgbViewSelect->setVisible(true);
		ui->rgbViewSelectLabel->setVisible(true);

		Project* p = fixedStereoPlusColor->getProject();

		ui->leftViewSelect->setModel(p);
		ui->leftViewSelect->setRootModelIndex(p->indexOfClass(Image::staticMetaObject.className()));

		qint64 leftViewId = fixedStereoPlusColor->leftViewId();

		if (leftViewId != -1) {
			QVector<qint64> ids = fixedStereoPlusColor->getProject()->getIdsByClass(Image::staticMetaObject.className());
			ui->leftViewSelect->setCurrentIndex(ids.indexOf(leftViewId));
		} else {
			ui->leftViewSelect->setCurrentIndex(-1);
		}

		ui->rgbViewSelect->setModel(p);
		ui->rgbViewSelect->setRootModelIndex(p->indexOfClass(Image::staticMetaObject.className()));

		qint64 rgbViewId = fixedStereoPlusColor->rgbViewId();

		if (rgbViewId != -1) {
			QVector<qint64> ids = fixedStereoPlusColor->getProject()->getIdsByClass(Image::staticMetaObject.className());
			ui->rgbViewSelect->setCurrentIndex(ids.indexOf(rgbViewId));
		} else {
			ui->rgbViewSelect->setCurrentIndex(-1);
		}

		ui->rightViewSelect->setModel(p);
		ui->rightViewSelect->setRootModelIndex(p->indexOfClass(Image::staticMetaObject.className()));

		qint64 rightViewId = fixedStereoPlusColor->rightViewId();

		if (rightViewId != -1) {
			QVector<qint64> ids = fixedStereoPlusColor->getProject()->getIdsByClass(Image::staticMetaObject.className());
			ui->rightViewSelect->setCurrentIndex(ids.indexOf(rightViewId));
		} else {
			ui->rightViewSelect->setCurrentIndex(-1);
		}

		_sequence = sequence;
	}


}

void FixedStereoSequenceEditor::refreshDisplay() {

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

	if (fixedColor != nullptr) {
		ui->folderLineEdit->setText(fixedColor->baseFolder());
	}

	if (fixedStereoPlusColor != nullptr) {
		ui->folderLineEdit->setText(fixedStereoPlusColor->baseFolder());
	}


}

void FixedStereoSequenceEditor::loadImages(QString dirPath) {

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

	QString left_suffix = ui->left_suffix_edit->text();
	QString right_suffix = ui->right_suffix_edit->text();
	QString rgb_suffix = ui->rgb_suffix_edit->text();

	if (left_suffix.isEmpty()) {
		return;
	}

	if (right_suffix.isEmpty()) {
		return;
	}

	if (fixedStereoPlusColor != nullptr and rgb_suffix.isEmpty()) {
		return;
	}

	if (fixedColor == nullptr and fixedStereoPlusColor == nullptr) {
		return;
	}

	QDir directory(dirPath);

	if (!directory.exists()) {
		return;
	}

	QDirIterator it(directory);

	using flagtype = uint8_t;

	constexpr flagtype LeftFlag = 0b001;
	constexpr flagtype RightFlag = 0b010;
	constexpr flagtype RgbFlag = 0b100;
	constexpr flagtype StereoFlags = 0b011;
	constexpr flagtype AllFlags = 0b111;

	QMap<QString, flagtype> files;

	while (it.hasNext()) {
		QFileInfo info(it.next());

		if (!info.isFile()) {
			continue;
		}

		QString lowerFileExt = info.completeSuffix().toLower();

		if (lowerFileExt != "png" and
				lowerFileExt != "jpg" and
				lowerFileExt != "jpeg" and
				lowerFileExt != "bmp") {
			continue;
		}

		QString basename = info.baseName();

		bool isLeft = basename.endsWith(left_suffix);
		bool isRight = basename.endsWith(right_suffix);
		bool isRGB = fixedStereoPlusColor != nullptr and basename.endsWith(rgb_suffix);

		if (isLeft) {
			QString suffixFree = basename.left(basename.size()-left_suffix.size());
			suffixFree += "." + info.completeSuffix();

			files[suffixFree] = files.value(suffixFree, 0)|LeftFlag;
		}

		if (isRight) {
			QString suffixFree = basename.left(basename.size()-right_suffix.size());
			suffixFree += "." + info.completeSuffix();

			files[suffixFree] = files.value(suffixFree, 0)|RightFlag;
		}

		if (isRGB) {
			QString suffixFree = basename.left(basename.size()-rgb_suffix.size());
			suffixFree += "." + info.completeSuffix();

			files[suffixFree] = files.value(suffixFree, 0)|RgbFlag;
		}
	}

	if (fixedStereoPlusColor != nullptr) {

		QVector<FixedStereoPlusColorSequence::ImageTriplet> triplets;

		for (QString suffixFree : files.keys()) {

			if (files[suffixFree] != AllFlags) {
				continue;
			}

			QFileInfo fInfos(suffixFree);

			FixedStereoPlusColorSequence::ImageTriplet triplet;
			triplet.StereoLeftImgPath = fInfos.baseName() + left_suffix + "." + fInfos.completeSuffix();
			triplet.StereoRightImgPath = fInfos.baseName() + right_suffix + "." + fInfos.completeSuffix();
			triplet.ColorImagePath = fInfos.baseName() + rgb_suffix + "." + fInfos.completeSuffix();

			triplets.push_back(triplet);

		}

		fixedStereoPlusColor->setImgsLists(directory.absolutePath(), triplets);

	}

	if (fixedColor != nullptr) {

		QVector<FixedColorStereoSequence::ImagePair> pairs;

		for (QString suffixFree : files.keys()) {

			if (files[suffixFree] != StereoFlags) {
				continue;
			}

			QFileInfo fInfos(suffixFree);

			FixedColorStereoSequence::ImagePair pair;
			pair.StereoLeftImgPath = fInfos.baseName() + left_suffix + "." + fInfos.completeSuffix();
			pair.StereoRightImgPath = fInfos.baseName() + right_suffix + "." + fInfos.completeSuffix();

			pairs.push_back(pair);

		}

		fixedColor->setImgsLists(directory.absolutePath(), pairs);

	}

}



void FixedStereoSequenceEditor::leftViewSelectionChanged(int row) {

	QModelIndex parent = ui->leftViewSelect->rootModelIndex();
	QModelIndex selection = ui->leftViewSelect->model()->index(row, 0, parent);

	if (!selection.isValid()) {
		return;
	}

	QVariant cand = selection.data(Project::IdRole);

	if (!cand.isValid()) {
		return;
	}

	if (!cand.canConvert<qint64>()) {
		return;
	}

	qint64 id = qvariant_cast<qint64>(cand);

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

	if (fixedColor != nullptr) {
		fixedColor->setLeftViewId(id);
	}

	if (fixedStereoPlusColor != nullptr) {
		fixedStereoPlusColor->setLeftViewId(id);
	}

}
void FixedStereoSequenceEditor::rgbViewSelectionChanged(int row) {


	QModelIndex parent = ui->rgbViewSelect->rootModelIndex();
	QModelIndex selection = ui->rgbViewSelect->model()->index(row, 0, parent);

	if (!selection.isValid()) {
		return;
	}

	QVariant cand = selection.data(Project::IdRole);

	if (!cand.isValid()) {
		return;
	}

	if (!cand.canConvert<qint64>()) {
		return;
	}

	qint64 id = qvariant_cast<qint64>(cand);

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

	if (fixedColor != nullptr) {
		return;
	}

	if (fixedStereoPlusColor != nullptr) {
		fixedStereoPlusColor->setRgbViewId(id);
	}

}
void FixedStereoSequenceEditor::rightViewSelectionChanged(int row) {

	QModelIndex parent = ui->rightViewSelect->rootModelIndex();
	QModelIndex selection = ui->rightViewSelect->model()->index(row, 0, parent);

	if (!selection.isValid()) {
		return;
	}

	QVariant cand = selection.data(Project::IdRole);

	if (!cand.isValid()) {
		return;
	}

	if (!cand.canConvert<qint64>()) {
		return;
	}

	qint64 id = qvariant_cast<qint64>(cand);

	FixedColorStereoSequence* fixedColor = qobject_cast<FixedColorStereoSequence*>(_sequence);
	FixedStereoPlusColorSequence* fixedStereoPlusColor = qobject_cast<FixedStereoPlusColorSequence*>(_sequence);

	if (fixedColor != nullptr) {
		fixedColor->setRightViewId(id);
	}

	if (fixedStereoPlusColor != nullptr) {
		fixedStereoPlusColor->setRightViewId(id);
	}

}

FixedStereoSequenceEditorFactory::FixedStereoSequenceEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}


QString FixedStereoSequenceEditorFactory::TypeDescrName() const {
	return tr("Fixed stereo sequence editor");
}

QString FixedStereoSequenceEditorFactory::itemClassName() const {
	return FixedStereoSequenceEditor::staticMetaObject.className();
}

Editor* FixedStereoSequenceEditorFactory::factorizeEditor(QWidget* parent) const {
	return new FixedStereoSequenceEditor(parent);
}

}
