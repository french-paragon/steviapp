#include "solutioninitconfigdialog.h"
#include "ui_solutioninitconfigdialog.h"

#include <QPushButton>

#include "datablocks/project.h"
#include "datablocks/image.h"

namespace StereoVisionApp {

SolutionInitConfigDialog::SolutionInitConfigDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::SolutionInitConfigDialog),
	_p(nullptr)
{
	ui->setupUi(this);

	connect(ui->setSolutionButton, &QPushButton::pressed, this, &SolutionInitConfigDialog::onRunButtonPressed);
	connect(ui->cancelButton, &QPushButton::pressed, this, &SolutionInitConfigDialog::onCancelButtonPressed);

	_keys.clear();
	reloadImageList();
}

SolutionInitConfigDialog::~SolutionInitConfigDialog()
{
	delete ui;
}

void SolutionInitConfigDialog::onRunButtonPressed() {
	accept();
}
void SolutionInitConfigDialog::onCancelButtonPressed() {
	reject();
}

quint64 SolutionInitConfigDialog::selectedStartingImage() const {

	if (ui->imageSelectComboBox->currentIndex() < _keys.size() and
			ui->imageSelectComboBox->currentIndex() >= 0) {
		return _keys[ui->imageSelectComboBox->currentIndex()];
	}

	return -1;

}

void SolutionInitConfigDialog::setProject(Project *p)
{
	_p = p;
	reloadImageList();
}

void SolutionInitConfigDialog::reloadImageList() {

	QStringList names;
	int prevKey;

	if (_keys.size() > ui->imageSelectComboBox->currentIndex() and
			ui->imageSelectComboBox->currentIndex() >= 0) {
		prevKey = _keys[ui->imageSelectComboBox->currentIndex()];
	} else {
		prevKey = -1;
	}

	_keys.clear();
	names.clear();

	_keys.push_back(-1);
	names.push_back(tr("Automatic"));

	if (_p != nullptr) {

		for (qint64 id : _p->getIdsByClass(ImageFactory::imageClassName())) {
			Image* im = qobject_cast<Image*>(_p->getById(id));

			if (im == nullptr) {
				continue;
			}

			_keys.push_back(id);
			names.push_back(im->objectName());
		}

	}

	int pos = _keys.indexOf(prevKey);

	if (pos > _keys.size() or pos < 0) {
		pos = 0;
	}

	ui->imageSelectComboBox->clear();
	ui->imageSelectComboBox->addItems(names);
	ui->imageSelectComboBox->setCurrentIndex(pos);

}

} // namespace StereoVisionApp
