#include "sparsestereosolverconfigdialog.h"
#include "ui_sparsestereosolverconfigdialog.h"

#include "datablocks/project.h"
#include "datablocks/image.h"

namespace StereoVisionApp {

SparseStereoSolverConfigDialog::SparseStereoSolverConfigDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::SparseStereoSolverConfigDialog),
	_shouldRun(false)
{
	ui->setupUi(this);

	connect(ui->RunButton, &QPushButton::clicked, this, &SparseStereoSolverConfigDialog::onRunButtonPressed);
	connect(ui->cancelButton, &QPushButton::clicked, this, &SparseStereoSolverConfigDialog::onCancelButtonPressed);
}

SparseStereoSolverConfigDialog::~SparseStereoSolverConfigDialog()
{
	delete ui;
}

void SparseStereoSolverConfigDialog::onRunButtonPressed() {
	_shouldRun = true;
	accept();
}
void SparseStereoSolverConfigDialog::onCancelButtonPressed() {
	_shouldRun = false;
	reject();
}

bool SparseStereoSolverConfigDialog::shouldRun() const
{
	return _shouldRun;
}

bool SparseStereoSolverConfigDialog::computeUncertainty() const {
	return ui->predictUncertaintyCheckBox->isChecked() and useSparseOptimizer();
}
bool SparseStereoSolverConfigDialog::useSparseOptimizer() const {
	return ui->useSparseOptimizerCheckBox->isChecked();
}

int SparseStereoSolverConfigDialog::numberOfSteps() const {
	return ui->nStepSpinBoxpinBox->value();
}

void SparseStereoSolverConfigDialog::setProject(Project *p) {
	_p = p;
	reloadImageList();
}

quint64 SparseStereoSolverConfigDialog::selectedStartingImage1() const {

	if (ui->image1SelectComboBox->currentIndex() < _keys.size() and
			ui->image1SelectComboBox->currentIndex() >= 0) {
		return _keys[ui->image1SelectComboBox->currentIndex()];
	}

	return -1;
}
quint64 SparseStereoSolverConfigDialog::selectedStartingImage2() const {

	if (ui->image2SelectComboBox->currentIndex() < _keys.size() and
			ui->image2SelectComboBox->currentIndex() >= 0) {
		return _keys[ui->image2SelectComboBox->currentIndex()];
	}

	return -1;
}

void SparseStereoSolverConfigDialog::reloadImageList() {
	QStringList names;
	int prevKey1;
	int prevKey2;

	if (_keys.size() > ui->image1SelectComboBox->currentIndex() and
			ui->image1SelectComboBox->currentIndex() >= 0) {
		prevKey1 = _keys[ui->image1SelectComboBox->currentIndex()];
	} else {
		prevKey1 = -1;
	}

	if (_keys.size() > ui->image2SelectComboBox->currentIndex() and
			ui->image2SelectComboBox->currentIndex() >= 0) {
		prevKey2 = _keys[ui->image2SelectComboBox->currentIndex()];
	} else {
		prevKey2 = -1;
	}

	_keys.clear();
	names.clear();

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

	int pos1 = _keys.indexOf(prevKey1);
	int pos2 = _keys.indexOf(prevKey2);

	if (pos1 > _keys.size() or pos1 < 0) {
		pos1 = 0;
	}
	if (pos2 > _keys.size() or pos2 < 0) {
		pos2 = 0;
	}

	ui->image1SelectComboBox->clear();
	ui->image1SelectComboBox->addItems(names);
	ui->image1SelectComboBox->setCurrentIndex(pos1);

	ui->image2SelectComboBox->clear();
	ui->image2SelectComboBox->addItems(names);
	ui->image2SelectComboBox->setCurrentIndex(pos2);
}

} //namespace StereoVisionApp
