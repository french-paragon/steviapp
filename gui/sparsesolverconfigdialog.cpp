#include "sparsesolverconfigdialog.h"
#include "ui_sparsesolverconfigdialog.h"

namespace StereoVisionApp {

SparseSolverConfigDialog::SparseSolverConfigDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::SparseSolverConfigDialog),
	_shouldRun(false)
{
	ui->setupUi(this);

	connect(ui->RunButton, &QPushButton::clicked, this, &SparseSolverConfigDialog::onRunButtonPressed);
	connect(ui->cancelButton, &QPushButton::clicked, this, &SparseSolverConfigDialog::onCancelButtonPressed);
}

SparseSolverConfigDialog::~SparseSolverConfigDialog()
{
	delete ui;
}

void SparseSolverConfigDialog::onRunButtonPressed() {
	_shouldRun = true;
	accept();
}
void SparseSolverConfigDialog::onCancelButtonPressed() {
	_shouldRun = false;
	reject();
}

bool SparseSolverConfigDialog::shouldRun() const
{
	return _shouldRun;
}

bool SparseSolverConfigDialog::computeUncertainty() const {
	return ui->predictUncertaintyCheckBox->isChecked();
}
bool SparseSolverConfigDialog::useSparseOptimizer() const {
	return ui->useSparseOptimizerCheckBox->isChecked();
}

int SparseSolverConfigDialog::numberOfSteps() const {
	return ui->nStepSpinBoxpinBox->value();
}

} //namespace StereoVisionApp
