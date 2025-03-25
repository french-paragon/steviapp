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

void SparseSolverConfigDialog::enableComputeUncertaintyOption(bool enable) {
	ui->predictUncertaintyCheckBox->setVisible(enable);
}

void SparseSolverConfigDialog::enableUseCurrentSolutionOption(bool enable) {
	ui->initWithCurrentSolCheckBox->setVisible(enable);
}

bool SparseSolverConfigDialog::shouldRun() const
{
	return _shouldRun;
}

bool SparseSolverConfigDialog::computeUncertainty() const {
	return ui->predictUncertaintyCheckBox->isChecked() and useSparseOptimizer();
}
bool SparseSolverConfigDialog::useSparseOptimizer() const {
	return ui->useSparseOptimizerCheckBox->isChecked();
}
bool SparseSolverConfigDialog::initWithCurrentSol() const {
	return ui->initWithCurrentSolCheckBox->isChecked();
}

int SparseSolverConfigDialog::numberOfSteps() const {
	return ui->nStepSpinBoxpinBox->value();
}

void SparseSolverConfigDialog::setComputeUncertainty(bool compute) {
    return ui->predictUncertaintyCheckBox->setChecked(compute);
}
void SparseSolverConfigDialog::setUseSparseOptimizer(bool useSparse) {
    return ui->useSparseOptimizerCheckBox->setChecked(useSparse);
}

void SparseSolverConfigDialog::setNumberOfSteps(int nSteps) {
	ui->nStepSpinBoxpinBox->setValue(nSteps);
}

FixedParameters SparseSolverConfigDialog::getFixedParameters() const {

	return ui->fixedParametersWidget->getFixedParameters();
}

void SparseSolverConfigDialog::setFixedParameters(FixedParameters parameters) {

	ui->fixedParametersWidget->setFixedParameters(parameters);
}

} //namespace StereoVisionApp
