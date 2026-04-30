#include "sparsesolverconfigdialog.h"
#include "ui_sparsesolverconfigdialog.h"

#include <cmath>
#include <QFileInfo>
#include <QAction>
#include <QFileDialog>

namespace StereoVisionApp {

SparseSolverConfigDialog::SparseSolverConfigDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::SparseSolverConfigDialog),
	_shouldRun(false)
{
	ui->setupUi(this);

	connect(ui->RunButton, &QPushButton::clicked, this, &SparseSolverConfigDialog::onRunButtonPressed);
	connect(ui->cancelButton, &QPushButton::clicked, this, &SparseSolverConfigDialog::onCancelButtonPressed);

    QAction* clearLineAction = new QAction(QIcon::fromTheme("edit-clear"), tr("clear"), ui->logsPrefixLineEdit);
    QAction* openFolderAction = new QAction(QIcon::fromTheme("folder-open"), tr("open"), ui->logsPrefixLineEdit);

    ui->logsDirectoryLineEdit->addAction(openFolderAction, QLineEdit::ActionPosition::TrailingPosition);
    ui->logsDirectoryLineEdit->addAction(clearLineAction, QLineEdit::ActionPosition::TrailingPosition);

    connect(clearLineAction, &QAction::triggered, this, [this] () {
        setLogsDirectory("");
    });

    connect(openFolderAction, &QAction::triggered, this, [this] () {
        QString folderPath = QFileDialog::getExistingDirectory(this, tr("get logs directory"), logsDirectory());
        if (!folderPath.isEmpty()) {
            setLogsDirectory(folderPath);
        }
    });
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
bool SparseSolverConfigDialog::useRobustCameras() const {
    return ui->useRobustCameraCheckBox->isChecked();
}

int SparseSolverConfigDialog::numberOfSteps() const {
	return ui->nStepSpinBoxpinBox->value();
}

double SparseSolverConfigDialog::functionTolerance() const {
    return std::pow(10,double(ui->funcTolBox->value()));
}
double SparseSolverConfigDialog::parametersTolerance() const {
    return std::pow(10,double(ui->paramTolBox->value()));
}

void SparseSolverConfigDialog::setComputeUncertainty(bool compute) {
    return ui->predictUncertaintyCheckBox->setChecked(compute);
}
void SparseSolverConfigDialog::setUseSparseOptimizer(bool useSparse) {
    return ui->useSparseOptimizerCheckBox->setChecked(useSparse);
}
void SparseSolverConfigDialog::setUseRobustCameras(bool useRobust) {
    return ui->useRobustCameraCheckBox->setChecked(useRobust);
}

void SparseSolverConfigDialog::setNumberOfSteps(int nSteps) {
	ui->nStepSpinBoxpinBox->setValue(nSteps);
}

QString SparseSolverConfigDialog::logsDirectory() const {
    return ui->logsDirectoryLineEdit->text();
}
void SparseSolverConfigDialog::setLogsDirectory(QString const& directory) {

    if (directory.isEmpty()) {
        ui->logsDirectoryLineEdit->setText(directory);
        return;
    }

    QFileInfo infos(directory);

    if (!infos.exists() or !infos.isDir()) {
        return;
    }

    ui->logsDirectoryLineEdit->setText(directory);
}

QString SparseSolverConfigDialog::logsPrefix() const {
    return ui->logsPrefixLineEdit->text();
}
void SparseSolverConfigDialog::setLogsPrefix(QString const& prefix) {
    ui->logsPrefixLineEdit->setText(prefix);
}

FixedParameters SparseSolverConfigDialog::getFixedParameters() const {

	return ui->fixedParametersWidget->getFixedParameters();
}

void SparseSolverConfigDialog::setFixedParameters(FixedParameters parameters) {

	ui->fixedParametersWidget->setFixedParameters(parameters);
}

} //namespace StereoVisionApp
