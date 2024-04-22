#include "trajectoryeditpositionoptionsdialog.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QLineEdit>
#include <QSpinBox>

#include <QDialogButtonBox>
#include <QAbstractButton>

#include "datablocks/trajectory.h"

namespace StereoVisionApp {

TrajectoryEditPositionOptionsDialog::TrajectoryEditPositionOptionsDialog(QWidget *parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* formLayout = new QFormLayout();

    _posFileLine = new QLineEdit(this);
    formLayout->addRow(tr("Position file: "), _posFileLine);

    _epsgLine = new QLineEdit(this);
    formLayout->addRow(tr("Position EPSG: "), _epsgLine);

    _timeColSpinBox = new QSpinBox(this);
    _xColSpinBox = new QSpinBox(this);
    _yColSpinBox = new QSpinBox(this);
    _zColSpinBox = new QSpinBox(this);

    _timeColSpinBox->setMinimum(0);
    _xColSpinBox->setMinimum(0);
    _yColSpinBox->setMinimum(0);
    _zColSpinBox->setMinimum(0);

    _timeColSpinBox->setMaximum(999);
    _xColSpinBox->setMaximum(999);
    _yColSpinBox->setMaximum(999);
    _zColSpinBox->setMaximum(999);

    formLayout->addRow(tr("Time column: "), _timeColSpinBox);
    formLayout->addRow(tr("x column: "), _xColSpinBox);
    formLayout->addRow(tr("y column: "), _yColSpinBox);
    formLayout->addRow(tr("z column: "), _zColSpinBox);

    mainLayout->addLayout(formLayout);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);

    buttonBox->addButton(tr("Apply"), QDialogButtonBox::ApplyRole);
    buttonBox->addButton(tr("Clear"), QDialogButtonBox::ResetRole);
    buttonBox->addButton(tr("Cancel"), QDialogButtonBox::RejectRole);

    mainLayout->addWidget(buttonBox);

    setResult(QDialogButtonBox::RejectRole); //default result is reject

    connect(buttonBox, &QDialogButtonBox::clicked, this, [this, buttonBox] (QAbstractButton *button) {
        QDialogButtonBox::ButtonRole role = buttonBox->buttonRole(button);
        done(role);
    });

}

void TrajectoryEditPositionOptionsDialog::ConfigureTrajectoryPositionOptions(Trajectory* traj, QWidget* parent) {

    if (traj == nullptr) {
        return;
    }

    TrajectoryEditPositionOptionsDialog dialog(parent);

    dialog._posFileLine->setText(traj->positionFile());
    dialog._epsgLine->setText(traj->positionEpsg());

    dialog._timeColSpinBox->setValue(traj->positionColumns(Trajectory::Axis::T));
    dialog._xColSpinBox->setValue(traj->positionColumns(Trajectory::Axis::X));
    dialog._yColSpinBox->setValue(traj->positionColumns(Trajectory::Axis::Y));
    dialog._zColSpinBox->setValue(traj->positionColumns(Trajectory::Axis::Z));

    int status = dialog.exec();

    if (status == QDialogButtonBox::RejectRole) {
        return;
    }

    if (status == QDialogButtonBox::ResetRole) {
        traj->setPositionFile("");
        traj->setPositionEpsg("");

        traj->setPositionColumn(Trajectory::Axis::T, 0);
        traj->setPositionColumn(Trajectory::Axis::X, 1);
        traj->setPositionColumn(Trajectory::Axis::Y, 2);
        traj->setPositionColumn(Trajectory::Axis::Z, 3);
    }

    if (status == QDialogButtonBox::ApplyRole) {
        traj->setPositionFile(dialog._posFileLine->text());
        traj->setPositionEpsg(dialog._epsgLine->text());

        traj->setPositionColumn(Trajectory::Axis::T, dialog._timeColSpinBox->value());
        traj->setPositionColumn(Trajectory::Axis::X, dialog._xColSpinBox->value());
        traj->setPositionColumn(Trajectory::Axis::Y, dialog._yColSpinBox->value());
        traj->setPositionColumn(Trajectory::Axis::Z, dialog._zColSpinBox->value());
    }

}

} // namespace StereoVisionApp
