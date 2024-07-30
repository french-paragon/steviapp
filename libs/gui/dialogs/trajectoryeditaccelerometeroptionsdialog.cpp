#include "trajectoryeditaccelerometeroptionsdialog.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QLineEdit>
#include <QSpinBox>

#include <QDialogButtonBox>
#include <QAbstractButton>

#include "datablocks/trajectory.h"

namespace StereoVisionApp {

TrajectoryEditAccelerometerOptionsDialog::TrajectoryEditAccelerometerOptionsDialog(QWidget *parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* formLayout = new QFormLayout();

    _accFileLine = new QLineEdit(this);
    formLayout->addRow(tr("Accelerometer file: "), _accFileLine);

    _timeScaleSpinBox = new QDoubleSpinBox(this);
    _timeDeltaSpinBox = new QDoubleSpinBox(this);

    _timeScaleSpinBox->setMinimum(0);
    _timeScaleSpinBox->setMaximum(999999999999);
    _timeScaleSpinBox->setDecimals(6);

    _timeDeltaSpinBox->setMinimum(-999999999999);
    _timeDeltaSpinBox->setMaximum(999999999999);

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

    formLayout->addRow(tr("Time scale: "), _timeScaleSpinBox);
    formLayout->addRow(tr("Time delta: "), _timeDeltaSpinBox);

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

void TrajectoryEditAccelerometerOptionsDialog::ConfigureTrajectoryAccelerometerOptions(Trajectory* traj, QWidget* parent) {

    if (traj == nullptr) {
        return;
    }

    TrajectoryEditAccelerometerOptionsDialog dialog(parent);

    dialog._accFileLine->setText(traj->accelerometerFile());

    dialog._timeScaleSpinBox->setValue(traj->accelerometerTimeScale());
    dialog._timeDeltaSpinBox->setValue(traj->accelerometerTimeDelta());

    dialog._timeColSpinBox->setValue(traj->accelerometerColumns(Trajectory::Axis::T));
    dialog._xColSpinBox->setValue(traj->accelerometerColumns(Trajectory::Axis::X));
    dialog._yColSpinBox->setValue(traj->accelerometerColumns(Trajectory::Axis::Y));
    dialog._zColSpinBox->setValue(traj->accelerometerColumns(Trajectory::Axis::Z));

    int status = dialog.exec();

    if (status == QDialogButtonBox::RejectRole) {
        return;
    }

    if (status == QDialogButtonBox::ResetRole) {
        traj->setAccelerometerFile("");

        traj->setAccelerometerTimeScale(1);
        traj->setAccelerometerTimeDelta(0);

        traj->setAccelerometerColumn(Trajectory::Axis::T, 0);
        traj->setAccelerometerColumn(Trajectory::Axis::X, 1);
        traj->setAccelerometerColumn(Trajectory::Axis::Y, 2);
        traj->setAccelerometerColumn(Trajectory::Axis::Z, 3);
    }

    if (status == QDialogButtonBox::ApplyRole) {
        traj->setAccelerometerFile(dialog._accFileLine->text());

        traj->setAccelerometerTimeScale(dialog._timeScaleSpinBox->value());
        traj->setAccelerometerTimeDelta(dialog._timeDeltaSpinBox->value());

        traj->setAccelerometerColumn(Trajectory::Axis::T, dialog._timeColSpinBox->value());
        traj->setAccelerometerColumn(Trajectory::Axis::X, dialog._xColSpinBox->value());
        traj->setAccelerometerColumn(Trajectory::Axis::Y, dialog._yColSpinBox->value());
        traj->setAccelerometerColumn(Trajectory::Axis::Z, dialog._zColSpinBox->value());
    }

}

} // namespace StereoVisionApp
