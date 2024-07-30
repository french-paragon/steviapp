#include "trajectoryeditgyrooptionsdialog.h"

#include "datablocks/trajectory.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QDialogButtonBox>

#include <QMetaEnum>

namespace StereoVisionApp {

TrajectoryEditGyroOptionsDialog::TrajectoryEditGyroOptionsDialog(QWidget *parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* formLayout = new QFormLayout();

    _gyroFileLine = new QLineEdit(this);
    formLayout->addRow(tr("Gyro file: "), _gyroFileLine);

    _angleRepresentationBox = new QComboBox(this);
    QMetaEnum angleRepConv = QMetaEnum::fromType<Trajectory::AngleRepresentation>();

    for (int i = 0; i < angleRepConv.keyCount(); i++) {
        const char* key = angleRepConv.key(i);
        QString name = key;
        int val = angleRepConv.keyToValue(key);

        _angleRepresentationBox->addItem(name, val);
    }

    formLayout->addRow(tr("Angle representation: "), _angleRepresentationBox);

    _angleUnitBox = new QComboBox(this);
    QMetaEnum angleUnitConv = QMetaEnum::fromType<Trajectory::AngleUnits>();

    for (int i = 0; i < angleUnitConv.keyCount(); i++) {
        const char* key = angleUnitConv.key(i);
        QString name = key;
        int val = angleUnitConv.keyToValue(key);

        _angleUnitBox->addItem(name, val);
    }
    formLayout->addRow(tr("Angle unit: "), _angleUnitBox);

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
    _wColSpinBox = new QSpinBox(this);

    _timeColSpinBox->setMinimum(0);
    _xColSpinBox->setMinimum(0);
    _yColSpinBox->setMinimum(0);
    _zColSpinBox->setMinimum(0);
    _wColSpinBox->setMinimum(0);

    _timeColSpinBox->setMaximum(999);
    _xColSpinBox->setMaximum(999);
    _yColSpinBox->setMaximum(999);
    _zColSpinBox->setMaximum(999);
    _wColSpinBox->setMaximum(999);

    formLayout->addRow(tr("Time scale: "), _timeScaleSpinBox);
    formLayout->addRow(tr("Time delta: "), _timeDeltaSpinBox);

    formLayout->addRow(tr("Time column: "), _timeColSpinBox);
    formLayout->addRow(tr("x column: "), _xColSpinBox);
    formLayout->addRow(tr("y column: "), _yColSpinBox);
    formLayout->addRow(tr("z column: "), _zColSpinBox);
    formLayout->addRow(tr("w column: "), _wColSpinBox);

    _xPositiveBox = new QCheckBox(this);
    _yPositiveBox = new QCheckBox(this);
    _zPositiveBox = new QCheckBox(this);
    _wPositiveBox = new QCheckBox(this);

    formLayout->addRow(tr("x angle positive: "), _xPositiveBox);
    formLayout->addRow(tr("y angle positive: "), _yPositiveBox);
    formLayout->addRow(tr("z angle positive: "), _zPositiveBox);
    formLayout->addRow(tr("w angle positive: "), _wPositiveBox);

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

void TrajectoryEditGyroOptionsDialog::ConfigureTrajectoryGyroOptions(Trajectory* traj, QWidget* parent) {

    if (traj == nullptr) {
        return;
    }

    TrajectoryEditGyroOptionsDialog dialog(parent);

    dialog._gyroFileLine->setText(traj->gyroFile());

    int idxRep = dialog._angleRepresentationBox->findData(traj->gyroAngleRepresentation());
    dialog._angleRepresentationBox->setCurrentIndex(idxRep);
    int idxUnit = dialog._angleUnitBox->findData(traj->gyroAngleUnits());
    dialog._angleUnitBox->setCurrentIndex(idxUnit);

    dialog._timeScaleSpinBox->setValue(traj->gyroTimeScale());
    dialog._timeDeltaSpinBox->setValue(traj->gyroTimeDelta());

    dialog._timeColSpinBox->setValue(traj->gyroColumns(Trajectory::Axis::T));
    dialog._xColSpinBox->setValue(traj->gyroColumns(Trajectory::Axis::X));
    dialog._yColSpinBox->setValue(traj->gyroColumns(Trajectory::Axis::Y));
    dialog._zColSpinBox->setValue(traj->gyroColumns(Trajectory::Axis::Z));
    dialog._wColSpinBox->setValue(traj->gyroColumns(Trajectory::Axis::W));

    dialog._xPositiveBox->setChecked(traj->gyroSign(Trajectory::Axis::X) > 0);
    dialog._yPositiveBox->setChecked(traj->gyroSign(Trajectory::Axis::Y) > 0);
    dialog._zPositiveBox->setChecked(traj->gyroSign(Trajectory::Axis::Z) > 0);
    dialog._wPositiveBox->setChecked(traj->gyroSign(Trajectory::Axis::W) > 0);


    int status = dialog.exec();

    if (status == QDialogButtonBox::RejectRole) {
        return;
    }

    if (status == QDialogButtonBox::ResetRole) {
        traj->setGyroFile("");

        traj->setGyroAngleRepresentation(Trajectory::EulerXYZ);
        traj->setGyroAngleUnits(Trajectory::Degree);

        traj->setGyroTimeScale(1);
        traj->setGyroTimeDelta(0);

        traj->setGyroColumn(Trajectory::Axis::T, 0);
        traj->setGyroColumn(Trajectory::Axis::X, 1);
        traj->setGyroColumn(Trajectory::Axis::Y, 2);
        traj->setGyroColumn(Trajectory::Axis::Z, 3);
        traj->setGyroColumn(Trajectory::Axis::W, 4);

        traj->setGyroSign(Trajectory::Axis::X, 1);
        traj->setGyroSign(Trajectory::Axis::Y, 1);
        traj->setGyroSign(Trajectory::Axis::Z, 1);
        traj->setGyroSign(Trajectory::Axis::W, 1);
    }

    if (status == QDialogButtonBox::ApplyRole) {
        traj->setGyroFile(dialog._gyroFileLine->text());

        traj->setGyroAngleRepresentation(dialog._angleRepresentationBox->currentData().toInt());
        traj->setGyroAngleUnits(dialog._angleUnitBox->currentData().toInt());

        traj->setGyroTimeScale(dialog._timeScaleSpinBox->value());
        traj->setGyroTimeDelta(dialog._timeDeltaSpinBox->value());

        traj->setGyroColumn(Trajectory::Axis::T, dialog._timeColSpinBox->value());
        traj->setGyroColumn(Trajectory::Axis::X, dialog._xColSpinBox->value());
        traj->setGyroColumn(Trajectory::Axis::Y, dialog._yColSpinBox->value());
        traj->setGyroColumn(Trajectory::Axis::Z, dialog._zColSpinBox->value());
        traj->setGyroColumn(Trajectory::Axis::W, dialog._wColSpinBox->value());

        traj->setGyroSign(Trajectory::Axis::X, (dialog._xPositiveBox->isChecked()) ? 1 : -1);
        traj->setGyroSign(Trajectory::Axis::Y, (dialog._yPositiveBox->isChecked()) ? 1 : -1);
        traj->setGyroSign(Trajectory::Axis::Z, (dialog._zPositiveBox->isChecked()) ? 1 : -1);
        traj->setGyroSign(Trajectory::Axis::W, (dialog._wPositiveBox->isChecked()) ? 1 : -1);
    }

}

} // namespace StereoVisionApp
