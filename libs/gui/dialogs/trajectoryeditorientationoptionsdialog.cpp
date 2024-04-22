#include "trajectoryeditorientationoptionsdialog.h"

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

TrajectoryEditOrientationOptionsDialog::TrajectoryEditOrientationOptionsDialog(QWidget *parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* formLayout = new QFormLayout();

    _orientationFileLine = new QLineEdit(this);
    formLayout->addRow(tr("Orientation file: "), _orientationFileLine);

    _topocentricConventionBox = new QComboBox(this);
    QMetaEnum topoConv = QMetaEnum::fromType<Trajectory::TopocentricConvention>();

    for (int i = 0; i < topoConv.keyCount(); i++) {
        const char* key = topoConv.key(i);
        QString name = key;
        int val = topoConv.keyToValue(key);

        _topocentricConventionBox->addItem(name, val);
    }

    formLayout->addRow(tr("Topocentric convention: "), _topocentricConventionBox);

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

void TrajectoryEditOrientationOptionsDialog::ConfigureTrajectoryOrientationOptions(Trajectory* traj, QWidget* parent) {

    if (traj == nullptr) {
        return;
    }

    TrajectoryEditOrientationOptionsDialog dialog(parent);

    dialog._orientationFileLine->setText(traj->orientationFile());

    int idxTopo = dialog._topocentricConventionBox->findData(traj->orientationTopocentricConvention());
    dialog._topocentricConventionBox->setCurrentIndex(idxTopo);
    int idxRep = dialog._angleRepresentationBox->findData(traj->orientationAngleRepresentation());
    dialog._angleRepresentationBox->setCurrentIndex(idxRep);
    int idxUnit = dialog._angleUnitBox->findData(traj->orientationAngleUnits());
    dialog._angleUnitBox->setCurrentIndex(idxUnit);

    dialog._timeColSpinBox->setValue(traj->orientationColumns(Trajectory::Axis::T));
    dialog._xColSpinBox->setValue(traj->orientationColumns(Trajectory::Axis::X));
    dialog._yColSpinBox->setValue(traj->orientationColumns(Trajectory::Axis::Y));
    dialog._zColSpinBox->setValue(traj->orientationColumns(Trajectory::Axis::Z));
    dialog._wColSpinBox->setValue(traj->orientationColumns(Trajectory::Axis::W));

    dialog._xPositiveBox->setChecked(traj->orientationSign(Trajectory::Axis::X) > 0);
    dialog._yPositiveBox->setChecked(traj->orientationSign(Trajectory::Axis::Y) > 0);
    dialog._zPositiveBox->setChecked(traj->orientationSign(Trajectory::Axis::Z) > 0);
    dialog._wPositiveBox->setChecked(traj->orientationSign(Trajectory::Axis::W) > 0);


    int status = dialog.exec();

    if (status == QDialogButtonBox::RejectRole) {
        return;
    }

    if (status == QDialogButtonBox::ResetRole) {
        traj->setOrientationFile("");

        traj->setOrientationTopocentricConvention(Trajectory::NED);
        traj->setOrientationAngleRepresentation(Trajectory::EulerXYZ);
        traj->setOrientationAngleUnits(Trajectory::Degree);

        traj->setOrientationColumn(Trajectory::Axis::T, 0);
        traj->setOrientationColumn(Trajectory::Axis::X, 1);
        traj->setOrientationColumn(Trajectory::Axis::Y, 2);
        traj->setOrientationColumn(Trajectory::Axis::Z, 3);
        traj->setOrientationColumn(Trajectory::Axis::W, 4);

        traj->setOrientationSign(Trajectory::Axis::X, 1);
        traj->setOrientationSign(Trajectory::Axis::Y, 1);
        traj->setOrientationSign(Trajectory::Axis::Z, 1);
        traj->setOrientationSign(Trajectory::Axis::W, 1);
    }

    if (status == QDialogButtonBox::ApplyRole) {
        traj->setOrientationFile(dialog._orientationFileLine->text());

        traj->setOrientationTopocentricConvention(dialog._topocentricConventionBox->currentData().toInt());
        traj->setOrientationAngleRepresentation(dialog._angleRepresentationBox->currentData().toInt());
        traj->setOrientationAngleUnits(dialog._angleUnitBox->currentData().toInt());

        traj->setOrientationColumn(Trajectory::Axis::T, dialog._timeColSpinBox->value());
        traj->setOrientationColumn(Trajectory::Axis::X, dialog._xColSpinBox->value());
        traj->setOrientationColumn(Trajectory::Axis::Y, dialog._yColSpinBox->value());
        traj->setOrientationColumn(Trajectory::Axis::Z, dialog._zColSpinBox->value());
        traj->setOrientationColumn(Trajectory::Axis::W, dialog._wColSpinBox->value());

        traj->setOrientationSign(Trajectory::Axis::X, (dialog._xPositiveBox->isChecked()) ? 1 : -1);
        traj->setOrientationSign(Trajectory::Axis::Y, (dialog._yPositiveBox->isChecked()) ? 1 : -1);
        traj->setOrientationSign(Trajectory::Axis::Z, (dialog._zPositiveBox->isChecked()) ? 1 : -1);
        traj->setOrientationSign(Trajectory::Axis::W, (dialog._wPositiveBox->isChecked()) ? 1 : -1);
    }

}

} // namespace StereoVisionApp
