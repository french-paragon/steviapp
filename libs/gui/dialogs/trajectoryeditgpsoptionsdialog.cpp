#include "trajectoryeditgpsoptionsdialog.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QMetaEnum>
#include <QDialogButtonBox>
#include <QLabel>
#include <QAbstractButton>

#include "datablocks/trajectory.h"

namespace StereoVisionApp {

TrajectoryEditGpsOptionsDialog::TrajectoryEditGpsOptionsDialog(QWidget* parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* formLayout = new QFormLayout();

    _gpsFileLine = new QLineEdit(this);
    formLayout->addRow(tr("GPS Data file: "), _gpsFileLine);

    _epsgLine = new QLineEdit(this);
    formLayout->addRow(tr("GPS EPSG: "), _epsgLine);

    _topocentricConventionBox = new QComboBox(this);
    QMetaEnum topoConv = QMetaEnum::fromType<Trajectory::TopocentricConvention>();

    for (int i = 0; i < topoConv.keyCount(); i++) {
        const char* key = topoConv.key(i);
        QString name = key;
        int val = topoConv.keyToValue(key);

        _topocentricConventionBox->addItem(name, val);
    }

    formLayout->addRow(tr("Topocentric convention: "), _topocentricConventionBox);

    _timeScaleSpinBox = new QDoubleSpinBox(this);
    _timeDeltaSpinBox = new QDoubleSpinBox(this);

    _timeScaleSpinBox->setMinimum(0);
    _timeScaleSpinBox->setMaximum(999999999999);
    _timeScaleSpinBox->setDecimals(6);

    _timeDeltaSpinBox->setMinimum(-999999999999);
    _timeDeltaSpinBox->setMaximum(999999999999);

    auto configureColSpinBox = [this] () -> QSpinBox* {
            QSpinBox* ret = new QSpinBox(this);
            ret->setMinimum(-1);
            ret->setMaximum(999);
            ret->setValue(-1);
            return ret;
    };

    _timeColSpinBox = configureColSpinBox();
    _posXColSpinBox = configureColSpinBox();
    _posYColSpinBox = configureColSpinBox();
    _posZColSpinBox = configureColSpinBox();
    _speedXColSpinBox = configureColSpinBox();
    _speedYColSpinBox = configureColSpinBox();
    _speedZColSpinBox = configureColSpinBox();
    _varXColSpinBox = configureColSpinBox();
    _varYColSpinBox = configureColSpinBox();
    _varZColSpinBox = configureColSpinBox();
    _covXYColSpinBox = configureColSpinBox();
    _covYZColSpinBox = configureColSpinBox();
    _covZXColSpinBox = configureColSpinBox();
    _speedVarXColSpinBox = configureColSpinBox();
    _speedVarYColSpinBox = configureColSpinBox();
    _speedVarZColSpinBox = configureColSpinBox();
    _speedCovXYColSpinBox = configureColSpinBox();
    _speedCovYZColSpinBox = configureColSpinBox();
    _speedCovZXColSpinBox = configureColSpinBox();

    formLayout->addRow(tr("Time scale: "), _timeScaleSpinBox);
    formLayout->addRow(tr("Time delta: "), _timeDeltaSpinBox);

    formLayout->addRow(tr("Time column: "), _timeColSpinBox);

    QHBoxLayout* posLayout = new QHBoxLayout();
    posLayout->addWidget(new QLabel("x", this));
    posLayout->addWidget(_posXColSpinBox);
    posLayout->addWidget(new QLabel("y", this));
    posLayout->addWidget(_posYColSpinBox);
    posLayout->addWidget(new QLabel("z", this));
    posLayout->addWidget(_posZColSpinBox);
    formLayout->addRow(tr("pos columns: "), posLayout);

    QHBoxLayout* speedLayout = new QHBoxLayout();
    speedLayout->addWidget(new QLabel("x", this));
    speedLayout->addWidget(_speedXColSpinBox);
    speedLayout->addWidget(new QLabel("y", this));
    speedLayout->addWidget(_speedYColSpinBox);
    speedLayout->addWidget(new QLabel("z", this));
    speedLayout->addWidget(_speedZColSpinBox);
    formLayout->addRow(tr("speed columns: "), speedLayout);

    QHBoxLayout* varLayout = new QHBoxLayout();
    varLayout->addWidget(new QLabel("x", this));
    varLayout->addWidget(_varXColSpinBox);
    varLayout->addWidget(new QLabel("y", this));
    varLayout->addWidget(_varYColSpinBox);
    varLayout->addWidget(new QLabel("z", this));
    varLayout->addWidget(_varZColSpinBox);
    formLayout->addRow(tr("variance columns: "), varLayout);

    QHBoxLayout* covLayout = new QHBoxLayout();
    covLayout->addWidget(new QLabel("xy", this));
    covLayout->addWidget(_covXYColSpinBox);
    covLayout->addWidget(new QLabel("yz", this));
    covLayout->addWidget(_covYZColSpinBox);
    covLayout->addWidget(new QLabel("zx", this));
    covLayout->addWidget(_covZXColSpinBox);
    formLayout->addRow(tr("covariance columns: "), covLayout);

    QHBoxLayout* speedVarLayout = new QHBoxLayout();
    speedVarLayout->addWidget(new QLabel("x", this));
    speedVarLayout->addWidget(_speedVarXColSpinBox);
    speedVarLayout->addWidget(new QLabel("y", this));
    speedVarLayout->addWidget(_speedVarYColSpinBox);
    speedVarLayout->addWidget(new QLabel("z", this));
    speedVarLayout->addWidget(_speedVarZColSpinBox);
    formLayout->addRow(tr("speed variance columns: "), speedVarLayout);

    QHBoxLayout* speecCovLayout = new QHBoxLayout();
    speecCovLayout->addWidget(new QLabel("xy", this));
    speecCovLayout->addWidget(_speedCovXYColSpinBox);
    speecCovLayout->addWidget(new QLabel("yz", this));
    speecCovLayout->addWidget(_speedCovYZColSpinBox);
    speecCovLayout->addWidget(new QLabel("zx", this));
    speecCovLayout->addWidget(_speedCovZXColSpinBox);
    formLayout->addRow(tr("speed covariance columns: "), speecCovLayout);

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

void TrajectoryEditGpsOptionsDialog::ConfigureTrajectoryGpsOptions(Trajectory* traj, QWidget* parent) {

    if (traj == nullptr) {
        return;
    }

    TrajectoryEditGpsOptionsDialog dialog(parent);

    dialog._gpsFileLine->setText(traj->gpsFile());

    dialog._epsgLine->setText(traj->gpsEpsg());

    int idxTopo = dialog._topocentricConventionBox->findData(traj->gpsTopocentricConvention());
    dialog._topocentricConventionBox->setCurrentIndex(idxTopo);

    dialog._timeScaleSpinBox->setValue(traj->gpsTimeScale());
    dialog._timeDeltaSpinBox->setValue(traj->gpsTimeDelta());

    dialog._timeColSpinBox->setValue(traj->gpsPosColumns(Trajectory::Axis::T));

    dialog._posXColSpinBox->setValue(traj->gpsPosColumns(Trajectory::Axis::X));
    dialog._posYColSpinBox->setValue(traj->gpsPosColumns(Trajectory::Axis::Y));
    dialog._posZColSpinBox->setValue(traj->gpsPosColumns(Trajectory::Axis::Z));

    dialog._speedXColSpinBox->setValue(traj->gpsSpeedColumns(Trajectory::Axis::X));
    dialog._speedYColSpinBox->setValue(traj->gpsSpeedColumns(Trajectory::Axis::Y));
    dialog._speedZColSpinBox->setValue(traj->gpsSpeedColumns(Trajectory::Axis::Z));

    dialog._varXColSpinBox->setValue(traj->gpsVarColumns(Trajectory::Axis::X));
    dialog._varYColSpinBox->setValue(traj->gpsVarColumns(Trajectory::Axis::Y));
    dialog._varZColSpinBox->setValue(traj->gpsVarColumns(Trajectory::Axis::Z));

    dialog._covXYColSpinBox->setValue(traj->gpsCovColumns(Trajectory::Axis::X, Trajectory::Axis::Y));
    dialog._covYZColSpinBox->setValue(traj->gpsCovColumns(Trajectory::Axis::Y, Trajectory::Axis::Z));
    dialog._covZXColSpinBox->setValue(traj->gpsCovColumns(Trajectory::Axis::Z, Trajectory::Axis::X));

    dialog._speedVarXColSpinBox->setValue(traj->gpsSpeedVarColumns(Trajectory::Axis::X));
    dialog._speedVarYColSpinBox->setValue(traj->gpsSpeedVarColumns(Trajectory::Axis::Y));
    dialog._speedVarZColSpinBox->setValue(traj->gpsSpeedVarColumns(Trajectory::Axis::Z));

    dialog._speedCovXYColSpinBox->setValue(traj->gpsSpeedCovColumns(Trajectory::Axis::X, Trajectory::Axis::Y));
    dialog._speedCovYZColSpinBox->setValue(traj->gpsSpeedCovColumns(Trajectory::Axis::Y, Trajectory::Axis::Z));
    dialog._speedCovZXColSpinBox->setValue(traj->gpsSpeedCovColumns(Trajectory::Axis::Z, Trajectory::Axis::X));

    int status = dialog.exec();

    if (status == QDialogButtonBox::RejectRole) {
        return;
    }

    if (status == QDialogButtonBox::ResetRole) {
        traj->setGpsFile("");

        traj->setGpsEpsg("");
        traj->setGpsTopocentricConvention(Trajectory::NED);

        traj->setGpsTimeScale(1);
        traj->setGpsTimeDelta(0);

        traj->setGpsPosColumn(Trajectory::Axis::T, 0);

        traj->setGpsPosColumn(Trajectory::Axis::X, 1);
        traj->setGpsPosColumn(Trajectory::Axis::Y, 2);
        traj->setGpsPosColumn(Trajectory::Axis::Z, 3);

        traj->setGpsSpeedColumn(Trajectory::Axis::X, -1);
        traj->setGpsSpeedColumn(Trajectory::Axis::Y, -1);
        traj->setGpsSpeedColumn(Trajectory::Axis::Z, -1);

        traj->setGpsPosColumn(Trajectory::Axis::X, -1);
        traj->setGpsPosColumn(Trajectory::Axis::Y, -1);
        traj->setGpsPosColumn(Trajectory::Axis::Z, -1);

        traj->setGpsVarColumn(Trajectory::Axis::X, -1);
        traj->setGpsVarColumn(Trajectory::Axis::Y, -1);
        traj->setGpsVarColumn(Trajectory::Axis::Z, -1);

        traj->setGpsCovColumn(Trajectory::Axis::X, Trajectory::Axis::Y, -1);
        traj->setGpsCovColumn(Trajectory::Axis::Y, Trajectory::Axis::Z, -1);
        traj->setGpsCovColumn(Trajectory::Axis::Z, Trajectory::Axis::X, -1);

        traj->setGpsSpeedVarColumn(Trajectory::Axis::X, -1);
        traj->setGpsSpeedVarColumn(Trajectory::Axis::Y, -1);
        traj->setGpsSpeedVarColumn(Trajectory::Axis::Z, -1);

        traj->setGpsSpeedCovColumn(Trajectory::Axis::X, Trajectory::Axis::Y, -1);
        traj->setGpsSpeedCovColumn(Trajectory::Axis::Y, Trajectory::Axis::Z, -1);
        traj->setGpsSpeedCovColumn(Trajectory::Axis::Z, Trajectory::Axis::X, -1);
    }

    if (status == QDialogButtonBox::ApplyRole) {


        traj->setGpsFile(dialog._gpsFileLine->text());

        traj->setGpsEpsg(dialog._epsgLine->text());
        traj->setGpsTopocentricConvention(
            static_cast<Trajectory::TopocentricConvention>(
                dialog._topocentricConventionBox->currentData().toInt()));

        traj->setGpsTimeScale(dialog._timeScaleSpinBox->value());
        traj->setGpsTimeDelta(dialog._timeDeltaSpinBox->value());

        traj->setGpsPosColumn(Trajectory::Axis::T, dialog._timeColSpinBox->value());

        traj->setGpsPosColumn(Trajectory::Axis::X, dialog._posXColSpinBox->value());
        traj->setGpsPosColumn(Trajectory::Axis::Y, dialog._posYColSpinBox->value());
        traj->setGpsPosColumn(Trajectory::Axis::Z, dialog._posZColSpinBox->value());

        traj->setGpsSpeedColumn(Trajectory::Axis::X, dialog._speedXColSpinBox->value());
        traj->setGpsSpeedColumn(Trajectory::Axis::Y, dialog._speedYColSpinBox->value());
        traj->setGpsSpeedColumn(Trajectory::Axis::Z, dialog._speedZColSpinBox->value());

        traj->setGpsVarColumn(Trajectory::Axis::X, dialog._varXColSpinBox->value());
        traj->setGpsVarColumn(Trajectory::Axis::Y, dialog._varYColSpinBox->value());
        traj->setGpsVarColumn(Trajectory::Axis::Z, dialog._varZColSpinBox->value());

        traj->setGpsCovColumn(Trajectory::Axis::X, Trajectory::Axis::Y, dialog._covXYColSpinBox->value());
        traj->setGpsCovColumn(Trajectory::Axis::Y, Trajectory::Axis::Z, dialog._covYZColSpinBox->value());
        traj->setGpsCovColumn(Trajectory::Axis::Z, Trajectory::Axis::X, dialog._covZXColSpinBox->value());

        traj->setGpsSpeedVarColumn(Trajectory::Axis::X, dialog._speedVarXColSpinBox->value());
        traj->setGpsSpeedVarColumn(Trajectory::Axis::Y, dialog._speedVarYColSpinBox->value());
        traj->setGpsSpeedVarColumn(Trajectory::Axis::Z, dialog._speedVarZColSpinBox->value());

        traj->setGpsSpeedCovColumn(Trajectory::Axis::X, Trajectory::Axis::Y, -1);
        traj->setGpsSpeedCovColumn(Trajectory::Axis::Y, Trajectory::Axis::Z, -1);
        traj->setGpsSpeedCovColumn(Trajectory::Axis::Z, Trajectory::Axis::X, -1);
    }

}

} // namespace StereoVisionApp
