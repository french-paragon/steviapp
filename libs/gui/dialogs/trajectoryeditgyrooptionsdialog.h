#ifndef STEREOVISIONAPP_TRAJECTORYEDITGYROOPTIONSDIALOG_H
#define STEREOVISIONAPP_TRAJECTORYEDITGYROOPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>

class QLineEdit;
class QSpinBox;
class QComboBox;
class QCheckBox;
class QDoubleSpinBox;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryEditGyroOptionsDialog : public QDialog
{
public:
    TrajectoryEditGyroOptionsDialog(QWidget* parent = nullptr);

    /*!
     * \brief ConfigureTrajectoryPositionOptions configure the options for the positions for a trajectory
     * \param traj the trajectory to configure
     * \param parent the parent for the dialog
     */
    static void ConfigureTrajectoryGyroOptions(Trajectory* traj, QWidget* parent);

protected:

    QLineEdit* _gyroFileLine;

    QComboBox* _angleRepresentationBox;
    QComboBox* _angleUnitBox;

    QDoubleSpinBox* _timeScaleSpinBox;
    QDoubleSpinBox* _timeDeltaSpinBox;

    QSpinBox* _timeColSpinBox;
    QSpinBox* _xColSpinBox;
    QSpinBox* _yColSpinBox;
    QSpinBox* _zColSpinBox;
    QSpinBox* _wColSpinBox;

    QCheckBox* _xPositiveBox;
    QCheckBox* _yPositiveBox;
    QCheckBox* _zPositiveBox;
    QCheckBox* _wPositiveBox;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYEDITGYROOPTIONSDIALOG_H
