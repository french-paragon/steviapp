#ifndef STEREOVISIONAPP_TRAJECTORYEDITACCELEROMETEROPTIONSDIALOG_H
#define STEREOVISIONAPP_TRAJECTORYEDITACCELEROMETEROPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>

class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryEditAccelerometerOptionsDialog : public QDialog
{
    Q_OBJECT
public:
    TrajectoryEditAccelerometerOptionsDialog(QWidget* parent = nullptr);

    /*!
     * \brief ConfigureTrajectoryAccelerometerOptions configure the options for the accelerometer for a trajectory
     * \param traj the trajectory to configure
     * \param parent the parent for the dialog
     */
    static void ConfigureTrajectoryAccelerometerOptions(Trajectory* traj, QWidget* parent);

protected:

    QLineEdit* _accFileLine;

    QDoubleSpinBox* _timeScaleSpinBox;
    QDoubleSpinBox* _timeDeltaSpinBox;

    QSpinBox* _timeColSpinBox;
    QSpinBox* _xColSpinBox;
    QSpinBox* _yColSpinBox;
    QSpinBox* _zColSpinBox;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYEDITACCELEROMETEROPTIONSDIALOG_H
