#ifndef STEREOVISIONAPP_TRAJECTORYEDITPOSITIONOPTIONSDIALOG_H
#define STEREOVISIONAPP_TRAJECTORYEDITPOSITIONOPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>

class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryEditPositionOptionsDialog : public QDialog
{
    Q_OBJECT
public:
    TrajectoryEditPositionOptionsDialog(QWidget* parent = nullptr);

    /*!
     * \brief ConfigureTrajectoryPositionOptions configure the options for the positions for a trajectory
     * \param traj the trajectory to configure
     * \param parent the parent for the dialog
     */
    static void ConfigureTrajectoryPositionOptions(Trajectory* traj, QWidget* parent);

protected:

    QLineEdit* _posFileLine;

    QLineEdit* _epsgLine;

    QDoubleSpinBox* _timeScaleSpinBox;
    QDoubleSpinBox* _timeDeltaSpinBox;

    QSpinBox* _timeColSpinBox;
    QSpinBox* _xColSpinBox;
    QSpinBox* _yColSpinBox;
    QSpinBox* _zColSpinBox;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYEDITPOSITIONOPTIONSDIALOG_H
