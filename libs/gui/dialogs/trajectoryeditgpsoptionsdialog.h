#ifndef STEREOVISIONAPP_TRAJECTORYEDITGPSOPTIONSDIALOG_H
#define STEREOVISIONAPP_TRAJECTORYEDITGPSOPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>

class QLineEdit;
class QSpinBox;
class QComboBox;
class QDoubleSpinBox;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryEditGpsOptionsDialog : public QDialog
{
public:
    TrajectoryEditGpsOptionsDialog(QWidget* parent = nullptr);

    /*!
     * \brief ConfigureTrajectoryGpsOptions configure the options for the gps for a trajectory
     * \param traj the trajectory to configure
     * \param parent the parent for the dialog
     */
    static void ConfigureTrajectoryGpsOptions(Trajectory* traj, QWidget* parent);

protected:

    QLineEdit* _gpsFileLine;

    QLineEdit* _epsgLine;
    QComboBox* _topocentricConventionBox;

    QDoubleSpinBox* _timeScaleSpinBox;
    QDoubleSpinBox* _timeDeltaSpinBox;

    QSpinBox* _timeColSpinBox;
    QSpinBox* _posXColSpinBox;
    QSpinBox* _posYColSpinBox;
    QSpinBox* _posZColSpinBox;
    QSpinBox* _speedXColSpinBox;
    QSpinBox* _speedYColSpinBox;
    QSpinBox* _speedZColSpinBox;
    QSpinBox* _varXColSpinBox;
    QSpinBox* _varYColSpinBox;
    QSpinBox* _varZColSpinBox;
    QSpinBox* _covXYColSpinBox;
    QSpinBox* _covYZColSpinBox;
    QSpinBox* _covZXColSpinBox;
    QSpinBox* _speedVarXColSpinBox;
    QSpinBox* _speedVarYColSpinBox;
    QSpinBox* _speedVarZColSpinBox;
    QSpinBox* _speedCovXYColSpinBox;
    QSpinBox* _speedCovYZColSpinBox;
    QSpinBox* _speedCovZXColSpinBox;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYEDITGPSOPTIONSDIALOG_H
