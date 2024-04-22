#ifndef STEREOVISIONAPP_TRAJECTORYEDITORIENTATIONOPTIONSDIALOG_H
#define STEREOVISIONAPP_TRAJECTORYEDITORIENTATIONOPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>

class QLineEdit;
class QSpinBox;
class QComboBox;
class QCheckBox;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryEditOrientationOptionsDialog : public QDialog
{
    Q_OBJECT
public:
    TrajectoryEditOrientationOptionsDialog(QWidget* parent = nullptr);

    /*!
     * \brief ConfigureTrajectoryPositionOptions configure the options for the positions for a trajectory
     * \param traj the trajectory to configure
     * \param parent the parent for the dialog
     */
    static void ConfigureTrajectoryOrientationOptions(Trajectory* traj, QWidget* parent);

protected:

    QLineEdit* _orientationFileLine;

    QComboBox* _topocentricConventionBox;
    QComboBox* _angleRepresentationBox;
    QComboBox* _angleUnitBox;

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

#endif // STEREOVISIONAPP_TRAJECTORYEDITORIENTATIONOPTIONSDIALOG_H
