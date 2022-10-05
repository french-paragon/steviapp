#ifndef CAMERACALIBRATIONOPTIONSDIALOG_H
#define CAMERACALIBRATIONOPTIONSDIALOG_H

#include <QDialog>

namespace StereoVisionApp {

namespace Ui {
class CameraCalibrationOptionsDialog;
}

class CameraCalibrationOptionsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit CameraCalibrationOptionsDialog(QWidget *parent = nullptr);
	~CameraCalibrationOptionsDialog();

	float checkboardCornerSize() const;
	void setCheckboardCornerSize(float width);

	int nIterations() const;
	void setNIterations(int nIter);

private:
	Ui::CameraCalibrationOptionsDialog *ui;
};

} // namespace StereoVisionApp

#endif // CAMERACALIBRATIONOPTIONSDIALOG_H
