#ifndef HEXAGONALTARGETDETECTIONOPTIONSDIALOG_H
#define HEXAGONALTARGETDETECTIONOPTIONSDIALOG_H

#include <QDialog>

namespace StereoVisionApp {

namespace Ui {
class HexagonalTargetDetectionOptionsDialog;
}

class HexagonalTargetDetectionOptionsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit HexagonalTargetDetectionOptionsDialog(QWidget *parent = nullptr);
	~HexagonalTargetDetectionOptionsDialog();

	double minThreshold() const;
	void setMinThreshold(double threshold);

	double diffThreshold() const;
	void setDiffThreshold(double threshold);

	int minArea() const;
	void setMinArea(int minArea);

	int maxArea() const;
	void setMaxArea(int maxArea);

	double minToMaxAxisRatioThreshold() const;
	void setMinToMaxAxisRatioThreshold(double threshold);

	double hexagonMaxRelDiameter() const;
	void setHexagonMaxRelDiameter(double threshold);

	double redGain() const;
	void setRedGain(double gain);

	double greenGain() const;
	void setGreenGain(double gain);

	double blueGain() const;
	void setBlueGain(double gain);

	bool replaceOld() const;
	void setReplaceOld(bool replace);

private:
	Ui::HexagonalTargetDetectionOptionsDialog *ui;
};

} // namespace StereoVisionApp

#endif // HEXAGONALTARGETDETECTIONOPTIONSDIALOG_H
