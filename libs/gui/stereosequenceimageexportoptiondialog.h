#ifndef STEREOSEQUENCEIMAGEEXPORTOPTIONDIALOG_H
#define STEREOSEQUENCEIMAGEEXPORTOPTIONDIALOG_H

#include <QDialog>


namespace StereoVisionApp {

namespace Ui {
class StereoSequenceImageExportOptionDialog;
}

class StereoSequenceImageExportOptionDialog : public QDialog
{
	Q_OBJECT

public:
	explicit StereoSequenceImageExportOptionDialog(QWidget *parent = nullptr);
	~StereoSequenceImageExportOptionDialog();

	QString exportDir() const;
	void setExportDir(QString const& dir);

	QString leftBackgroundImage() const;
	void setLeftBackgroundImage(QString const& dir);

	QString rightBackgroundImage() const;
	void setRightBackgroundImage(QString const& dir);

	int searchWidth() const;
	void setSearchWidth(int width);

	int searchRadius() const;
	void setSearchRadius(int radius);

	int hiearchicalLevel() const;
	void setHiearchicalLevel(int level);

	int transitionCostWeight() const;
	void setTransitionCostWeight(int weight);

	int visualWeight() const;
	void setVisualWeight(int weight);

	int visualPatchRadius() const;
	void setVisualPatchRadius(int radius);

	float visualThreshold() const;
	void setVisualThreshold(float threshold);

	int depthWeight() const;
	void setDepthWeight(int weight);

	int depthThreshold() const;
	void setDepthThreshold(int threshold);


private:

	void openExportDirectory();

	void openLeftBgImage();
	void openRightBgImage();

	QString openImage();

	Ui::StereoSequenceImageExportOptionDialog *ui;
};

} // namespace StereoVisionApp

#endif // STEREOSEQUENCEIMAGEEXPORTOPTIONDIALOG_H
