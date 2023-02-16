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

	int hiearchicalLevel() const;
	void setHiearchicalLevel(int level);

	int transitionCostWeight() const;
	void setTransitionCostWeight(int weight);

	int erosionRadius() const;
	void setErosionRadius(int radius);

	int dilationRadius() const;
	void setDilationRadius(int radius);

	int extensionRadius() const;
	void setExtensionRadius(int radius);

	int nHistogramBins() const;
	void setNHistogramBins(int nBins);

	int cutoffHistogramBin() const;
	void setCutoffHistogramBins(int cutoff);


private:

	void openExportDirectory();

	void openLeftBgImage();
	void openRightBgImage();

	QString openImage();

	Ui::StereoSequenceImageExportOptionDialog *ui;
};

} // namespace StereoVisionApp

#endif // STEREOSEQUENCEIMAGEEXPORTOPTIONDIALOG_H
