#ifndef STEREOSEQUENCEEXPORTOPTIONDIALOG_H
#define STEREOSEQUENCEEXPORTOPTIONDIALOG_H

#include <QDialog>

namespace StereoVisionApp {

class ExportableStereoSequence;

namespace Ui {
class StereoSequenceExportOptionDialog;
}

class StereoSequenceExportOptionDialog : public QDialog
{
	Q_OBJECT

public:
	explicit StereoSequenceExportOptionDialog(QWidget *parent = nullptr);
	~StereoSequenceExportOptionDialog();

	QString exportDir() const;
	void setExportDir(QString const& dir);

	int searchWidth() const;
	void setSearchWidth(int width);

	int searchRadius() const;
	void setSearchRadius(int radius);

	float maxDist() const;
	void setMaxDist(float dist);

	int maxDispDelta() const;
	void setMaxDispDelta(int disp_delta);

	int erodingDistance() const;
	void setErodingDistance(int dist);

	int openingDistance() const;
	void setOpeningDistance(int dist);

	bool exportImages() const;

private:

	void onOpenDirButtonClicked();

	Ui::StereoSequenceExportOptionDialog *ui;
};

} // namespace StereoVisionApp

#endif // STEREOSEQUENCEEXPORTOPTIONDIALOG_H
