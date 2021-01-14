#ifndef RECTIFIEDIMAGEEXPORTOPTIONSDIALOG_H
#define RECTIFIEDIMAGEEXPORTOPTIONSDIALOG_H

#include <QDialog>

namespace StereoVisionApp {

namespace Ui {
class RectifiedImageExportOptionsDialog;
}

class RectifiedImageExportOptionsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit RectifiedImageExportOptionsDialog(QWidget *parent = nullptr);
	~RectifiedImageExportOptionsDialog();

	QString selectedFolder() const;
	bool useOptimizedCameraParameters() const;
	float gamma() const;

protected:

	void onFolderSelectButtonClicked();

private:
	Ui::RectifiedImageExportOptionsDialog *ui;
};

} //namespace StereoVisionApp
#endif // RECTIFIEDIMAGEEXPORTOPTIONSDIALOG_H
