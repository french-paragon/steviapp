#ifndef CAMERACALIBRATIONSPARSEALIGNEMENTEDITOR_H
#define CAMERACALIBRATIONSPARSEALIGNEMENTEDITOR_H

#include <QWidget>

#include "./editor.h"

namespace StereoVisionApp {

class CameraCalibration;
class CameraCalibrationSparseAlignementViewerInterface;

namespace Ui {
class CameraCalibrationSparseAlignementEditor;
}

class CameraCalibrationSparseAlignementEditor : public Editor
{
	Q_OBJECT

public:
	explicit CameraCalibrationSparseAlignementEditor(QWidget *parent = nullptr);
	~CameraCalibrationSparseAlignementEditor();

	void setCalibration(CameraCalibration* p);
	void clearCalibration();

	float getGridSize() const;
	void setGridSize(float grid_size);

private:
	Ui::CameraCalibrationSparseAlignementEditor *ui;

	CameraCalibrationSparseAlignementViewerInterface* _viewerInterface;
};

class CameraCalibrationSparseAlignementEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit CameraCalibrationSparseAlignementEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;


};

} // namespace StereoVisionApp

#endif // CAMERACALIBRATIONSPARSEALIGNEMENTEDITOR_H
