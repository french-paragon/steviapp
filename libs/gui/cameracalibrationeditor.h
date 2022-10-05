#ifndef CAMERACALIBRATIONEDITOR_H
#define CAMERACALIBRATIONEDITOR_H

#include <QWidget>

#include "./editor.h"
#include "./imagewidget.h"

class QSortFilterProxyModel;

namespace StereoVisionApp {

class CameraCalibration;
class CheckboardPtsDrawable;

namespace Ui {
class CameraCalibrationEditor;
}

class CameraCalibrationEditor : public Editor
{
	Q_OBJECT

public:
	explicit CameraCalibrationEditor(QWidget *parent = nullptr);
	~CameraCalibrationEditor();

	void setCalibration(CameraCalibration* cam);

Q_SIGNALS:

	void optimizeCalibrationTriggered(Project* p, qint64 calibrationid);

protected:


private:

	void openContextMenu(QPoint const& pos);
	void optimizationTriggered();

	void moveToNextImage();
	void moveToPreviousImage();
	void moveToImage(int row);

	void moveToImage(QModelIndex idx);

	void detectCheckBoardsAllImages();
	void detectCheckBoards(qint64 id);

	void imgsListContextMenu(QPoint const& position);

	Ui::CameraCalibrationEditor *ui;

	QAction* _moveToNextImg;
	QAction* _moveToPrevImg;

	int _currentId;
	CameraCalibration* _currentCalib;

	CheckboardPtsDrawable* _checkBoardDrawer;

	QSortFilterProxyModel* _proxyModel;
};

class CheckboardPtsDrawable : public ImageWidgetDrawable
{
	Q_OBJECT
public:

	explicit CheckboardPtsDrawable(QWidget *parent);

	void setCalibration(CameraCalibration* calib);
	void setImgId(qint64 id);

protected:

	void paintItemImpl(QPainter* painter) const override;

	qint64 _id;
	CameraCalibration* _calib;
};

class CameraCalibrationEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit CameraCalibrationEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;


};


} // namespace StereoVisionApp

#endif // CAMERACALIBRATIONEDITOR_H
