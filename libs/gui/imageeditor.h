#ifndef IMAGEEDITOR_H
#define IMAGEEDITOR_H

#include <QWidget>

#include "./editor.h"

namespace QImageDisplay {
class ImageWidget;
}

namespace StereoVisionApp {

class Image;
class ImageDatablockDisplayAdapter;
class ImageLandmarksOverlay;

namespace Ui {
class ImageEditor;
}

class ImageEditor : public Editor
{
	Q_OBJECT

public:

	static const QString ImageEditorClassName;

	explicit ImageEditor(QWidget *parent = nullptr);
	~ImageEditor();

	void setImage(Image* img);

protected:

	void afterProjectChange(Project* op);

	void addPoint(QPointF const& imageCoordinates);
    void openPointMenu(qint64 id, QImageDisplay::ImageWidget *widget, QPoint const& pos);
	void openNonPointMenu(QPoint const& pos);

	void moveToNextImage();
	void moveToPreviousImage();

	void moveToNextLandmark();
	void moveToPreviousLandmark();

	void onCurrentImageIndexChanged(int id);
	void onCurrentLandmarkIndexChanged();

	void clearCombobox();
	void setComboboxIndices();

	void onViewSingleStateChanged();

private:
	Ui::ImageEditor *ui;

	QAction* _moveToNextImg;
	QAction* _moveToPrevImg;

	QAction* _moveToNextLandmark;
	QAction* _moveToPrevLandmark;

	qint64 _current_image_id;
	qint64 _current_landmark_id;

    ImageDatablockDisplayAdapter* _img_display_adapter;
    ImageLandmarksOverlay* _img_landmark_overlay;
};

class ImageEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit ImageEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;


};

} // namespace StereoVisionApp

#endif // IMAGEEDITOR_H
