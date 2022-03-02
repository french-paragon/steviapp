#ifndef STEREOVISIONAPP_IMAGEPOINTDETAILSEDITOR_H
#define STEREOVISIONAPP_IMAGEPOINTDETAILSEDITOR_H

#include "editor.h"

namespace StereoVisionApp {

class Image;
class ImagePointsSolutionModel;

namespace Ui {
class ImagePointDetailsEditor;
}

class ImagePointDetailsEditor : public Editor
{
	Q_OBJECT

public:
	explicit ImagePointDetailsEditor(QWidget *parent = nullptr);
	~ImagePointDetailsEditor();

	void setImage(Image* img);

private:
	Ui::ImagePointDetailsEditor *ui;

	ImagePointsSolutionModel* _model;
};

class ImagePointDetailsEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit ImagePointDetailsEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEPOINTDETAILSEDITOR_H
