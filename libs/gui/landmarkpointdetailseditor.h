#ifndef STEREOVISIONAPP_LANDMARKPOINTDETAILSEDITOR_H
#define STEREOVISIONAPP_LANDMARKPOINTDETAILSEDITOR_H

#include "editor.h"

namespace StereoVisionApp {

class Landmark;
class LandmarkPointsSolutionModel;

namespace Ui {
class LandmarkPointDetailsEditor;
}

class LandmarkPointDetailsEditor : public Editor
{
	Q_OBJECT
public:
	explicit LandmarkPointDetailsEditor(QWidget *parent = nullptr);
	~LandmarkPointDetailsEditor();

	void setLandmark(Landmark* img);

private:
	Ui::LandmarkPointDetailsEditor *ui;

	LandmarkPointsSolutionModel* _model;
};

class LandmarkPointDetailsEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit LandmarkPointDetailsEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKPOINTDETAILSEDITOR_H
