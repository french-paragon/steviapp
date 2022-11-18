#ifndef STEREOVISIONAPP_LOCALCOORDINATESYSTEMPOINTDETAILSEDITOR_H
#define STEREOVISIONAPP_LOCALCOORDINATESYSTEMPOINTDETAILSEDITOR_H

#include "editor.h"

namespace StereoVisionApp {

class LocalCoordinateSystem;
class LocalCoordinatesPointsSolutionModel;

namespace Ui {
class LocalCoordinateSystemPointDetailsEditor;
}

class LocalCoordinateSystemPointDetailsEditor : public Editor
{
	Q_OBJECT

public:
	explicit LocalCoordinateSystemPointDetailsEditor(QWidget *parent = nullptr);
	~LocalCoordinateSystemPointDetailsEditor();

	void setLocalCoordinateSystem(LocalCoordinateSystem* lcs);

private:
	Ui::LocalCoordinateSystemPointDetailsEditor *ui;

	LocalCoordinatesPointsSolutionModel* _model;
};

class LocalCoordinateSystemPointDetailsEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit LocalCoordinateSystemPointDetailsEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;

};

} //namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALCOORDINATESYSTEMPOINTDETAILSEDITOR_H
