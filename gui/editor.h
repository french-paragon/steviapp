#ifndef STEREOVISIONAPP_EDITOR_H
#define STEREOVISIONAPP_EDITOR_H

#include <QWidget>

namespace StereoVisionApp {

class Project;

class Editor : public QWidget
{
	Q_OBJECT
public:
	explicit Editor(QWidget *parent = nullptr);

	Project *activeProject() const;
	void setProject(Project* p);

Q_SIGNALS:

protected:

	virtual void beforeProjectChange(Project* np);
	virtual void afterProjectChange(Project* op);

	Project* _activeProject;

};

class EditorFactory : public QObject
{
	Q_OBJECT
public:
	explicit EditorFactory(QObject *parent = nullptr);


	virtual QString TypeDescrName() const = 0;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const = 0;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDITOR_H
