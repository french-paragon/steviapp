#include "editor.h"

namespace StereoVisionApp {

Editor::Editor(QWidget *parent) :
	QWidget(parent),
	_activeProject(nullptr)
{

}

void Editor::setProject(Project* p) {
	beforeProjectChange(p);
	Project* op = _activeProject;
	_activeProject = p;
	afterProjectChange(op);
}

void Editor::beforeProjectChange(Project*) {
	return;
}
void Editor::afterProjectChange(Project*) {
	return;
}

Project *Editor::activeProject() const
{
	return _activeProject;
}


EditorFactory::EditorFactory(QObject *parent) : QObject(parent)
{

}

QString EditorFactory::itemClassName() const {
	Editor* e = factorizeEditor(nullptr);
	QString cs = e->metaObject()->className();
	delete e;
	return cs;
}

} // namespace StereoVisionApp
