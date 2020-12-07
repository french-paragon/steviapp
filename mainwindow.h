#ifndef STEREOVISIONAPP_MAINWINDOW_H
#define STEREOVISIONAPP_MAINWINDOW_H

#include <QMainWindow>
#include <QMap>

namespace StereoVisionApp {

class Project;
class Image;

class Editor;
class EditorFactory;

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

	void newEmptyProject();
	void saveProject();
	void openProject();
	void saveProjectAs();

	Editor* openEditor(QString editorClassName);
	bool isEditorAvailable(QString editorClassName) const;
	void installEditor(EditorFactory* factory);
	void closeEditor(int index);

	void openSparseViewer();

private:

	void resetProject();

	void configureProjectWindowsMenu();
	void projectContextMenu(QPoint const& pt);
	void onProjectSelectionChanged();

	void runSparseOptim();

	Ui::MainWindow *ui;

	Project* _activeProject;

	QMap<QString, EditorFactory*> _installedEditors;
	QMap<QString, Editor*> _openedEditors;
};

} // namespace StereoVisionApp
#endif // STEREOVISIONAPP_MAINWINDOW_H
