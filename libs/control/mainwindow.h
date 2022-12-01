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

	static MainWindow* getActiveMainWindow();

	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

	void newEmptyProject();
	void saveProject();
	void openProject();
	void openProject(QString const& fname);
	void saveProjectAs();
	void saveProjectAs(QString const& fname);

	Editor* openEditor(QString editorClassName);
	bool isEditorAvailable(QString editorClassName) const;
	void installEditor(EditorFactory* factory);
	void closeEditor(int index);

	void openSparseViewer();

	Project* activeProject() const;

private:

	void displayInfoMessage(QString msg);

	void resetProject();

	void configureProjectWindowsMenu();
	void projectContextMenu(QPoint const& pt);
	void onProjectSelectionChanged();

	void datablockContextMenu(QPoint const& pt);

	void clearOptimSolution();
	void runCoarseOptim();
	void runSparseOptim();

	Ui::MainWindow *ui;

	Project* _activeProject;

	QMap<QString, EditorFactory*> _installedEditors;
	QMap<QString, Editor*> _openedEditors;
};

} // namespace StereoVisionApp
#endif // STEREOVISIONAPP_MAINWINDOW_H
