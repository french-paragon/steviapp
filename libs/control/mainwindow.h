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

    /*!
     * \brief openEditor open a specific editor and set the current tab to said editor
     * \param editorClassName the editor class
     * \param additionalId an additional id to allow to distinguish multiple editors of the same class open at once.
     * \return a pointer to the editor, or nullptr if the editor is not open and could not be created
     */
    Editor* openEditor(QString editorClassName, QString additionalId = "");
	bool isEditorAvailable(QString editorClassName) const;
    /*!
     * \brief openedEditor return a specific editor if it is currently open
     * \param editorClassName the class of the editor
     * \param additionalId an additional id to allow to distinguish multiple editors of the same class open at once.
     * \return a pointer to the editor, or nullptr if the editor is not open
     */
    Editor* openedEditor(QString editorClassName, QString additionalId = "");
    void jumpToEditor(Editor* editor);


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

    inline static QString buildFullEditorId(QString editorClassName, QString additionalId) {
        return editorClassName + additionalId;
    }

	Ui::MainWindow *ui;

	Project* _activeProject;

	QMap<QString, EditorFactory*> _installedEditors;
	QMap<QString, Editor*> _openedEditors;
};

} // namespace StereoVisionApp
#endif // STEREOVISIONAPP_MAINWINDOW_H
