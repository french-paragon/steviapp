#ifndef STEREOVISIONAPP_APPLICATION_H
#define STEREOVISIONAPP_APPLICATION_H

#include <QObject>

class QCoreApplication;

namespace StereoVisionApp {

class Project;
class MainWindow;

class StereoVisionApplication : public QObject
{
public:

	static StereoVisionApplication* GetCameraApp();

	StereoVisionApplication(int & argc, char** argv);
	~StereoVisionApplication();

	Project* getCurrentProject() const;
	MainWindow* mainWindow() const;

	void newEmptyProject();
	void saveProject();
	void openProject(QString const& fname);
	void saveProjectAs(QString const& fname);

	/*!
	 * \brief exec start the application event loop
	 * \return the final status code of the application.
	 */
	int exec();

protected:

	void resetProject();

	Project* _headLessProject;
	MainWindow* _mw;

	bool _isHeadLess;

	QCoreApplication* _QtApp;

	QString _openProjectFile;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_APPLICATION_H
