#ifndef STEREOVISIONAPP_APPLICATION_H
#define STEREOVISIONAPP_APPLICATION_H

#include <QObject>

class QCoreApplication;

namespace StereoVisionApp {

class Project;
class MainWindow;

class StereoVisionApplication : public QObject
{
	Q_OBJECT
public:

	static StereoVisionApplication* GetAppInstance();

	StereoVisionApplication(int & argc, char** argv);
	~StereoVisionApplication();

	Project* getCurrentProject() const;
	MainWindow* mainWindow() const;

	void newEmptyProject();
	void saveProject();
	void openProject(QString const& fname);
	void saveProjectAs(QString const& fname);

    /*!
     * \brief loadRessources load some ressources, like application plugins.
     */
    void loadRessources();

	/*!
	 * \brief exec start the application event loop
	 * \return the final status code of the application.
	 */
	int exec();

Q_SIGNALS:

	void scriptFileExecututionRequested(QString scriptpath, QStringList argv);

protected:

	void resetProject();

    /*!
     * \brief loadApplicationPlugins load the plugins for the application
     *
     * Plugins are usually requested via the command line argument
     */
    void loadApplicationPlugins(); //TODO: also list permanent plugins in the config file.

	static StereoVisionApplication* CurrentApp;

	Project* _headLessProject;
	MainWindow* _mw;

	bool _isHeadLess;

	QCoreApplication* _QtApp;


	QVector<QString> _scriptFiles;
	QStringList _scriptsargs;
	QString _openProjectFile;
    QVector<QString> _requestedPlugins;

    bool _ressource_load_flag;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_APPLICATION_H
