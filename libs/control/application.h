#ifndef STEREOVISIONAPP_APPLICATION_H
#define STEREOVISIONAPP_APPLICATION_H

#include <QObject>
#include <QMap>

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
     * \brief registerAdditionalInterface register a QObject as an arbitrary interface in the app.
     * \param name the name of the interface
     * \param interface the interface to register, the application takes ownership of the interface on success
     * \return true if the interface could be registered, false otherwise (e.g. if an interface with such a name already exist).
     *
     * The interface is an arbitrary QObject, these can be casted safely using qobject_cast.
     * This allow to extend the application with arbitrary functions that can be accessed in a safe way.
     */
    bool registerAdditionalInterface(QString const& name, QObject* interface);

    /*!
     * \brief getAdditionalInterface get an additional interface of the application
     * \param name the name that was used to register the interface.
     * \return a pointer to the interface, or nullptr in case of error (e.g. if the interface is not installed.
     */
    QObject* getAdditionalInterface(QString const& name) const;

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

    QMap<QString, QObject*> _additionalInterfaces;

    bool _ressource_load_flag;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_APPLICATION_H
