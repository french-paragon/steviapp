#include "application.h"

#include <QCoreApplication>
#include <QApplication>

#include <QPluginLoader>
#include <QFileInfo>
#include <QDir>

#include <QVector>
#include <QString>
#include <QDebug>

#include "mainwindow.h"
#include "stereoappplugininterface.h"

#include "datablocks/project.h"

namespace StereoVisionApp {

StereoVisionApplication* StereoVisionApplication::CurrentApp = nullptr;

StereoVisionApplication* StereoVisionApplication::GetAppInstance() {
	return CurrentApp;
}

StereoVisionApplication::StereoVisionApplication(int &argc, char **argv) :
	_headLessProject(nullptr),
    _mw(nullptr),
    _ressource_load_flag(false)
{

	_isHeadLess = false;

	_openProjectFile = "";

	for (int i = 1; i < argc; i++) {
		if (!qstrcmp(argv[i], "--headless")) {
			_isHeadLess = true;

		} else if (!qstrcmp(argv[i], "--script")) {
			i++;
			if (i < argc) {
				_scriptFiles.push_back(QString(argv[i]));
			}

        } else if (!qstrcmp(argv[i], "--module")) {
            i++;
            if (i < argc) {
                _requestedPlugins.push_back(QString(argv[i]));
            }

        } else if (!qstrcmp(argv[i], "--")) {

			for (i=i+1; i < argc; i++) {
				_scriptsargs.push_back(argv[i]);
			}
			break;

		} else {
			_openProjectFile = QString(argv[i]);
		}
	}

	if (_isHeadLess) {
		_QtApp = new QCoreApplication(argc, argv);
	} else {
		_QtApp = new QApplication(argc, argv);
		_mw = new MainWindow();
	}

	CurrentApp = this;

}

StereoVisionApplication::~StereoVisionApplication() {

	if (_mw != nullptr) {
		delete _mw;
	}

    if (_QtApp != nullptr) {
        delete _QtApp;
    }
}

Project* StereoVisionApplication::getCurrentProject() const {
	if (_mw != nullptr) {
		return _mw->activeProject();

	} else if (_isHeadLess) {
		return _headLessProject;
	}

	return nullptr;
}

MainWindow* StereoVisionApplication::mainWindow() const
{
	return _mw;
}

void StereoVisionApplication::newEmptyProject() {

	if (_mw != nullptr) {
		_mw->newEmptyProject();

	} else {
		resetProject();
	}

}

void StereoVisionApplication::saveProject() {

	if (_mw != nullptr) {
		_mw->saveProject();

	} else if (_isHeadLess) {

		if (_headLessProject == nullptr) {
			return;
		}

		_headLessProject->save();
	}

}
void StereoVisionApplication::openProject(QString const& fname) {

	if (_mw != nullptr) {
		_mw->openProject(fname);
	} else if (_isHeadLess) {

		if (!fname.isEmpty()) {
			resetProject();
			_headLessProject->load(fname);
		}
	}

}
void StereoVisionApplication::saveProjectAs(QString const& fname) {

	if (_mw != nullptr) {
		_mw->saveProjectAs(fname);
	} else {

		if (_headLessProject == nullptr) {
			return;
		}

		if (!fname.isEmpty()) {
			_headLessProject->save(fname);
		}
	}

}

void StereoVisionApplication::loadRessources() {

    if (_ressource_load_flag) {
        return; //load the ressources only once
    }

    _ressource_load_flag = true;

    //load the plugins
    loadApplicationPlugins();
}
int StereoVisionApplication::exec() {

	if (_QtApp == nullptr) {
		return 1;
	}

	if (!_openProjectFile.isEmpty()) {
		openProject(_openProjectFile);
	}

	if (_mw != nullptr) {
		_mw->show();
	}

	for (QString const& scriptpath : qAsConst(_scriptFiles)) {
		Q_EMIT scriptFileExecututionRequested(scriptpath, _scriptsargs);
	}

	if (_isHeadLess) {
		return 0;
	}

	return _QtApp->exec();

}

void StereoVisionApplication::resetProject() {

	if (_headLessProject != nullptr) {
		_headLessProject->deleteLater();
	}

	if (_isHeadLess) {
		_headLessProject = StereoVisionApp::ProjectFactory::defaultProjectFactory().createProject(this);
	}
}

void StereoVisionApplication::loadApplicationPlugins() {

    QVector<QString> pluginsToLoad = _requestedPlugins;

    for (QString rawPluginPath : pluginsToLoad) {

        QString pluginPath = rawPluginPath;

#ifndef NDEBUG
        //try to see if a debug build of the plugin exist, and load it instead.
        QFileInfo pluginPathInfos(pluginPath);

        QDir dir = pluginPathInfos.dir();
        QString debugName = pluginPathInfos.baseName() + "_d." + pluginPathInfos.completeSuffix();

        QFileInfo pluginDebugPathInfos(dir.filePath(debugName));

        if (pluginDebugPathInfos.exists()) {
            pluginPath = pluginDebugPathInfos.filePath();
        }
#endif

        QPluginLoader* plugin = new QPluginLoader(pluginPath, this);
        plugin->load();

        if (plugin->instance() == nullptr) {
            delete plugin;
            continue;
        }

        StereoAppPluginInterface* stereoappplugin = qobject_cast<StereoAppPluginInterface*>(plugin->instance());

        if (stereoappplugin != nullptr) {
            stereoappplugin->loadModule(this);
        } else {
            delete plugin;
            continue;
        }

    }

}

} // namespace StereoVisionApp
