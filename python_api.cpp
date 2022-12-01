#include <pybind11/pybind11.h>

#include <array>

#include "control/application.h"

#include "datablocks/project.h"

#include <QString>
#include <QVector>

namespace py = pybind11;

/*!
 * \brief The PythonStereoVisionApp class serve as an access point to an app instance
 *
 * This class will check if an app exist or, if not, create one.
 */
class PythonStereoVisionApp {

public:

	PythonStereoVisionApp(std::string const& appLocation = "", bool headless = true);

	StereoVisionApp::StereoVisionApplication* getApp() const;

	void openProject(std::string const& path);
	void saveProject();
	void saveProjectAs(std::string const& path);

	void printDataBlocks() const;

private:

	std::string _app_argv;
	std::array<char*, 2> _argv_ptrs;
	StereoVisionApp::StereoVisionApplication* _instancedApp;

};

PythonStereoVisionApp::PythonStereoVisionApp(std::string const& appLocation, bool headless) {

	_app_argv = appLocation + ((headless) ? "\0 --headless" : "");
	_argv_ptrs[0] = const_cast<char*>(_app_argv.c_str());

	if (headless) {
		_argv_ptrs[1] = const_cast<char*>(_app_argv.c_str()) + _app_argv.size() - 10;
	}

	StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

	//create an app instance if the python library is used outside of the app.
	if (app == nullptr) {
		int argc = (headless) ? 2 : 1;
		_instancedApp = new StereoVisionApp::StereoVisionApplication(argc, _argv_ptrs.data());
	}

}

StereoVisionApp::StereoVisionApplication* PythonStereoVisionApp::getApp() const {
	if (_instancedApp != nullptr) {
		return _instancedApp;
	}

	return StereoVisionApp::StereoVisionApplication::GetAppInstance();
}



void PythonStereoVisionApp::openProject(std::string const& path) {
	StereoVisionApp::StereoVisionApplication* app = getApp();

	if (app == nullptr) {
		py::print("Ho no, cannot access a valid app instance !");
		return;
	}

	app->openProject(QString::fromStdString(path));
}
void PythonStereoVisionApp::saveProject() {

	StereoVisionApp::StereoVisionApplication* app = getApp();

	if (app == nullptr) {
		py::print("Ho no, cannot access a valid app instance !");
		return;
	}

	app->saveProject();

}
void PythonStereoVisionApp::saveProjectAs(std::string const& path) {

	StereoVisionApp::StereoVisionApplication* app = getApp();

	if (app == nullptr) {
		py::print("Ho no, cannot access a valid app instance !");
		return;
	}

	app->saveProjectAs(QString::fromStdString(path));
}

void PythonStereoVisionApp::printDataBlocks() const {

	StereoVisionApp::StereoVisionApplication* app = getApp();

	if (app == nullptr) {
		py::print("Ho no, cannot access a valid app instance !");
		return;
	}

	StereoVisionApp::Project* proj = app->getCurrentProject();

	if (proj == nullptr) {
		py::print("No open project !");
		return;
	}

	for (QString const& className : proj->installedTypes()) {

		py::print(className.toStdString());

		QVector<qint64> idxs = proj->getIdsByClass(className);

		for (qint64 id : qAsConst(idxs)) {
			StereoVisionApp::DataBlock* block = proj->getById(id);

			if (block == nullptr) {
				continue;
			}

			py::print("\t", block->objectName().toStdString());
		}

	}
}

PYBIND11_MODULE(pysteviapp, m) {
	m.doc() = "Stereovision app python interface";

	py::class_<PythonStereoVisionApp>(m, "AppInstance")
			.def(py::init<const std::string &, bool>(), py::arg("appLocation") = "", py::arg("headless") = true)
			.def("openProject", &PythonStereoVisionApp::openProject, py::arg("path"))
			.def("saveProject", &PythonStereoVisionApp::saveProject)
			.def("saveProjectAs", &PythonStereoVisionApp::saveProjectAs, py::arg("path"))
			.def("printDatablocks", &PythonStereoVisionApp::printDataBlocks);

}
