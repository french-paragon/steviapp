#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>

#include "control/application.h"

#include "datablocks/floatparameter.h"
#include "datablocks/project.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"
#include "datablocks/stereorig.h"
#include "datablocks/localcoordinatesystem.h"

#include "control/imagebaseactions.h"
#include "control/solversactions.h"
#include "control/localcoordinatesystemactions.h"
#include "control/stereorigactions.h"

#include <QString>
#include <QVector>
#include <QFile>
#include <QFileInfo>

#include <QJsonDocument>
#include <QJsonObject>

namespace py = pybind11;

class Watcher : public QObject {
	Q_OBJECT
public:
	explicit Watcher(QObject* watchedObject = nullptr, QObject* parent = nullptr) :
		QObject(parent),
		_watchedIsAlive(watchedObject != nullptr)
	{
		if (watchedObject != nullptr) {
			connect(watchedObject, &QObject::destroyed, this, [this] () {_watchedIsAlive = false;});
		}
	}

	bool watchedIsAlive() const{
		return _watchedIsAlive;
	}
protected:
	bool _watchedIsAlive;
};

class DataBlockReference {
public:

	DataBlockReference() : //build an invalid reference
		_watcher(),
		_datablock(nullptr) {

	}

	DataBlockReference(StereoVisionApp::DataBlock* datablock) :
		_watcher(datablock),
		_datablock(datablock) {

	}
	DataBlockReference(DataBlockReference const& other) :
		_watcher(other._datablock),
		_datablock(other._datablock) {

	}

	qint64 getInternalId() const {
		if (_watcher.watchedIsAlive()) {
			return _datablock->internalId();
		}
		return -1;
	}

	std::string name() const {
		if (_watcher.watchedIsAlive()) {
			return _datablock->objectName().toStdString();
		}
		return "";
	}
	void setObjectName(std::string const& name) {
		if (_watcher.watchedIsAlive()) {
			_datablock->setObjectName(QString::fromStdString(name));
		}
	}

	bool isFixed() const {
		if (_watcher.watchedIsAlive()) {
			return _datablock->isFixed();
		}
		return -1;
	}
	void setFixed(bool fixed) {
		if (_watcher.watchedIsAlive()) {
			_datablock->setFixed(fixed);
		}
	}

	std::string blockType() const {
		if (_watcher.watchedIsAlive()) {
			return _datablock->metaObject()->className();
		}
		return "Invalid";
	}

	StereoVisionApp::DataBlock* datablock() const {
		if (_watcher.watchedIsAlive()) {
			return _datablock;
		}
		return nullptr;
	}

	bool isValid() const {
		return _watcher.watchedIsAlive();
	}

protected:
	Watcher _watcher;
	StereoVisionApp::DataBlock* _datablock;
};

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

	DataBlockReference addImage(std::string const& filename, std::string const& name) {

		StereoVisionApp::StereoVisionApplication* app = getApp();

		if (app == nullptr) {
			py::print("Ho no, cannot access a valid app instance !");
			return DataBlockReference();
		}

		StereoVisionApp::Project* proj = app->getCurrentProject();

		if (proj == nullptr) {
			py::print("No open project !");
			return DataBlockReference();
		}

		qint64 id = StereoVisionApp::addImage(QString::fromStdString(filename), proj);

		if (id < 0) {
			return DataBlockReference();
		}

		StereoVisionApp::Image* img = proj->getDataBlock<StereoVisionApp::Image>(id);

		img->setObjectName(QString::fromStdString(name));

		return DataBlockReference(img);

	}

	DataBlockReference addImageWithCam(std::string const& filename, std::string const& name, DataBlockReference camera) {

		StereoVisionApp::StereoVisionApplication* app = getApp();

		if (app == nullptr) {
			py::print("Ho no, cannot access a valid app instance !");
			return DataBlockReference();
		}

		StereoVisionApp::Project* proj = app->getCurrentProject();

		if (proj == nullptr) {
			py::print("No open project !");
			return DataBlockReference();
		}

		StereoVisionApp::Camera* cam = qobject_cast<StereoVisionApp::Camera*>(camera.datablock());

		if (cam == nullptr) {
			py::print("Invalid camera provided !");
			return DataBlockReference();
		}

		qint64 id = StereoVisionApp::addImage(QString::fromStdString(filename), proj, cam);

		if (id < 0) {
			return DataBlockReference();
		}

		StereoVisionApp::Image* img = proj->getDataBlock<StereoVisionApp::Image>(id);

		img->setObjectName(QString::fromStdString(name));

		return DataBlockReference(img);

	}

	std::vector<DataBlockReference> addImages(std::vector<std::string> const& filenames) {

		std::vector<DataBlockReference> ret;
		ret.reserve(filenames.size());

		for (std::string const& filename : filenames) {

			QFileInfo infos(QString::fromStdString(filename));

			DataBlockReference ref = addImage(filename, infos.baseName().toStdString());

			if (ref.isValid()) {
				ret.emplace_back(ref);
			}
		}

		return ret;

	}

	template<typename T>
	DataBlockReference addDatablock(std::string const& name) {

		StereoVisionApp::StereoVisionApplication* app = getApp();

		if (app == nullptr) {
			py::print("Ho no, cannot access a valid app instance !");
			return DataBlockReference();
		}

		StereoVisionApp::Project* proj = app->getCurrentProject();

		if (proj == nullptr) {
			py::print("No open project !");
			return DataBlockReference();
		}

		qint64 id = proj->createDataBlock(T::staticMetaObject.className());

		if (id < 0) {
			return DataBlockReference();
		}

		StereoVisionApp::DataBlock* block = proj->getById(id);

		block->setObjectName(QString::fromStdString(name));

		return DataBlockReference(block);

	}

	template<typename T>
	void configureDatablockDataFromJson(DataBlockReference datablock, std::string const& jsonFilePath) {

		if (!datablock.isValid()) {
			py::print("Invalid datablock reference provided !");
			return;
		}

		T* block = qobject_cast<T*>(datablock.datablock());

		if (block == nullptr) {
			py::print("Provided datablock reference is not of the correct class !");
			return;
		}

		QString inFilePath = QString::fromStdString(jsonFilePath);

		if (inFilePath.isEmpty()) {
			return;
		}

		QFile infile(inFilePath);
		bool opened = infile.open(QIODevice::ReadOnly | QIODevice::Text);

		if (!opened) {
			return;
		}

		QByteArray data = infile.readAll();
		infile.close();

		QJsonDocument doc = QJsonDocument::fromJson(data);
		QJsonObject blockRep = doc.object();

		block->setParametersFromJsonRepresentation(blockRep);

	}


	DataBlockReference getDataBlockByName(std::string const& name, std::string const& typeHint = "") {

		StereoVisionApp::StereoVisionApplication* app = getApp();

		if (app == nullptr) {
			py::print("Ho no, cannot access a valid app instance !");
			return DataBlockReference();
		}

		StereoVisionApp::Project* proj = app->getCurrentProject();

		if (proj == nullptr) {
			py::print("No open project !");
			return DataBlockReference();
		}

		StereoVisionApp::DataBlock* block = proj->getByName(QString::fromStdString(name), QString::fromStdString(typeHint));

		return DataBlockReference(block);
	}

	void setImageAsReference(DataBlockReference image) {

		StereoVisionApp::Image* img = qobject_cast<StereoVisionApp::Image*>(image.datablock());

		if (img == nullptr) {
			py::print("Camera datablock is not a reference to a camera !");
			return;
		}

		StereoVisionApp::floatParameterGroup<3> zero;
		zero.value(0) = 0;
		zero.value(1) = 0;
		zero.value(2) = 0;
		zero.setIsSet();

		img->setOptPos(zero);
		img->setOptRot(zero);

	}

	void assignImagesToCamera(std::vector<DataBlockReference> const& images, DataBlockReference camera) {

		StereoVisionApp::Camera* cam = qobject_cast<StereoVisionApp::Camera*>(camera.datablock());

		if (cam == nullptr) {
			py::print("Camera datablock is not a reference to a camera !");
			return;
		}

		for (DataBlockReference const& db_ref : images) {
			StereoVisionApp::Image* img = qobject_cast<StereoVisionApp::Image*>(db_ref.datablock());

			if (img == nullptr) {
				py::print("Datablock \"", db_ref.name(),"\" is not a reference to an image !");
				continue;
			}

			img->assignCamera(cam->internalId());
		}
	}

	void assignImagesToStereoRig(DataBlockReference stereoRig, DataBlockReference img1, DataBlockReference img2) {

		StereoVisionApp::StereoRig* rig = qobject_cast<StereoVisionApp::StereoRig*>(stereoRig.datablock());
		StereoVisionApp::Image* im1 = qobject_cast<StereoVisionApp::Image*>(img1.datablock());
		StereoVisionApp::Image* im2 = qobject_cast<StereoVisionApp::Image*>(img2.datablock());

		if (rig == nullptr or im1 == nullptr or im2 == nullptr) {
			py::print("One of the provided datablock is invalid!");
			return;
		}

		rig->insertImagePair(im1->internalId(), im2->internalId());

	}

	void detectHexagonalTargets(DataBlockReference img,
								double minThreshold = 80,
								double diffThreshold = 30,
								int minArea = 10,
								int maxArea = 800,
								double minToMaxAxisRatioThreshold = 0.6,
								double hexRelMaxDiameter = 0.2,
								double hexFirRelMaxRes = 0.1,
								double redGain = 1.1,
								double greenGain = 1.1,
								double blueGain = 1.0,
								bool clearPrevious = false,
								bool useHexScale = false,
								float hexEdge = 90.016) {

		StereoVisionApp::Image* image = qobject_cast<StereoVisionApp::Image*>(img.datablock());

		if (image == nullptr) {
			py::print("Invalid image reference provided");
			return;
		}

		StereoVisionApp::detectHexagonalTargets(image,
												minThreshold,
												diffThreshold,
												minArea,
												maxArea,
												minToMaxAxisRatioThreshold,
												hexRelMaxDiameter,
												hexFirRelMaxRes,
												redGain,
												greenGain,
												blueGain,
												clearPrevious,
												useHexScale,
												hexEdge);

	}

	void alignHexagonalTargetsToImage(DataBlockReference img) {

		StereoVisionApp::Image* image = qobject_cast<StereoVisionApp::Image*>(img.datablock());

		if (image == nullptr) {
			py::print("Invalid image reference provided!");
			return;
		}

		StereoVisionApp::orientHexagonalTargetsRelativeToCamera(image->internalId(), image->getProject());

	}

	void alignImageToObservedLandmarks(DataBlockReference img) {

		StereoVisionApp::Image* image = qobject_cast<StereoVisionApp::Image*>(img.datablock());

		if (image == nullptr) {
			py::print("Invalid image reference provided!");
			return;
		}

		StereoVisionApp::orientCamerasRelativeToObservedLandmarks(image->internalId(), image->getProject());

	}

	void alignImageUsingStereoRig(DataBlockReference img, DataBlockReference stereoRig) {

		StereoVisionApp::Image* image = qobject_cast<StereoVisionApp::Image*>(img.datablock());

		if (image == nullptr) {
			py::print("Invalid image reference provided!");
			return;
		}

		StereoVisionApp::StereoRig* rig = qobject_cast<StereoVisionApp::StereoRig*>(stereoRig.datablock());

		if (rig == nullptr) {
			py::print("Invalid stereo rig reference provided!");
			return;
		}

		StereoVisionApp::ImagePair* pair = rig->getPairForImage(image->internalId());

		if (pair == nullptr) {
			py::print("Provided image is not in provided stereo rig!");
			return;
		}

		qint64 ref_image_id = (pair->idImgCam1() == image->internalId()) ? pair->idImgCam2() : pair->idImgCam1();
		qint64 unaligned_image_id = image->internalId();

		StereoVisionApp::alignImagesInRig(rig->getProject(), rig->internalId(), ref_image_id, unaligned_image_id);

	}

	void refineGlobalSolution(int nIterations,
							  bool useCurrentSolutionAtStart = true,
							  bool useSparseOptimizer = true,
							  bool predictUncertainty = false) {

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

		StereoVisionApp::solveSparseHeadless(proj,
											 useCurrentSolutionAtStart,
											 useSparseOptimizer,
											 predictUncertainty,
											 nIterations,
											 StereoVisionApp::FixedParameter::NoFixedParameters);

	}

	void alignLocalCoordinateSystems(std::vector<DataBlockReference> references) {

		for (DataBlockReference & datablock: references) {
			StereoVisionApp::LocalCoordinateSystem* lcs = qobject_cast<StereoVisionApp::LocalCoordinateSystem*>(datablock.datablock());

			if (lcs != nullptr) {
				StereoVisionApp::alignLocalCoordinateSystemToPoints({lcs->internalId()}, lcs->getProject());
			}
		}

	}

	void printDataBlocks() const;

private:

	std::string _app_argv;
	std::array<char*, 2> _argv_ptrs;
	StereoVisionApp::StereoVisionApplication* _instancedApp;

};

PythonStereoVisionApp::PythonStereoVisionApp(std::string const& appLocation, bool headless):
	_instancedApp(nullptr)
{

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

	py::class_<DataBlockReference>(m, "DatablockReference")
			.def_property_readonly("isValid", &DataBlockReference::isValid)
			.def_property("fixed", &DataBlockReference::isFixed, &DataBlockReference::setFixed)
			.def_property("name", &DataBlockReference::name, &DataBlockReference::setObjectName);

	py::class_<PythonStereoVisionApp>(m, "AppInstance")
			.def(py::init<const std::string &, bool>(), py::arg("appLocation") = "", py::arg("headless") = true)
			.def("openProject", &PythonStereoVisionApp::openProject, py::arg("path"))
			.def("saveProject", &PythonStereoVisionApp::saveProject)
			.def("saveProjectAs", &PythonStereoVisionApp::saveProjectAs, py::arg("path"))
			.def("getDatablockByName", &PythonStereoVisionApp::getDataBlockByName, py::arg("name"), py::arg("typeHint") = "")
			.def("addImage", &PythonStereoVisionApp::addImage, py::arg("filename"), py::arg("imgname"))
			.def("addImage", &PythonStereoVisionApp::addImageWithCam, py::arg("filename"), py::arg("imgname"), py::arg("cam"))
			.def("addImages", &PythonStereoVisionApp::addImages, py::arg("filenames"))
			.def("addLandmark", &PythonStereoVisionApp::addDatablock<StereoVisionApp::Landmark>, py::arg("name"))
			.def("addCamera", &PythonStereoVisionApp::addDatablock<StereoVisionApp::Camera>, py::arg("name"))
			.def("addStereoRig", &PythonStereoVisionApp::addDatablock<StereoVisionApp::StereoRig>, py::arg("name"))
			.def("importCameraData", &PythonStereoVisionApp::configureDatablockDataFromJson<StereoVisionApp::Camera>, py::arg("datablock"), py::arg("filepath"))
			.def("importStereoRigData", &PythonStereoVisionApp::configureDatablockDataFromJson<StereoVisionApp::StereoRig>, py::arg("datablock"), py::arg("filepath"))
			.def("setImageAsReference", &PythonStereoVisionApp::setImageAsReference, py::arg("image"))
			.def("assignImagesToCamera", &PythonStereoVisionApp::assignImagesToCamera, py::arg("images"), py::arg("camera"))
			.def("assignImagesToStereoRig", &PythonStereoVisionApp::assignImagesToStereoRig, py::arg("stereorig"), py::arg("image1"), py::arg("image2"))
			.def("detectHexagonalTargets", &PythonStereoVisionApp::detectHexagonalTargets,
				 py::arg("image"),
				 py::arg("minThreshold") = 80,
				 py::arg("diffThreshold") = 30,
				 py::arg("minArea") = 10,
				 py::arg("maxArea") = 800,
				 py::arg("minToMaxAxisRatioThreshold") = 0.6,
				 py::arg("hexRelMaxDiameter") = 0.2,
				 py::arg("hexFirRelMaxRes") = 0.1,
				 py::arg("redGain") = 1.1,
				 py::arg("greenGain") = 1.1,
				 py::arg("blueGain") = 1.0,
				 py::arg("clearPrevious") = false,
				 py::arg("useHexScale") = false,
				 py::arg("hexEdge") = 90.016)
			.def("alignHexagonalTargetsToImage", &PythonStereoVisionApp::alignHexagonalTargetsToImage, py::arg("image"))
			.def("alignImageToObservedLandmarks", &PythonStereoVisionApp::alignImageToObservedLandmarks, py::arg("image"))
			.def("alignImageUsingStereoRig", &PythonStereoVisionApp::alignImageUsingStereoRig, py::arg("image"), py::arg("rig"))
			.def("refineGlobalSolution", &PythonStereoVisionApp::refineGlobalSolution,
				 py::arg("nIterations") = 5,
				 py::arg("useCurrentSolutionAtStart") = true,
				 py::arg("useSparseOptimizer") = true,
				 py::arg("predictUncertainty") = false)
			.def("alignLocalCoordinateSystems", &PythonStereoVisionApp::alignLocalCoordinateSystems, py::arg("localCoordinateSystems"))
			.def("printDatablocks", &PythonStereoVisionApp::printDataBlocks);

}

#include "python_api.moc"
