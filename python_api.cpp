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
#include "datablocks/correspondencesset.h"

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

    void newProject();
    void openProject(std::string const& path);
	void saveProject();
	void saveProjectAs(std::string const& path);

    DataBlockReference addPlaceholderImage(std::string const& name, DataBlockReference camera) {

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

        qint64 id = StereoVisionApp::addPlaceholderImage(QString::fromStdString(name), proj, cam);

        if (id < 0) {
            return DataBlockReference();
        }

        StereoVisionApp::Image* img = proj->getDataBlock<StereoVisionApp::Image>(id);

        return DataBlockReference(img);

    }

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
            py::print("One of the provided datablocks is invalid!");
			return;
		}

		rig->insertImagePair(im1->internalId(), im2->internalId());

	}

    void assignLandmarkToImage(DataBlockReference landmark,
                                DataBlockReference img,
                                std::array<double, 2> projectionPos,
                                bool uncertain = false,
                                double sigma_pos = 1.0) {

        StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(landmark.datablock());
        StereoVisionApp::Image* im = qobject_cast<StereoVisionApp::Image*>(img.datablock());

        if (lm == nullptr or im == nullptr) {
            py::print("One of the provided datablocks is invalid!");
            return;
        }

        QPointF point(projectionPos[0], projectionPos[1]);
        im->addImageLandmark(point, lm->internalId(), uncertain, sigma_pos);
    }

    void assignLandmarkToLocalSystem(DataBlockReference landmark,
                                     DataBlockReference localSystem,
                                     std::array<double, 3> localLmPos,
                                     bool uncertain = false,
                                     double sigma_pos = 1.0) {

        StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(landmark.datablock());
        StereoVisionApp::LocalCoordinateSystem* ls = qobject_cast<StereoVisionApp::LocalCoordinateSystem*>(localSystem.datablock());

        if (lm == nullptr or ls == nullptr) {
            py::print("One of the provided datablocks is invalid!");
            return;
        }

        StereoVisionApp::floatParameter priorX(localLmPos[0], true);
        StereoVisionApp::floatParameter priorY(localLmPos[1], true);
        StereoVisionApp::floatParameter priorZ(localLmPos[2], true);

        if (uncertain) {
            priorX.setUncertainty(sigma_pos);
            priorY.setUncertainty(sigma_pos);
            priorZ.setUncertainty(sigma_pos);
        }

        ls->addLandmarkLocalCoordinates(lm->internalId(), priorX, priorY, priorZ);

    }

    void assignLandmarkPosition(DataBlockReference landmark,
                                std::array<double, 3> lmPos,
                                bool uncertain = false,
                                double sigma_pos = 1.0,
                                bool optimized = false) {

             StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(landmark.datablock());

             if (lm == nullptr) {
                 py::print("One of the provided datablocks is invalid!");
                 return;
             }

             if (optimized) {

                 StereoVisionApp::floatParameterGroup<3> optPos;

                 optPos.setIsSet();

                 optPos.value(0) = lmPos[0];
                 optPos.value(1) = lmPos[1];
                 optPos.value(2) = lmPos[2];

                 if (uncertain) {
                     optPos.setUncertain();
                     optPos.stddev(0,0) = sigma_pos;
                     optPos.stddev(1,1) = sigma_pos;
                     optPos.stddev(2,2) = sigma_pos;
                 }

                 lm->setOptPos(optPos);

             } else {

                 StereoVisionApp::floatParameter priorX(lmPos[0], true);
                 StereoVisionApp::floatParameter priorY(lmPos[1], true);
                 StereoVisionApp::floatParameter priorZ(lmPos[2], true);

                 if (uncertain) {
                     priorX.setUncertainty(sigma_pos);
                     priorY.setUncertainty(sigma_pos);
                     priorZ.setUncertainty(sigma_pos);
                 }

                 lm->setXCoord(priorX);
                 lm->setYCoord(priorY);
                 lm->setZCoord(priorZ);
             }

         }

    void addImg2ImgCorrespondence(DataBlockReference correspondanceSet,
                                  DataBlockReference img1,
                                  std::array<double, 2> img1Pos,
                                  DataBlockReference img2,
                                  std::array<double, 2> img2Pos,
                                  bool uncertain = false,
                                  double sigma_pos = 1.0) {

        StereoVisionApp::CorrespondencesSet* cs = qobject_cast<StereoVisionApp::CorrespondencesSet*>(correspondanceSet.datablock());
        StereoVisionApp::Image* im1 = qobject_cast<StereoVisionApp::Image*>(img1.datablock());
        StereoVisionApp::Image* im2 = qobject_cast<StereoVisionApp::Image*>(img2.datablock());

        if (cs == nullptr or im1 == nullptr or im2 == nullptr) {
            py::print("One of the provided datablocks is invalid!");
            return;
        }

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::UV> cimg1;
        cimg1.blockId = im1->internalId();
        cimg1.u = img1Pos[0];
        cimg1.v = img1Pos[1];
        cimg1.sigmaU = sigma_pos;
        cimg1.sigmaV = sigma_pos;

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::UV> cimg2;
        cimg2.blockId = im2->internalId();
        cimg2.u = img2Pos[0];
        cimg2.v = img2Pos[1];
        cimg2.sigmaU = sigma_pos;
        cimg2.sigmaV = sigma_pos;

        StereoVisionApp::Correspondences::GenericPair correspondence;
        correspondence.c1 = cimg1;
        correspondence.c2 = cimg2;

        cs->addCorrespondence(correspondence);

    }

    void addLcs2LcsCorrespondence(DataBlockReference correspondanceSet,
                                  DataBlockReference lcs1,
                                  std::array<double, 3> lcs1Pos,
                                  DataBlockReference lcs2,
                                  std::array<double, 3> lcs2Pos,
                                  bool uncertain = false,
                                  double sigma_pos = 1.0) {

        StereoVisionApp::CorrespondencesSet* cs = qobject_cast<StereoVisionApp::CorrespondencesSet*>(correspondanceSet.datablock());
        StereoVisionApp::LocalCoordinateSystem* lc1 = qobject_cast<StereoVisionApp::LocalCoordinateSystem*>(lcs1.datablock());
        StereoVisionApp::LocalCoordinateSystem* lc2 = qobject_cast<StereoVisionApp::LocalCoordinateSystem*>(lcs2.datablock());

        if (cs == nullptr or lc1 == nullptr or lc2 == nullptr) {
            py::print("One of the provided datablocks is invalid!");
            return;
        }

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::XYZ> clcs1;
        clcs1.blockId = lc1->internalId();
        clcs1.x = lcs1Pos[0];
        clcs1.y = lcs1Pos[1];
        clcs1.z = lcs1Pos[2];
        clcs1.sigmaX = sigma_pos;
        clcs1.sigmaY = sigma_pos;
        clcs1.sigmaZ = sigma_pos;

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::XYZ> clcs2;
        clcs2.blockId = lc1->internalId();
        clcs2.x = lcs2Pos[0];
        clcs2.y = lcs2Pos[1];
        clcs2.z = lcs2Pos[2];
        clcs2.sigmaX = sigma_pos;
        clcs2.sigmaY = sigma_pos;
        clcs2.sigmaZ = sigma_pos;

        StereoVisionApp::Correspondences::GenericPair correspondence;
        correspondence.c1 = clcs1;
        correspondence.c2 = clcs2;

        cs->addCorrespondence(correspondence);

    }

    void addLcs2ImgCorrespondence(DataBlockReference correspondanceSet,
                                  DataBlockReference lcs,
                                  std::array<double, 3> lcsPos,
                                  DataBlockReference img,
                                  std::array<double, 2> imgPos,
                                  bool uncertain = false,
                                  double sigma_pos_lcs = 1.0,
                                  double sigma_pos_img = 1.0) {

        StereoVisionApp::CorrespondencesSet* cs = qobject_cast<StereoVisionApp::CorrespondencesSet*>(correspondanceSet.datablock());
        StereoVisionApp::LocalCoordinateSystem* lc = qobject_cast<StereoVisionApp::LocalCoordinateSystem*>(lcs.datablock());
        StereoVisionApp::Image* im = qobject_cast<StereoVisionApp::Image*>(img.datablock());

        if (cs == nullptr or lc == nullptr or im == nullptr) {
            py::print("One of the provided datablocks is invalid!");
            return;
        }

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::XYZ> clcs;
        clcs.blockId = lc->internalId();
        clcs.x = lcsPos[0];
        clcs.y = lcsPos[1];
        clcs.z = lcsPos[2];
        clcs.sigmaX = sigma_pos_lcs;
        clcs.sigmaY = sigma_pos_lcs;
        clcs.sigmaZ = sigma_pos_lcs;

        StereoVisionApp::Correspondences::Typed<StereoVisionApp::Correspondences::UV> cimg;
        cimg.blockId = im->internalId();
        cimg.u = imgPos[0];
        cimg.v = imgPos[1];
        cimg.sigmaU = sigma_pos_img;
        cimg.sigmaV = sigma_pos_img;

        StereoVisionApp::Correspondences::GenericPair correspondence;
        correspondence.c1 = clcs;
        correspondence.c2 = cimg;

        cs->addCorrespondence(correspondence);

    }

    void addGenericCorrespondence(DataBlockReference correspondanceSet,
                                  std::string const& correspondenceDescr) {
        StereoVisionApp::CorrespondencesSet* cs = qobject_cast<StereoVisionApp::CorrespondencesSet*>(correspondanceSet.datablock());

        if (cs == nullptr) {
            py::print("The provided datablock is invalid!");
            return;
        }

        auto opt = StereoVisionApp::Correspondences::GenericPair::fromString(QString::fromStdString(correspondenceDescr));

        const char* error_msg = "Invalid correspondence description provided!";

        if (!opt.has_value()) {
            py::print(error_msg);
            return;
        }

        if (!opt->isValid()) {
            py::print(error_msg);
            return;
        }

        cs->addCorrespondence(opt.value());
    }

    void setRigidBodyPose(DataBlockReference rigidBody,
                          std::array<double, 3> position,
                          std::array<double, 3> rotationAxis,
                          bool uncertain = false,
                          double position_sigma = 1.0,
                          double rotation_sigma = 1.0,
                          bool isOptimized = false) {

        StereoVisionApp::RigidBody* rb = qobject_cast<StereoVisionApp::RigidBody*>(rigidBody.datablock());

        if (rb == nullptr) {
            py::print("Provided datablock is invalid!");
            return;
        }

        if (isOptimized) {
            StereoVisionApp::floatParameterGroup<3> optPos;
            optPos.value(0) = position[0];
            optPos.value(1) = position[1];
            optPos.value(2) = position[2];
            optPos.setIsSet();

            StereoVisionApp::floatParameterGroup<3> optRot;
            optRot.value(0) = rotationAxis[0];
            optRot.value(1) = rotationAxis[1];
            optRot.value(2) = rotationAxis[2];
            optRot.setIsSet();

            rb->setOptPos(optPos);
            rb->setOptRot(optRot);

            return;
        }

        StereoVisionApp::floatParameter xPos(position[0]);
        StereoVisionApp::floatParameter yPos(position[1]);
        StereoVisionApp::floatParameter zPos(position[2]);

        StereoVisionApp::floatParameter xRot(rotationAxis[0]);
        StereoVisionApp::floatParameter yRot(rotationAxis[1]);
        StereoVisionApp::floatParameter zRot(rotationAxis[2]);

        if (uncertain) {

            xPos.setUncertainty(position_sigma);
            yPos.setUncertainty(position_sigma);
            zPos.setUncertainty(position_sigma);

            xRot.setUncertainty(rotation_sigma);
            yRot.setUncertainty(rotation_sigma);
            zRot.setUncertainty(rotation_sigma);
        }

        rb->setXCoord(xPos);
        rb->setYCoord(yPos);
        rb->setZCoord(zPos);

        rb->setXRot(xRot);
        rb->setYRot(yRot);
        rb->setZRot(zRot);
    }

    void setCameraIntrisicParameters(DataBlockReference camera,
                                     double fLen,
                                     std::array<double, 2> pp,
                                     std::array<int, 2> size,
                                     std::array<double, 3> k,
                                     std::array<double, 2> p,
                                     std::array<double, 2> B,
                                     bool isOptimized = false) {

        StereoVisionApp::Camera* cam = qobject_cast<StereoVisionApp::Camera*>(camera.datablock());

        if (cam == nullptr) {
            py::print("Provided datablock is invalid!");
            return;
        }
        StereoVisionApp::floatParameter fLenParam(fLen, true);

        StereoVisionApp::floatParameter pp1(pp[0], true);
        StereoVisionApp::floatParameter pp2(pp[1], true);

        StereoVisionApp::floatParameter k1(k[0], true);
        StereoVisionApp::floatParameter k2(k[1], true);
        StereoVisionApp::floatParameter k3(k[2], true);

        StereoVisionApp::floatParameter p1(p[0], true);
        StereoVisionApp::floatParameter p2(p[1], true);

        StereoVisionApp::floatParameter B1(B[0], true);
        StereoVisionApp::floatParameter B2(B[1], true);

        cam->setImWidth(size[0]);
        cam->setImHeight(size[1]);

        if (isOptimized) {
            cam->setOptimizedFLen(fLenParam);

            cam->setOptimizedOpticalCenterX(pp1);
            cam->setOptimizedOpticalCenterY(pp2);

            cam->setOptimizedK1(k1);
            cam->setOptimizedK2(k2);
            cam->setOptimizedK3(k3);

            cam->setOptimizedP1(p1);
            cam->setOptimizedP2(p2);

            cam->setOptimizedB1(B1);
            cam->setOptimizedB2(B2);

        } else {
            cam->setFLen(fLenParam);

            cam->setOpticalCenterX(pp1);
            cam->setOpticalCenterY(pp2);

            cam->setK1(k1);
            cam->setK2(k2);
            cam->setK3(k3);

            cam->setP1(p1);
            cam->setP2(p2);

            cam->setB1(B1);
            cam->setB2(B2);
        }
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


void PythonStereoVisionApp::newProject() {
    StereoVisionApp::StereoVisionApplication* app = getApp();

    if (app == nullptr) {
        py::print("Ho no, cannot access a valid app instance !");
        return;
    }

    app->newEmptyProject();
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
            .def("newProject", &PythonStereoVisionApp::newProject)
			.def("openProject", &PythonStereoVisionApp::openProject, py::arg("path"))
			.def("saveProject", &PythonStereoVisionApp::saveProject)
			.def("saveProjectAs", &PythonStereoVisionApp::saveProjectAs, py::arg("path"))
            .def("getDatablockByName", &PythonStereoVisionApp::getDataBlockByName, py::arg("name"), py::arg("typeHint") = "")
            .def("addPlaceholderImage", &PythonStereoVisionApp::addPlaceholderImage, py::arg("name"), py::arg("cam"))
            .def("addImage", &PythonStereoVisionApp::addImage, py::arg("filename"), py::arg("imgname"))
			.def("addImage", &PythonStereoVisionApp::addImageWithCam, py::arg("filename"), py::arg("imgname"), py::arg("cam"))
			.def("addImages", &PythonStereoVisionApp::addImages, py::arg("filenames"))
            .def("addLandmark", &PythonStereoVisionApp::addDatablock<StereoVisionApp::Landmark>, py::arg("name"))
            .def("addCamera", &PythonStereoVisionApp::addDatablock<StereoVisionApp::Camera>, py::arg("name"))
            .def("addLocalCoordinateSystem", &PythonStereoVisionApp::addDatablock<StereoVisionApp::LocalCoordinateSystem>, py::arg("name"))
            .def("addCorrespondencesSet", &PythonStereoVisionApp::addDatablock<StereoVisionApp::CorrespondencesSet>, py::arg("name"))
            .def("assignLandmarkToImage", &PythonStereoVisionApp::assignLandmarkToImage,
                 py::arg("landmark"),
                 py::arg("image"),
                 py::arg("projectionPos"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos") = 1.0)
            .def("assignLandmarkToLocalSystem", &PythonStereoVisionApp::assignLandmarkToLocalSystem,
                 py::arg("landmark"),
                 py::arg("localSystem"),
                 py::arg("localPos"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos") = 1.0)
            .def("assignLandmarkPosition", &PythonStereoVisionApp::assignLandmarkPosition,
                 py::arg("landmark"),
                 py::arg("localPos"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos") = 1.0,
                 py::arg("isOptimized") = false)
            .def("addImg2ImgCorrespondence", &PythonStereoVisionApp::addImg2ImgCorrespondence,
                 py::arg("correspondencesSet"),
                 py::arg("img1"),
                 py::arg("posImg1"),
                 py::arg("img2"),
                 py::arg("posImg2"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos") = 1.0)
            .def("addLcs2LcsCorrespondence", &PythonStereoVisionApp::addLcs2LcsCorrespondence,
                 py::arg("correspondencesSet"),
                 py::arg("lcs1"),
                 py::arg("posLcs1"),
                 py::arg("lcs2"),
                 py::arg("posLcs2"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos") = 1.0)
            .def("addLcs2ImgCorrespondence", &PythonStereoVisionApp::addLcs2ImgCorrespondence,
                 py::arg("correspondencesSet"),
                 py::arg("lcs"),
                 py::arg("posLcs"),
                 py::arg("img"),
                 py::arg("posImg"),
                 py::arg("uncertain") = false,
                 py::arg("sigma_pos_lcs") = 1.0,
                 py::arg("sigma_pos_img") = 1.0)
            .def("addGenericCorrespondence", &PythonStereoVisionApp::addGenericCorrespondence,
                 py::arg("correspondencesSet"),
                 py::arg("correspondenceDescr"))
            .def("setRigidBodyPose", &PythonStereoVisionApp::setRigidBodyPose,
                 py::arg("rigidBody"),
                 py::arg("position"),
                 py::arg("rotationAxis"),
                 py::arg("uncertain") = false,
                 py::arg("sigmaPosition") = 1.0,
                 py::arg("sigmaRotation") = 1.0,
                 py::arg("isOptimized") = false)
            .def("setCameraIntrisicParameters", &PythonStereoVisionApp::setCameraIntrisicParameters,
                 py::arg("camera"),
                 py::arg("fLen"),
                 py::arg("pp"),
                 py::arg("size"),
                 py::arg("k") = std::array<double, 3>{0,0,0},
                 py::arg("p") = std::array<double, 2>{0,0},
                 py::arg("B") = std::array<double, 2>{0,0},
                 py::arg("isOptimized") = false)
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
