#include "localcoordinatesystemactions.h"

#include <datablocks/project.h>
#include <datablocks/localcoordinatesystem.h>
#include <datablocks/landmark.h>

#include <StereoVision/geometry/pointcloudalignment.h>

#include <QDebug>
#include <iostream>

int StereoVisionApp::alignLocalCoordinateSystemToPoints(QList<qint64> lcsIds, Project* p) {

	int processedCount = 0;

	for (qint64 id : lcsIds) {

		LocalCoordinateSystem* lcs = p->getDataBlock<LocalCoordinateSystem>(id);

		if (lcs == nullptr) {
			continue;
		}

		QVector<qint64> lmlcids = lcs->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className());
		QVector<LandmarkLocalCoordinates*> observedPts;
		observedPts.reserve(lmlcids.size());

		for (qint64 lmlcid : lmlcids) {

			LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinates(lmlcid);

			if (!lmlc->xCoord().isSet() or !lmlc->yCoord().isSet() or !lmlc->zCoord().isSet()) {
				continue;
			}

			Landmark* lm = lmlc->attachedLandmark();

			if (lm == nullptr) {
				continue;
			}

			bool hasPosMeasure = lm->optPos().isSet();

			if (!hasPosMeasure) {
				continue;
			}

			observedPts.push_back(lmlc);

		}

		if (observedPts.size() < 5) {
			qDebug() << "Skipping coordinate system " << lcs->objectName() << " due to lack of observed points";
			continue;
		}

		Eigen::Matrix3Xf localCoordinates;
		localCoordinates.resize(3,observedPts.size());

		Eigen::Matrix3Xf worldCoordinates;
		worldCoordinates.resize(3,observedPts.size());

		Eigen::VectorXf obs;
		obs.resize(3*observedPts.size());

		std::vector<int> idxs;
		idxs.resize(observedPts.size()*3);

		std::vector<StereoVision::Geometry::Axis> coordinates;
		coordinates.resize(observedPts.size()*3);

		for (int i = 0; i < observedPts.size(); i++) {
			LandmarkLocalCoordinates* lmlc = observedPts[i];
			Landmark* lm = lmlc->attachedLandmark();

			localCoordinates(0,i) = lmlc->xCoord().value();
			localCoordinates(1,i) = lmlc->yCoord().value();
			localCoordinates(2,i) = lmlc->zCoord().value();

            worldCoordinates(0,i) = lm->optXCoord().value();
            worldCoordinates(1,i) = lm->optYCoord().value();
            worldCoordinates(2,i) = lm->optZCoord().value();

            obs[3*i] = lm->optXCoord().value();
			idxs[3*i] = i;
			coordinates[3*i] = StereoVision::Geometry::Axis::X;

            obs[3*i+1] = lm->optYCoord().value();
			idxs[3*i+1] = i;
			coordinates[3*i+1] = StereoVision::Geometry::Axis::Y;

            obs[3*i+2] = lm->optZCoord().value();
			idxs[3*i+2] = i;
			coordinates[3*i+2] = StereoVision::Geometry::Axis::Z;
		}

		StereoVision::Geometry::AffineTransform<float> localToWorldRaw;
		StereoVision::Geometry::ShapePreservingTransform<float> localToWorld;

		int iterationLimit = 500;
		float damping = 2e-1;
		float incrLimit = 1e-4;
		bool verbose = true;

		StereoVision::Geometry::IterativeTermination termination_status;

		//TODO: use a function that is invariant to scale
		localToWorldRaw = StereoVision::Geometry::estimateQuasiRigidMap(obs,
																		localCoordinates,
																		idxs,
																		coordinates,
																		damping,
																		&termination_status,
																		incrLimit,
																		iterationLimit,
																		verbose);

		std::cout << "Local to world raw Rotation = " << localToWorldRaw.R << std::endl;
		std::cout << "Local to world raw translation = " << localToWorldRaw.t << std::endl;

		std::cout << (localToWorldRaw*localCoordinates - worldCoordinates) << std::endl;

		localToWorld = StereoVision::Geometry::affine2ShapePreservingMap(localToWorldRaw);

		std::cout << "Local to world scale = " << localToWorld.s << std::endl;
		std::cout << "Local to world rotation = " << StereoVision::Geometry::rodriguezFormula(localToWorld.r) << std::endl;
		std::cout << "Local to world translation = " << localToWorld.t << std::endl;

		std::cout << (localToWorld*localCoordinates - worldCoordinates) << std::endl;

		localToWorld.s = 1;

		std::cout << (localToWorld*localCoordinates - worldCoordinates) << std::endl;


		floatParameterGroup<3> oPos;
		oPos.value(0) = localToWorld.t.x();
		oPos.value(1) = localToWorld.t.y();
		oPos.value(2) = localToWorld.t.z();
		oPos.setIsSet();

		floatParameterGroup<3> oRot;
		oRot.value(0) = localToWorld.r.x();
		oRot.value(1) = localToWorld.r.y();
		oRot.value(2) = localToWorld.r.z();
		oRot.setIsSet();

		lcs->setOptPos(oPos);
		lcs->setOptRot(oRot);

		processedCount++;
	}

	return processedCount;
}
