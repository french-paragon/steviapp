#include "projectactions.h"

#include "datablocks/project.h"
#include "datablocks/georeferenceddatablockinterface.h"

#include <limits>

namespace StereoVisionApp {

StereoVision::Geometry::AffineTransform<float> getLocalFrameOnSphere(Eigen::Vector3f mean, float minDist) {

    Eigen::Vector3f direction = mean.normalized();
    Eigen::Vector3f origin = direction*minDist;

    Eigen::Vector3f vertical = Eigen::Vector3f(0,0,1);

    if (direction.z() < 0) {
        vertical = -vertical;
    }

    float scaling = vertical.dot(direction);

    if (scaling < 1e-3) {
        Eigen::Vector3f toNorth = Eigen::Vector3f(0,0,1);
        direction.z() = 0;
        direction.normalize();
        Eigen::Vector3f toEast = toNorth.cross(direction);

        Eigen::Matrix<float,3,3> Rlocal2ecef;
        Rlocal2ecef.block<3,1>(0,0) = toEast;
        Rlocal2ecef.block<3,1>(0,1) = toNorth;
        Rlocal2ecef.block<3,1>(0,2) = direction;

        Eigen::Matrix<float,3,3> Recef2local = Rlocal2ecef.transpose();

        StereoVision::Geometry::AffineTransform<float>(Recef2local, -Recef2local*origin);
    }

    vertical /= scaling; //scale to form a right triangle perpendicular to the sphere
    Eigen::Vector3f toNorth = vertical - direction;

    if (direction.z() < 0) {
        toNorth = -toNorth;
    }

    if (toNorth.norm()*minDist < 1) {
        //put a local system at the poles
        return StereoVision::Geometry::AffineTransform<float>(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0,0,-minDist));
    }

    toNorth.normalize();
    Eigen::Vector3f toEast = toNorth.cross(direction);

    Eigen::Matrix<float,3,3> Rlocal2ecef;

    Rlocal2ecef.block<3,1>(0,0) = toEast;
    Rlocal2ecef.block<3,1>(0,1) = toNorth;
    Rlocal2ecef.block<3,1>(0,2) = direction;

    Eigen::Matrix<float,3,3> Recef2local = Rlocal2ecef.transpose();

    return StereoVision::Geometry::AffineTransform<float>(Recef2local, -Recef2local*origin);

}

bool estimateLocalCoordinateSystem(Project* p) {

    if (p == nullptr) {
        return false;
    }

    int count = 0;
    float minDist = std::numeric_limits<float>::infinity();

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();

    for (qint64 block_id : p->getIds()) {

        DataBlock* block = p->getById(block_id);

        GeoReferencedDataBlockInterface* geoBlock = qobject_cast<StereoVisionApp::GeoReferencedDataBlockInterface*>(block);

        if (geoBlock != nullptr) {

            if (!geoBlock->geoReferenceSupportActive()) {
                continue;
            }

            Eigen::Array<float, 3, Eigen::Dynamic> points = geoBlock->getLocalPointsEcef();

            for (int c = 0; c < points.cols(); c++) {
                Eigen::Vector3f pt = points.col(c);

                float dist = pt.norm();

                mean += pt;
                count++;

                if (dist < minDist) {
                    minDist = dist;
                }
            }
        }

    }

    if (count == 0) {
        return false;
    }

    mean /= count;

    p->setLocalCoordinateFrame(getLocalFrameOnSphere(mean, minDist));

    return true;
}

} //namespace StereoVisionApp
