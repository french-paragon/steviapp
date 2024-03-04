#ifndef TERRAINPROJECTOR_H
#define TERRAINPROJECTOR_H

#include "./geoRaster.h"

#include <MultidimArrays/MultidimArrays.h>
#include <MultidimArrays/MultidimIndexManipulators.h>

#include <Eigen/Core>
#include <Eigen/QR>

#include <StereoVision/geometry/rotations.h>

#include <proj.h>

namespace StereoVisionApp {
namespace Geo {

/*!
 * \brief The TerrainProjector class is used to project vectors from a flight on a terrain.
 */
template<typename TerrainT>
class TerrainProjector {

public:
    TerrainProjector(GeoRasterData<TerrainT,2> const& terrain) :
        _terrain(terrain)
    {

        _invGeoTransform.template block<2,2>(0,0) = Eigen::Matrix2d(terrain.geoTransform.template block<2,2>(0,0)).inverse();
        _invGeoTransform.template block<2,1>(0,2) = -_invGeoTransform.template block<2,2>(0,0)*terrain.geoTransform.template block<2,1>(0,2);

        _cover = Multidim::Array<bool, 2>(terrain.raster.shape());
        clearCover();

        //reprojection
        int nPoints = terrain.raster.shape()[0]*terrain.raster.shape()[1];

        typename Multidim::Array<double,3>::ShapeBlock shape = {terrain.raster.shape()[0], terrain.raster.shape()[1], 3};
        typename Multidim::Array<double,3>::ShapeBlock strides = {terrain.raster.shape()[1]*3, 3, 1}; //ensure each triplets of floats is a point.

        typename Multidim::Array<double,3> coordinates_vecs(shape, strides);

        _minTerrainHeight = _terrain.raster.valueUnchecked(0,0);
        _maxTerrainHeight = _terrain.raster.valueUnchecked(0,0);

        for (int i = 0; i < shape[0]; i++) {
            for (int j = 0; j < shape[1]; j++) {

                Eigen::Vector3d homogeneousImgCoord(j,i,1);
                Eigen::Vector2d geoCoord = terrain.geoTransform*homogeneousImgCoord;

                coordinates_vecs.atUnchecked(i,j,0) = geoCoord.y();
                coordinates_vecs.atUnchecked(i,j,1) = geoCoord.x();

                TerrainT terrainVal = _terrain.raster.valueUnchecked(i,j);
                coordinates_vecs.atUnchecked(i,j,2) = terrainVal;

                _minTerrainHeight = std::min(terrainVal, _minTerrainHeight);
                _maxTerrainHeight = std::max(terrainVal, _maxTerrainHeight);

            }
        }

        _ctx = proj_context_create();

        const char* wgs84_ecef = "EPSG:4978"; //The WGS84 earth centered, earth fixed frame used for projection on the terrain

        _projector = proj_create_crs_to_crs(_ctx, wgs84_ecef, terrain.crsInfos.c_str(), nullptr);
        _reprojector = proj_create_crs_to_crs(_ctx, terrain.crsInfos.c_str(), wgs84_ecef, nullptr);

        if (_projector == nullptr) { //in case of error
            return;
        }

        if (_reprojector == nullptr) { //in case of error
            return;
        }

        //Build the ECEF lattice

        proj_trans_generic(_reprojector, PJ_FWD,
                           &coordinates_vecs.atUnchecked(0,0,0), 3*sizeof(double), nPoints,
                           &coordinates_vecs.atUnchecked(0,0,1), 3*sizeof(double), nPoints,
                           &coordinates_vecs.atUnchecked(0,0,2), 3*sizeof(double), nPoints,
                           nullptr,0,0); //reproject to ecef coordinates

        _ecefTerrain = coordinates_vecs.cast<float>();

    }

    ~TerrainProjector() {

        if (_projector != nullptr) {
            proj_destroy(_projector);
        }

        if (_reprojector != nullptr) {
            proj_destroy(_reprojector);
        }

        if (_ctx != nullptr) {
            proj_context_destroy(_ctx);
        }
    }

    bool isValid() const {
        return _reprojector != nullptr and _projector != nullptr;
    }

    struct ProjectionResults {
        std::array<float,2> originMapPos;
        std::vector<std::array<float,2>> projectedPoints;
    };

    /*!
     * \brief projectVectors, project a set of vectors on the terrain
     * \param ecefOrigin the position the vectors are projected from.
     * \param directions the projection direction
     * \return a structure with the map coordinate of the center of projection and the different projected coordinates (in image space of the terrain model).
     *
     * This function bruteforce the projection of the rays on the terrain, starting the search from the down position for each vector.
     *
     */
    std::optional<ProjectionResults> projectVectors(std::array<double,3> const& ecefOrigin,
                                                    std::vector<std::array<float,3>> const& directions) {

        Eigen::Vector3d projectedCoord(ecefOrigin[0], ecefOrigin[1], ecefOrigin[2]);

        proj_trans_generic(_projector, PJ_FWD,
                           &projectedCoord[0], sizeof(double), 1,
                           &projectedCoord[1], sizeof(double), 1,
                           &projectedCoord[2], sizeof(double), 1,
                           nullptr,0,0); //project from ecef coordinates

        Eigen::Vector3d reprojected = projectedCoord;
        reprojected[2] = _minTerrainHeight;

        proj_trans_generic(_reprojector, PJ_FWD,
                           &reprojected[0], sizeof(double), 1,
                           &reprojected[1], sizeof(double), 1,
                           &reprojected[2], sizeof(double), 1,
                           nullptr,0,0); //project to ecef coordinates

        projectedCoord[2] = 1;

        Eigen::Vector2d imgOrigin = _invGeoTransform*projectedCoord;

        int imgI = std::round(imgOrigin[1]);
        int imgJ = std::round(imgOrigin[0]);

        ProjectionResults ret;
        ret.originMapPos[0] = imgOrigin[0];
        ret.originMapPos[1] = imgOrigin[1];

        std::array<double,3> downDirection = {reprojected[0] - ecefOrigin[0],
                                              reprojected[1] - ecefOrigin[1],
                                              reprojected[2] - ecefOrigin[2]};

        std::array<double,3> currentProjectionDirection = {directions[0][0],directions[0][1], directions[0][2]};

        ret.projectedPoints.resize(directions.size());

        for (int i = 0; i < directions.size(); i++) {

            currentProjectionDirection = {directions[i][0],directions[i][1], directions[i][2]};

            // we try to reproject assuming the previous vector is down.
            // this is not guaranteed to give the nearest intersection between the ray and the surface.
            // but if succesive directions intersect the surface close to one another, and there is no shading effect
            std::optional<std::array<float,2>> projected = projectPoints(ecefOrigin,
                                                                         currentProjectionDirection,
                                                                         downDirection,
                                                                         imgOrigin);

            if (projected.has_value()) {
                ret.projectedPoints[i] = projected.value();
            } else {
                ret.projectedPoints[i] = {std::nanf(""), std::nanf("")};
            }
        }

        return ret;

    }

    /*!
     * \brief projectVectorsOptimized, project a set of vectors on the terrain
     * \param ecefOrigin the position the vectors are projected from.
     * \param directions the projection direction
     * \return a structure with the map coordinate of the center of projection and the different projected coordinates (in image space of the terrain model).
     *
     * This function tries to speed up the projection process of a chunk of vectors by using some topological assumptions.
     *
     * Mainly, instead of projecting each vector one by one, it considers that sucsessive vectors in the list of directions are close together and search the projection nearby.
     *
     * To ensure that no obstructed intersection is detected, the algorithm process as such:
     *
     * - Search from the projection of the first and last vector stating from the pixel below the origin (with an algorithm garanteed to find the correct intersection).
     * - Search along the path formed by the intersection of the surface between the sucessive vectors for the other vector intersection.
     *
     * It is thus important to sort the directions such that vectors which are likely to have the same intersection are close
     * to one another.
     *
     * In some cases, this function can return projections which are in fact in the shade, but it will be much, much faster
     * than a brute force approach for projecting vectors.
     *
     * This function also update the cover map. Since it only switch pixels in the cover map from false to true,
     * it can be called from multiple threads at once with no impact on race conditions.
     *
     */
    std::optional<ProjectionResults> projectVectorsOptimized(std::array<double,3> const& ecefOrigin,
                                                             std::vector<std::array<float,3>> const& directions) {

        Eigen::Vector3d projectedCoord(ecefOrigin[0], ecefOrigin[1], ecefOrigin[2]);

        proj_trans_generic(_projector, PJ_FWD,
                           &projectedCoord[0], sizeof(double), 1,
                           &projectedCoord[1], sizeof(double), 1,
                           &projectedCoord[2], sizeof(double), 1,
                           nullptr,0,0); //project from ecef coordinates of the origin

        Eigen::Vector3d reprojected = projectedCoord;
        reprojected[2] = _minTerrainHeight;

        proj_trans_generic(_reprojector, PJ_FWD,
                           &reprojected[0], sizeof(double), 1,
                           &reprojected[1], sizeof(double), 1,
                           &reprojected[2], sizeof(double), 1,
                           nullptr,0,0); //project to ecef coordinates of the point on the ground

        projectedCoord[2] = 1;

        Eigen::Vector2d imgOrigin = _invGeoTransform*projectedCoord;

        int imgI = std::round(imgOrigin[1]);
        int imgJ = std::round(imgOrigin[0]);

        ProjectionResults ret;
        ret.originMapPos[0] = imgOrigin[0];
        ret.originMapPos[1] = imgOrigin[1];

        std::array<double,3> downDirection = {reprojected[0] - ecefOrigin[0],
                                              reprojected[1] - ecefOrigin[1],
                                              reprojected[2] - ecefOrigin[2]};

        std::array<double,3> currentProjectionDirection = {directions[0][0],directions[0][1], directions[0][2]};
        std::array<double,3> previousProjectionDirection = downDirection;
        Eigen::Vector2d previousProjectedCoord = imgOrigin;

        ret.projectedPoints.resize(directions.size());

        for (int i = 0; i < directions.size(); i++) {

            currentProjectionDirection = {directions[i][0],directions[i][1], directions[i][2]};

            // we try to reproject assuming the previous vector is down.
            // this is not guaranteed to give the nearest intersection between the ray and the surface.
            // but if succesive directions intersect the surface close to one another, and there is no shading effect
            std::optional<std::array<float,2>> projected = projectPoints(ecefOrigin,
                                                                         currentProjectionDirection,
                                                                         previousProjectionDirection,
                                                                         previousProjectedCoord);

            if (projected.has_value()) {

                ret.projectedPoints[i] = projected.value();

                previousProjectionDirection = currentProjectionDirection;
                previousProjectedCoord = Eigen::Vector2d(ret.projectedPoints[i][0], ret.projectedPoints[i][1]);

            } else {

                ret.projectedPoints[i] = {std::nanf(""), std::nanf("")};

                //do not update previous directions. It would make sense that
            }
        }

        return ret;

    }

    /*!
     * \brief clearCover reset the cover estimate
     *
     * This function will reset all pixels in the cover to false (not seen).
     * It should not be called while another thread is using the projector to avoid race conditions.
     */
    void clearCover() {
        for (int i = 0; i < _cover.shape()[0]; i++) {
            for (int j = 0; j < _cover.shape()[1]; j++) {
                _cover.atUnchecked(i,j) = false;
            }
        }
    }

    /*!
     * \brief cover gets the pixels of the terrain that have been seen.
     * \return a reference to the cover map in the projector
     */
    Multidim::Array<bool, 2> const& cover() const {
        return _cover;
    }

protected:

    /*!
     * \brief projectPoints is a point projection function garanteed to find the correct position of a point on a terrain.
     * \param ecefOrigin the ecef coordinate of the center of projection.
     * \param direction the direction the projected point should be searched (in ecef frame)
     * \param downDir the direction pointing from the center of projection towards the map (in ecef frame)
     * \param searchStart the point position (on the map) below the center of projection.
     * \return the projected coordinates of the point on the terrain, in image space. or std::nullopt if the view direction does not intersect the terrain
     */
    inline std::optional<std::array<float,2>> projectPoints(std::array<double,3> const& ecefOrigin,
                                                            std::array<double,3> const& direction,
                                                            std::array<double,3> const& downDir,
                                                            Eigen::Vector2d const& searchStart) {

        Eigen::Vector3d directionVec(direction[0], direction[1], direction[2]);
        directionVec.normalize();

        Eigen::Vector3d downVec(downDir[0], downDir[1], downDir[2]);
        downVec.normalize();

        Eigen::Vector3d perpVec = directionVec.cross(downVec);
        perpVec.normalize();

        Eigen::Matrix3d searchSpace;
        searchSpace.block<3,1>(0,0) = directionVec;
        searchSpace.block<3,1>(0,1) = downVec;
        searchSpace.block<3,1>(0,2) = perpVec;

        Eigen::ColPivHouseholderQR<Eigen::Matrix3d> searchSpaceInv = searchSpace.colPivHouseholderQr();

        //initialize the first triangle
        double startX = searchStart[0];
        double startY = searchStart[1];

        double fracX = startX - std::floor(startX);
        double fracY = startY - std::floor(startY);

        if (std::floor(startX) == std::ceil(startX)) {

            if (std::floor(startY) == std::ceil(startY)) {

                startX += 1./4.;
                startY += 1./4.;

            } else {
                startX += fracY/2.;

                if (std::floor(startX) == std::ceil(startX)) {
                    startX += fracY;
                }
            }

        } else if (std::floor(startY) == std::ceil(startY)) {
            startY += fracX/2.;

            if (std::floor(startY) == std::ceil(startY)) {
                startY += fracX;
            }
        }

        std::array<int,2> currentTrianglePoint1 = {int(std::floor(startX)), int(std::floor(startY))};
        std::array<int,2> currentTrianglePoint2 = (fracX - fracY < 0) ?
                                                    std::array<int,2>{int(std::floor(startX)), int(std::ceil(startY))} :
                                                    std::array<int,2>{int(std::ceil(startX)), int(std::floor(startY))};
        std::array<int,2> currentTrianglePoint3 = {int(std::ceil(startX)), int(std::ceil(startY))};

        std::array<std::array<int,2>,3> trianglePoints = {currentTrianglePoint1, currentTrianglePoint2, currentTrianglePoint3};

        assert(std::fabs(currentTrianglePoint2[0] - searchStart[0]) + std::fabs(currentTrianglePoint2[1] - searchStart[1]) <= 1.);

        bool intersectionInTriangle = false;

        Eigen::Matrix4d triangleAdjustement; //coordinates (in ecef) of the corners of the triangle., -direction; ones for barycentric constraint and 0 in final corner.

        triangleAdjustement.block<1,3>(3,0) = Eigen::Matrix<double,1,3>::Ones();
        triangleAdjustement(3,3) = 0;
        triangleAdjustement.block<3,1>(0,3) = directionVec;

        Eigen::Vector4d o;
        o[0] = ecefOrigin[0];
        o[1] = ecefOrigin[1];
        o[2] = ecefOrigin[2];
        o[3] = 1;

        Eigen::Vector3d projectedPoint;

        do {

            for (int i = 0; i < 3; i++) {
                std::array<int,2>& pointCoord = trianglePoints[i];

                if (pointCoord[0] < 0 or pointCoord[0] >= _ecefTerrain.shape()[0]) {
                    return std::nullopt;
                }

                if (pointCoord[1] < 0 or pointCoord[1] >= _ecefTerrain.shape()[1]) {
                    return std::nullopt;
                }

                triangleAdjustement(0,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],0);
                triangleAdjustement(1,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],1);
                triangleAdjustement(2,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],2);
            }

            Eigen::Vector4d results = triangleAdjustement.colPivHouseholderQr().solve(o);

            Eigen::Vector3d coeffs = results.block<3,1>(0,0);

            assert(std::fabs(coeffs[0] + coeffs[1] + coeffs[2] - 1) < 1e-6);

            if (coeffs[0] >= 0 and coeffs[1] >= 0 and coeffs[2] >= 0) { //Barycentric coordinates within the triangle
                intersectionInTriangle = true;
                projectedPoint = triangleAdjustement.block<3,3>(0,0)*coeffs;
                break;
            }

            //else select the next triangle to move to (need to change only a single point)

            int countNegative = 0;
            int lastNegative = -1;
            int lastPositive = -1;

            for (int i = 0; i < 3; i++) {
                if (coeffs[i] < 0) {
                    countNegative++;
                    lastNegative = i;
                } else {
                    lastPositive = i;
                }
            }

            assert(countNegative > 0 and countNegative < 3);

            if (countNegative > 1) {

                std::array<int,2> negatives;

                negatives[0] = lastNegative;
                negatives[1] = 3 - lastPositive - negatives[0];

                std::array<double,2> dists;

                for (int i = 0; i < 2; i++) {

                    Eigen::Vector3d vi = triangleAdjustement.block<3,1>(0, negatives[i]);

                    dists[i] = searchSpaceInv.solve(vi).x();
                }

                lastNegative = (dists[0] < dists[1]) ? negatives[0] : negatives[1];
            }

            //Switch to the next triangle. Along the edge opposed to the point with negative barycentric weight.
            std::array<int,2>& pointCoord = trianglePoints[lastNegative];

            std::array<int,2> delta = {0,0};

            //This formula for the index delta can be obtained by considering that the next triangle form a parallelogram with the current triangle.
            for (int i = 0; i < 3; i++) {
                delta[0] +=  trianglePoints[i][0] - pointCoord[0];
                delta[1] +=  trianglePoints[i][1] - pointCoord[1];
            }

            pointCoord[0] += delta[0];
            pointCoord[1] += delta[1];


        } while (!intersectionInTriangle);

        Eigen::Vector3d geoProj(projectedPoint[0], projectedPoint[1], projectedPoint[2]);

        proj_trans_generic(_projector, PJ_FWD,
                           &geoProj[0], sizeof(double), 1,
                           &geoProj[1], sizeof(double), 1,
                           &geoProj[2], sizeof(double), 1,
                           nullptr,0,0); //project from ecef coordinates

        geoProj[2] = 1;
        Eigen::Vector2d imgProj = _invGeoTransform*geoProj;

        return std::array<float,2>{float(imgProj[0]), float(imgProj[1])};

    }

    GeoRasterData<TerrainT,2> const& _terrain;
    Multidim::Array<float, 3> _ecefTerrain;

    TerrainT _minTerrainHeight;
    TerrainT _maxTerrainHeight;
    Eigen::Matrix<double, 2,3> _invGeoTransform;

    Multidim::Array<bool, 2> _cover;

    //PROJ variable
    PJ_CONTEXT* _ctx;

    PJ* _projector;
    PJ* _reprojector;
};

}
}

#endif // TERRAINPROJECTOR_H
