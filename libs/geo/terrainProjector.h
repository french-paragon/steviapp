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
        _projector = nullptr;
        _reprojector = nullptr;
        _ctx = nullptr;

        if (terrain.raster.empty()) { //empty terrain, might happen with default constructed projector
            _invGeoTransform.setConstant(0);
            return;
        }

        _invGeoTransform.template block<2,2>(0,0) = Eigen::Matrix2d(terrain.geoTransform.template block<2,2>(0,0)).inverse();
        _invGeoTransform.template block<2,1>(0,2) = -_invGeoTransform.template block<2,2>(0,0)*terrain.geoTransform.template block<2,1>(0,2);

        _cover = Multidim::Array<bool, 2>(terrain.raster.shape());
        clearCover();

        //reprojection
        int nPoints = terrain.raster.shape()[0]*terrain.raster.shape()[1];

        typename Multidim::Array<double,3>::ShapeBlock shape = {terrain.raster.shape()[0], terrain.raster.shape()[1], 3};
        typename Multidim::Array<double,3>::ShapeBlock strides = {terrain.raster.shape()[1]*3, 3, 1}; //ensure each triplets of floats is a point.

        _ecefTerrain = typename Multidim::Array<double,3>(shape, strides);

        _minTerrainHeight = std::numeric_limits<double>::infinity();
        _maxTerrainHeight = -std::numeric_limits<double>::infinity();

        _ecefOffset = {0,0,0};

        _ctx = proj_context_create();

        const char* wgs84_ecef = "EPSG:4978"; //The WGS84 earth centered, earth fixed frame used for projection on the terrain

        bool needReproject = terrain.crsInfos != wgs84_ecef and !terrain.crsInfos.empty();

        if (needReproject) {
            _projector = proj_create_crs_to_crs(_ctx, wgs84_ecef, terrain.crsInfos.c_str(), nullptr);
            _reprojector = proj_create_crs_to_crs(_ctx, terrain.crsInfos.c_str(), wgs84_ecef, nullptr);

            if (_projector == nullptr) { //in case of error
                return;
            }

            if (_reprojector == nullptr) { //in case of error
                return;
            }
        }

        int nFinitePoints = 0;

        //Build the ECEF lattice
        for (int i = 0; i < shape[0]; i++) {
            for (int j = 0; j < shape[1]; j++) {

                Eigen::Vector3d homogeneousImgCoord(j,i,1);
                Eigen::Vector2d geoCoord = terrain.geoTransform*homogeneousImgCoord;

                TerrainT terrainVal = _terrain.raster.valueUnchecked(i,j);

                PJ_COORD coord;

                coord.v[0] = geoCoord.x();
                coord.v[1] = geoCoord.y();
                coord.v[2] = terrainVal;
                coord.v[3] = 0;

                //invalid node in the terrain
                if (terrainVal < -1e4 or !std::isfinite(terrainVal)) {

                    _ecefTerrain.atUnchecked(i,j,0) = std::nan("");
                    _ecefTerrain.atUnchecked(i,j,1) = std::nan("");
                    _ecefTerrain.atUnchecked(i,j,2) = std::nan("");
                    continue;
                }

                //reproject to ecef coordinates

                if (needReproject) {
                    PJ_COORD reproject = proj_trans(_reprojector, PJ_FWD, coord);

                    _ecefTerrain.atUnchecked(i,j,0) = reproject.v[0];
                    _ecefTerrain.atUnchecked(i,j,1) = reproject.v[1];
                    _ecefTerrain.atUnchecked(i,j,2) = reproject.v[2];

                    if (terrainVal < -1e-3 or !std::isfinite(terrainVal)) { // some DTM uses large negative values to indicate invalid pixels.
                        continue;
                    }

                } else {

                    _ecefTerrain.atUnchecked(i,j,0) = coord.v[0];
                    _ecefTerrain.atUnchecked(i,j,1) = coord.v[1];
                    _ecefTerrain.atUnchecked(i,j,2) = coord.v[2];

                }

                _minTerrainHeight = std::min(terrainVal, _minTerrainHeight);
                _maxTerrainHeight = std::max(terrainVal, _maxTerrainHeight);

                if (std::isfinite(_ecefTerrain.atUnchecked(i,j,0)) and
                        std::isfinite(_ecefTerrain.atUnchecked(i,j,1)) and
                        std::isfinite(_ecefTerrain.atUnchecked(i,j,2))) {

                    _ecefOffset[0] += _ecefTerrain.atUnchecked(i,j,0);
                    _ecefOffset[1] += _ecefTerrain.atUnchecked(i,j,1);
                    _ecefOffset[2] += _ecefTerrain.atUnchecked(i,j,2);

                    nFinitePoints++;
                }
            }
        }

        _ecefOffset[0] /= nFinitePoints;
        _ecefOffset[1] /= nFinitePoints;
        _ecefOffset[2] /= nFinitePoints;

        //Offset for numerical stability
        for (int i = 0; i < shape[0]; i++) {
            for (int j = 0; j < shape[1]; j++) {
                _ecefTerrain.atUnchecked(i,j,0) -= _ecefOffset[0];
                _ecefTerrain.atUnchecked(i,j,1) -= _ecefOffset[1];
                _ecefTerrain.atUnchecked(i,j,2) -= _ecefOffset[2];
            }
        }

        buildBVH();

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

        if (_terrain.crsInfos.empty()) {
            return true;
        }

        return _reprojector != nullptr and _projector != nullptr;
    }

    /*!
     * \brief getTerrainAxisAlignedBoundingBox get the bounding box of the terrain
     * \return an array {min x, max x, min y, max y, min z, max z}
     */
    inline std::array<double, 6> getTerrainAxisAlignedBoundingBox() const {

        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();

        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();

        double minZ = std::numeric_limits<double>::infinity();
        double maxZ = -std::numeric_limits<double>::infinity();

        if (_boudingVolumeHierarchy.size() > 0) {
            Multidim::Array<double,3> const& largestAABB = _boudingVolumeHierarchy[_boudingVolumeHierarchy.size()-1];

            for (int i = 0; i < largestAABB.shape()[0]; i++) {
                for (int j = 0; j < largestAABB.shape()[1]; j++) {

                    double candMinX = largestAABB.valueUnchecked(i,j,BVHminXidx);
                    double candMaxX = largestAABB.valueUnchecked(i,j,BVHmaxXidx);

                    double candMinY = largestAABB.valueUnchecked(i,j,BVHminYidx);
                    double candMaxY = largestAABB.valueUnchecked(i,j,BVHmaxYidx);

                    double candMinZ = largestAABB.valueUnchecked(i,j,BVHminZidx);
                    double candMaxZ = largestAABB.valueUnchecked(i,j,BVHmaxZidx);

                    minX = std::min(candMinX,minX);
                    maxX = std::max(candMaxX,maxX);

                    minY = std::min(candMinY,minY);
                    maxY = std::max(candMaxY,maxY);

                    minZ = std::min(candMinZ,minZ);
                    maxZ = std::max(candMaxZ,maxZ);

                }
            }
        }

        return {minX, maxX, minY, maxY, minZ, maxZ};

    }

    inline std::array<int,2> getTerrainSize() const {
        return _terrain.raster.shape();
    }

    inline int bvhLevels() const {
        return _boudingVolumeHierarchy.size();
    }

    inline std::array<double,3> ecefOffset() const {
        return _ecefOffset;
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
     * This function use a BVH built by the terrain projector as an accelleration structure.
     *
     */
    std::optional<ProjectionResults> projectVectors(std::array<double,3> const& ecefOrigin,
                                                    std::vector<std::array<float,3>> const& directions) {

        if (!isValid()) {
            return std::nullopt;
        }

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

        //order i,j rather than x,y
        double tmp = imgOrigin[0];
        imgOrigin[0] = imgOrigin[1];
        imgOrigin[1] = tmp;

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

            std::optional<std::array<float,2>> projected = projectPoints(ecefOrigin,
                                                                         currentProjectionDirection);

            if (projected.has_value()) {
                ret.projectedPoints[i] = projected.value();
            } else {
                ret.projectedPoints[i] = {std::nanf(""), std::nanf("")};
            }
        }

        return ret;

    }


    //TODO create a function to use previously projected vector to restart the projection at an intermediate level in the BVH to be even faster at the cost of accuracy.

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

    static constexpr int BVHminXidx = 0;
    static constexpr int BVHmaxXidx = 1;
    static constexpr int BVHminYidx = 2;
    static constexpr int BVHmaxYidx = 3;
    static constexpr int BVHminZidx = 4;
    static constexpr int BVHmaxZidx = 5;

    /*!
     * \brief projectPoints is a point projection function garanteed to find the correct position of a point on a terrain.
     * \param ecefOrigin the ecef coordinate of the center of projection.
     * \param direction the direction the projected point should be searched (in ecef frame)
     * \return the projected coordinates of the point on the terrain, in image space. or std::nullopt if the view direction does not intersect the terrain
     */
    inline std::optional<std::array<float,2>> projectPoints(std::array<double,3> const& ecefOrigin,
                                                            std::array<double,3> const& direction) {

        Eigen::Vector3d origin;
        for (int i = 0; i < 3; i++) {
            origin[i] = ecefOrigin[i] - _ecefOffset[i];
        }

        Eigen::Vector3d directionVec(direction[0], direction[1], direction[2]);
        directionVec.normalize();

        std::optional<IntersectionInfos> proj = projectRayUsingBVH(origin, directionVec);

        if (!proj.has_value()) {
            return std::nullopt;
        }

        IntersectionInfos& intersectionInfos = proj.value();

        int imgI = std::round(intersectionInfos.dtmPixCoord[0]);
        int imgJ = std::round(intersectionInfos.dtmPixCoord[1]);

        if (imgI >= 0 and imgI < _cover.shape()[0] and imgJ >= 0 and imgJ < _cover.shape()[1]) {
            _cover.atUnchecked(imgI, imgJ) = true;
        }

        return intersectionInfos.dtmPixCoord; //order i,j instead of x,y

    }

    inline int intLog2(int val) {
        unsigned r = 0;

        while (val >>= 1) {
            r++;
        }

        return r;
    }

    inline void buildBVH() {

        int levelV = intLog2(_ecefTerrain.shape()[0]) + 1;
        int levelH = intLog2(_ecefTerrain.shape()[1]) + 1;

        int levels = std::max(levelV, levelH) + 1;
        _boudingVolumeHierarchy.reserve(levels);

        int currentHeight = _ecefTerrain.shape()[0]-1;
        int currentWidth = _ecefTerrain.shape()[1]-1;

        //fill in the first bounding volumes set (each volume is two triangles)
        Multidim::Array<double, 3>::ShapeBlock shape{currentHeight, currentWidth, 6};
        _boudingVolumeHierarchy.emplace_back(shape);

        for (int i = 0; i < _boudingVolumeHierarchy.back().shape()[0]; i++) {
            for (int j = 0; j < _boudingVolumeHierarchy.back().shape()[1]; j++) {

                double xMin = std::numeric_limits<double>::infinity();
                double xMax = -std::numeric_limits<double>::infinity();
                double yMin = std::numeric_limits<double>::infinity();
                double yMax = -std::numeric_limits<double>::infinity();
                double zMin = std::numeric_limits<double>::infinity();
                double zMax = -std::numeric_limits<double>::infinity();

                for (int di = 0; di < 2; di++) {
                    for (int dj = 0; dj < 2; dj++) {

                        int ti = i + di;
                        int tj = j + dj;

                        double xCandMin = _ecefTerrain.valueOrAlt({ti,tj,0}, xMin);
                        double xCandMax = _ecefTerrain.valueOrAlt({ti,tj,0}, xMax);
                        double yCandMin = _ecefTerrain.valueOrAlt({ti,tj,1}, yMin);
                        double yCandMax = _ecefTerrain.valueOrAlt({ti,tj,1}, yMax);
                        double zCandMin = _ecefTerrain.valueOrAlt({ti,tj,2}, zMin);
                        double zCandMax = _ecefTerrain.valueOrAlt({ti,tj,2}, zMax);

                        xMin = std::min(xMin, xCandMin);
                        xMax = std::max(xMax, xCandMax);
                        yMin = std::min(yMin, yCandMin);
                        yMax = std::max(yMax, yCandMax);
                        zMin = std::min(zMin, zCandMin);
                        zMax = std::max(zMax, zCandMax);
                    }
                }

                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminXidx) = xMin;
                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxXidx) = xMax;
                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminYidx) = yMin;
                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxYidx) = yMax;
                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminZidx) = zMin;
                _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxZidx) = zMax;

            }
        }

        while (currentHeight > 2 or currentWidth > 2) {

            int nextHeight = currentHeight/2;
            int nextWidth = currentWidth/2;

            if (2*nextHeight < currentHeight) {
                nextHeight += 1;
            }

            if (2*nextWidth < currentWidth) {
                nextWidth += 1;
            }

            Multidim::Array<double, 3>::ShapeBlock shape{nextHeight, nextWidth, 6};
            _boudingVolumeHierarchy.emplace_back(shape);

            for (int i = 0; i < _boudingVolumeHierarchy.back().shape()[0]; i++) {
                for (int j = 0; j < _boudingVolumeHierarchy.back().shape()[1]; j++) {

                    double xMin = std::numeric_limits<double>::infinity();
                    double xMax = -std::numeric_limits<double>::infinity();
                    double yMin = std::numeric_limits<double>::infinity();
                    double yMax = -std::numeric_limits<double>::infinity();
                    double zMin = std::numeric_limits<double>::infinity();
                    double zMax = -std::numeric_limits<double>::infinity();

                    for (int di = 0; di < 2; di++) {
                        for (int dj = 0; dj < 2; dj++) {

                            int ti = 2*i + di;
                            int tj = 2*j + dj;

                            int pbvhid = _boudingVolumeHierarchy.size()-2;
                            Multidim::Array<double, 3>& pbvh = _boudingVolumeHierarchy[pbvhid];

                            double xCandMin = pbvh.valueOrAlt({ti,tj,BVHminXidx}, xMin);
                            double xCandMax = pbvh.valueOrAlt({ti,tj,BVHmaxXidx}, xMax);
                            double yCandMin = pbvh.valueOrAlt({ti,tj,BVHminYidx}, yMin);
                            double yCandMax = pbvh.valueOrAlt({ti,tj,BVHmaxYidx}, yMax);
                            double zCandMin = pbvh.valueOrAlt({ti,tj,BVHminZidx}, zMin);
                            double zCandMax = pbvh.valueOrAlt({ti,tj,BVHmaxZidx}, zMax);

                            xMin = std::min(xMin, xCandMin);
                            xMax = std::max(xMax, xCandMax);
                            yMin = std::min(yMin, yCandMin);
                            yMax = std::max(yMax, yCandMax);
                            zMin = std::min(zMin, zCandMin);
                            zMax = std::max(zMax, zCandMax);
                        }
                    }

                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminXidx) = xMin;
                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxXidx) = xMax;
                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminYidx) = yMin;
                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxYidx) = yMax;
                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHminZidx) = zMin;
                    _boudingVolumeHierarchy.back().atUnchecked(i,j,BVHmaxZidx) = zMax;

                }
            }

            currentHeight = nextHeight;
            currentWidth = nextWidth;
        }

    }

    inline bool bvhCellIntersectRay(Eigen::Vector3d const& origin,
                                    Eigen::Vector3d const& direction,
                                    Eigen::Vector3d const& min,
                                    Eigen::Vector3d const& max) {

        Eigen::Vector3d minShifted = min - origin;
        Eigen::Vector3d maxShifted = max - origin;

        //need to invert that if the direction is negative
        for (int i = 0; i < 3; i++) {
            if (direction[i] < 0) {
                double tmp = minShifted[i];
                minShifted[i] = maxShifted[i];
                maxShifted[i] = tmp;
            }
        }

        Eigen::Vector3d rangesMin = minShifted.array()/direction.array();
        Eigen::Vector3d rangesMax = maxShifted.array()/direction.array();

        double rangeIntersectionsMin = rangesMin.maxCoeff();
        double rangeIntersectionsMax = rangesMax.minCoeff();

        return rangeIntersectionsMax > rangeIntersectionsMin;
    }

    struct IntersectionInfos {
        std::array<float,2> dtmPixCoord;
        float dist;
    };

    inline std::optional<IntersectionInfos> rayIntersectTerrain(Eigen::Vector3d const& origin,
                                                                Eigen::Vector3d const& direction,
                                                                std::array<int,3> const& trigI,
                                                                std::array<int,3> const& trigJ) {

        Eigen::Matrix4d triangleAdjustement; //coordinates (in ecef) of the corners of the triangle., -direction; ones for barycentric constraint and 0 in final corner.

        triangleAdjustement.block<1,3>(3,0) = Eigen::Matrix<double,1,3>::Ones();
        triangleAdjustement(3,3) = 0;
        triangleAdjustement.block<3,1>(0,3) = direction;

        Eigen::Vector4d o;
        o[0] = origin[0];
        o[1] = origin[1];
        o[2] = origin[2];
        o[3] = 1;

        for (int i = 0; i < 3; i++) {
            std::array<int,2> pointCoord = {trigI[i], trigJ[i]};

            if (pointCoord[0] < 0 or pointCoord[0] >= _ecefTerrain.shape()[0]) {
                return std::nullopt;
            }

            if (pointCoord[1] < 0 or pointCoord[1] >= _ecefTerrain.shape()[1]) {
                return std::nullopt;
            }

            triangleAdjustement(0,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],0);
            triangleAdjustement(1,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],1);
            triangleAdjustement(2,i) = _ecefTerrain.value(pointCoord[0], pointCoord[1],2);

            for (int k = 0; k < 3; k++) {
                if (!std::isfinite(triangleAdjustement(k,i))) {
                    return std::nullopt;
                }
            }
        }

        Eigen::Vector4d results = triangleAdjustement.colPivHouseholderQr().solve(o);

        Eigen::Vector3d coeffs = results.block<3,1>(0,0);

        assert(std::fabs(coeffs[0] + coeffs[1] + coeffs[2] - 1) < 1e-6);

        if (coeffs[0] >= 0 and coeffs[1] >= 0 and coeffs[2] >= 0) { //Barycentric coordinates within the triangle

            Eigen::Vector3d projectedPoint = triangleAdjustement.block<3,3>(0,0)*coeffs;

            float dist = (origin - projectedPoint).norm();

            Eigen::Vector3d geoProj(projectedPoint[0] + _ecefOffset[0],
                    projectedPoint[1] + _ecefOffset[1],
                    projectedPoint[2] + _ecefOffset[2]);

            proj_trans_generic(_projector, PJ_FWD,
                               &geoProj[0], sizeof(double), 1,
                               &geoProj[1], sizeof(double), 1,
                               &geoProj[2], sizeof(double), 1,
                               nullptr,0,0); //project from ecef coordinates

            geoProj[2] = 1;
            Eigen::Vector2d imgProj = _invGeoTransform*geoProj;

            IntersectionInfos infos{std::array<float,2>{float(imgProj[1]), float(imgProj[0])}, dist}; //order i,j instead of x,y
            return infos;
        }

        return std::nullopt;

    }

    std::optional<IntersectionInfos> projectRayUsingBVH(Eigen::Vector3d const& origin,
                                                        Eigen::Vector3d const& direction,
                                                        int level = 0,
                                                        int pixCoordI = 0,
                                                        int pixCoordJ = 0) {

        std::optional<IntersectionInfos> ret = std::nullopt;

        if (level < _boudingVolumeHierarchy.size()) {

            int bvhIdx = _boudingVolumeHierarchy.size()-level-1;
            Multidim::Array<double, 3>& currentBVH = _boudingVolumeHierarchy[bvhIdx];

            for (int di = 0; di < 2; di++) {
                for (int dj = 0; dj < 2; dj++) {

                    int i = pixCoordI + di;
                    int j = pixCoordJ + dj;

                    if (i >= currentBVH.shape()[0]) {
                        continue;
                    }

                    if (j >= currentBVH.shape()[1]) {
                        continue;
                    }

                    Eigen::Vector3d min;
                    Eigen::Vector3d max;

                    min.x() = currentBVH.atUnchecked(i,j,BVHminXidx);
                    min.y() = currentBVH.atUnchecked(i,j,BVHminYidx);
                    min.z() = currentBVH.atUnchecked(i,j,BVHminZidx);

                    max.x() = currentBVH.atUnchecked(i,j,BVHmaxXidx);
                    max.y() = currentBVH.atUnchecked(i,j,BVHmaxYidx);
                    max.z() = currentBVH.atUnchecked(i,j,BVHmaxZidx);

                    if (!bvhCellIntersectRay(origin, direction, min, max)) {
                        continue;
                    }

                    std::optional<IntersectionInfos> intersectionInfos = projectRayUsingBVH(origin,
                                                                                            direction,
                                                                                            level+1,
                                                                                            2*i,
                                                                                            2*j);

                    if (!intersectionInfos.has_value()) {
                        continue;
                    }

                    if (!ret.has_value()) {
                        ret = intersectionInfos;
                        continue;
                    }

                    if (intersectionInfos.value().dist < ret.value().dist) {
                        ret = intersectionInfos;
                    }

                }
            }

        } else if (level == _boudingVolumeHierarchy.size()) { //level of the dtm

            int i = pixCoordI/2;
            int j = pixCoordJ/2;

            if (i >= _ecefTerrain.shape()[0]-1) {
                return std::nullopt;
            }

            if (j >= _ecefTerrain.shape()[1]-1) {
                return std::nullopt;
            }

            std::optional<IntersectionInfos> intersectionCand1 = rayIntersectTerrain(origin,
                                                                                     direction,
                                                                                     {i,i,i+1},
                                                                                     {j,j+1,j});
            if (intersectionCand1.has_value()) {
                if (ret.has_value()) {
                    if (intersectionCand1.value().dist < ret.value().dist) {
                        ret = intersectionCand1;
                    }
                } else {
                    ret = intersectionCand1;
                }
            }

            std::optional<IntersectionInfos> intersectionCand2 = rayIntersectTerrain(origin,
                                                                                     direction,
                                                                                     {i,i+1,i+1},
                                                                                     {j+1,j,j+1});

            if (intersectionCand2.has_value()) {
                if (ret.has_value()) {
                    if (intersectionCand2.value().dist < ret.value().dist) {
                        ret = intersectionCand2;
                    }
                } else {
                    ret = intersectionCand2;
                }
            }

        }

        return ret;

    }

    GeoRasterData<TerrainT,2> const& _terrain;
    Multidim::Array<double, 3> _ecefTerrain;

    //BVH for the ecef terrain.
    //Each level is half the resolution of the previous one.
    //The third dimension contain vectors of the form [minX, maxX, minY, maxY, minZ, maxZ]
    //the last cells in the image are the coarser resolution;
    std::vector<Multidim::Array<double, 3>> _boudingVolumeHierarchy;

    std::array<double,3> _ecefOffset; //offset to make numerical computations more stable;

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
