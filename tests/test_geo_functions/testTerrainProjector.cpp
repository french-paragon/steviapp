#include <QtTest/QtTest>

#include <QMap>

#include "geo/terrainProjector.h"

using namespace StereoVisionApp;

const char* wgs84_latlon = "EPSG:4326";
const char* wgs84_ecef = "EPSG:4978";

struct TerrainProjectionTestData {

    StereoVisionApp::Geo::GeoRasterData<float, 2> terrain;
    std::array<double,3> center;
    std::vector<std::array<float,3>> viewDirections;
    std::vector<std::array<float,2>> projections;
};

TerrainProjectionTestData buildFlatTerrain(int wRadius = 50, int hRadius = 50, int nProjection = 300, double pixSize = 0.1) {

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* reprojector = proj_create_crs_to_crs(ctx, wgs84_latlon, wgs84_ecef, nullptr);

    TerrainProjectionTestData data;

    Multidim::Array<float,2> terrain(2*hRadius+1, 2*wRadius+1);

    for (int i = 0; i < terrain.shape()[0]; i++) {
        for (int j = 0; j < terrain.shape()[1]; j++) {
            terrain.atUnchecked(i,j) = 0;
        }
    }

    Eigen::Matrix<double, 2,3> geoTransform = Eigen::Matrix<double, 2,3>::Zero();
    geoTransform(0,0) = -pixSize;
    geoTransform(1,1) = -pixSize;

    geoTransform(0,2) = wRadius*pixSize;
    geoTransform(1,2) = hRadius*pixSize;

    std::string crsInfos = wgs84_latlon;

    std::vector<std::array<float,2>> projections(nProjection);
    std::vector<std::array<float,3>> viewDirections(nProjection);

    for (int i = 0; i < nProjection; i++) {

        float maxI = terrain.shape()[0]-2;
        float maxJ = terrain.shape()[1]-2;

        float posI = 0.5 + float(i)/(nProjection-1) * maxI;
        float posJ = 0.5 + float(i)/(nProjection-1) * maxJ;

        projections[i] = {posI, posJ};

        Eigen::Vector3d geoCoord(posJ, posI, 1);
        geoCoord.block<2,1>(0,0) = geoTransform*geoCoord;
        geoCoord[2] = 0;

        proj_trans_generic(reprojector, PJ_FWD,
                           &geoCoord[0], sizeof(double), 1,
                           &geoCoord[1], sizeof(double), 1,
                           &geoCoord[2], sizeof(double), 1,
                           nullptr,0,0); //reproject to ecef coordinates

        std::array<float,3> ecefCoord = {float(geoCoord[0]), float(geoCoord[1]), float(geoCoord[2])};

        viewDirections[i] = ecefCoord;
    }

    double dist = 0;

    for (int i = 0; i < 3; i++) {
        double tmp = viewDirections[0][i] - viewDirections[nProjection-1][i];
        dist += tmp*tmp;
    }

    dist = std::sqrt(dist);

    Eigen::Vector3d geoCoord(wRadius, hRadius, 1);
    geoCoord.block<2,1>(0,0) = geoTransform*geoCoord;
    geoCoord[2] = dist;

    proj_trans_generic(reprojector, PJ_FWD,
                       &geoCoord[0], sizeof(double), 1,
                       &geoCoord[1], sizeof(double), 1,
                       &geoCoord[2], sizeof(double), 1 ,
                       nullptr,0,0); //reproject to ecef coordinates

    std::array<double,3> ecefCenter = {geoCoord[0], geoCoord[1], geoCoord[2]};

    for (int i = 0; i < nProjection; i++) {
        viewDirections[i][0] -= ecefCenter[0];
        viewDirections[i][1] -= ecefCenter[1];
        viewDirections[i][2] -= ecefCenter[2];
    }

    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    data.terrain = StereoVisionApp::Geo::GeoRasterData<float, 2>{std::move(terrain), geoTransform, crsInfos};
    data.center = ecefCenter;
    data.viewDirections = viewDirections;
    data.projections = projections;

    return data;

}

class testTerrainProjector : public QObject {
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase_data();
    void initTestCase();

    void testProjection();
    void benchmarkProjection();

protected:

    TerrainProjectionTestData _terrainProjectionData;
};

void testTerrainProjector::initTestCase_data() {

}

void testTerrainProjector::initTestCase() {
    _terrainProjectionData = buildFlatTerrain();

    QVERIFY2(std::isfinite(_terrainProjectionData.center[0]) and
            std::isfinite(_terrainProjectionData.center[1]) and
            std::isfinite(_terrainProjectionData.center[2]) ,
            "Invalid center of projection");

    QCOMPARE(_terrainProjectionData.viewDirections.size(), _terrainProjectionData.projections.size());

    for (int i = 0; i < _terrainProjectionData.viewDirections.size(); i++) {


        QVERIFY2(std::isfinite(_terrainProjectionData.viewDirections[i][0]) and
                std::isfinite(_terrainProjectionData.viewDirections[i][1]) and
                std::isfinite(_terrainProjectionData.viewDirections[i][2]) ,
                qPrintable(QString("Invalid view direction %1").arg(i)));
    }

    for (int i = 0; i < _terrainProjectionData.projections.size(); i++) {


        QVERIFY2(std::isfinite(_terrainProjectionData.projections[i][0]) and
                std::isfinite(_terrainProjectionData.projections[i][1]) ,
                qPrintable(QString("Invalid projection %1").arg(i)));
    }

    for (int i = 0; i < _terrainProjectionData.terrain.raster.shape()[0]; i++) {
        for (int j = 0; j < _terrainProjectionData.terrain.raster.shape()[1]; j++) {
            QVERIFY2(std::isfinite(_terrainProjectionData.terrain.raster.valueUnchecked(i,j)),
                    qPrintable(QString("Invalid projection %1").arg(i)));
        }
    }
}

void testTerrainProjector::testProjection() {

    using RetType = Geo::TerrainProjector<float>::ProjectionResults;

    Geo::TerrainProjector<float> projector(_terrainProjectionData.terrain);

    std::optional<RetType> retVal = projector.projectVectors(_terrainProjectionData.center,
                                                                  _terrainProjectionData.viewDirections);

    QVERIFY(retVal.has_value());

    RetType projections = retVal.value();

    QCOMPARE(projections.projectedPoints.size(), _terrainProjectionData.viewDirections.size());

    double pixTol = 0.001;

    for (int i = 0; i < projections.projectedPoints.size(); i++) {

        QVERIFY2(std::isfinite(projections.projectedPoints[i][0]), qPrintable(QString("Reprojected point x is not finite at index %1").arg(i)));
        QVERIFY2(std::isfinite(projections.projectedPoints[i][1]), qPrintable(QString("Reprojected point y is not finite at index %1").arg(i)));

        double deltax = std::abs(projections.projectedPoints[i][0] - _terrainProjectionData.projections[i][0]);
        double deltay = std::abs(projections.projectedPoints[i][1] - _terrainProjectionData.projections[i][1]);

        double min = std::min(deltax, deltay);
        double max = std::max(deltax, deltay);

        double ratio = min/max;

        if (!std::isfinite(ratio)) {
            ratio = 0;
        }

        double dist = max*sqrt(1 + ratio*ratio);

        if (i != 0) {
            continue;
        }

        QVERIFY2(dist < pixTol, qPrintable(QString("Reprojected point too far from ground truth (dist = %1, index = %2)").arg(dist).arg(i)));

        deltax = std::abs(projections.projectedPoints[i][0] - _terrainProjectionData.projections[i][0]);
        deltay = std::abs(projections.projectedPoints[i][1] - _terrainProjectionData.projections[i][1]);

        min = std::min(deltax, deltay);
        max = std::max(deltax, deltay);

        ratio = min/max;

        if (!std::isfinite(ratio)) {
            ratio = 0;
        }

        dist = max*sqrt(1 + ratio*ratio);

        QVERIFY2(dist < pixTol, qPrintable(QString("ReprojectedOpt point too far from ground truth (dist = %1, index = %2)").arg(dist).arg(i)));
    }

}

void testTerrainProjector::benchmarkProjection() {

    using RetType = Geo::TerrainProjector<float>::ProjectionResults;

    Geo::TerrainProjector<float> projector(_terrainProjectionData.terrain);

    QBENCHMARK {
        std::optional<RetType> retVal = projector.projectVectors(_terrainProjectionData.center,
                                                                 _terrainProjectionData.viewDirections);
    };

}
QTEST_MAIN(testTerrainProjector)
#include "testTerrainProjector.moc"
