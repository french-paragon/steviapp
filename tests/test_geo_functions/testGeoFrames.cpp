#include <QtTest/QtTest>

#include <QMap>

#include "geo/wgs84.h"

#include <proj.h>

using namespace StereoVisionApp;

const char* wgs84_latlon = "EPSG:4326";
const char* wgs84_ecef = "EPSG:4978";

class testGeoFrames : public QObject {
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testWGS84();

    void cleanupTestCase();

protected:
    PJ_CONTEXT* _ctx;
};

void testGeoFrames::initTestCase() {
    _ctx = proj_context_create();
    if (_ctx == nullptr) {
        QSKIP("Failed to init proj context");
    }
}

void testGeoFrames::testWGS84() {

    PJ* reprojector = proj_create_crs_to_crs(_ctx, wgs84_latlon, wgs84_ecef, nullptr);

    if (reprojector == nullptr) {
        QSKIP("Failed to init proj crs 2 crs transform");
    }

    std::vector<std::array<double,3>> latLonHeightCoords =
        {{45.576,12.98218,120.21},{89.12,0,2343.27},{0,-179.21,-219.2},{-21.89,119.21,3219.2}};


    std::vector<std::array<double,3>> ecefCoords(latLonHeightCoords.size());

    for (size_t i = 0; i < latLonHeightCoords.size(); i++) {
        PJ_COORD llh;
        llh.lpz.lam = latLonHeightCoords[i][0];
        llh.lpz.phi = latLonHeightCoords[i][1];
        llh.lpz.z = latLonHeightCoords[i][2];

        PJ_COORD ecef = proj_trans(reprojector,PJ_FWD,llh);
        ecefCoords[i][0] = ecef.xyz.x;
        ecefCoords[i][1] = ecef.xyz.y;
        ecefCoords[i][2] = ecef.xyz.z;
    }

    //test forward
    for (size_t i = 0; i < latLonHeightCoords.size(); i++) {
        std::array<double,3> ecefComputed = StereoVisionApp::Geo::WGS84Ellipsoid::LatLonHeight2ECEF(latLonHeightCoords[i]);

        QCOMPARE(ecefComputed[0],ecefCoords[i][0]);
        QCOMPARE(ecefComputed[1],ecefCoords[i][1]);
        QCOMPARE(ecefComputed[2],ecefCoords[i][2]);
    }

    //test backward
    for (size_t i = 0; i < ecefCoords.size(); i++) {
        std::array<double,3> latlonHeightComputed = StereoVisionApp::Geo::WGS84Ellipsoid::Ecef2LatLonHeight(ecefCoords[i]);

        QVERIFY(std::abs(latlonHeightComputed[0] - latLonHeightCoords[i][0]) < 1e-6);
        QVERIFY(std::abs(latlonHeightComputed[1] - latLonHeightCoords[i][1]) < 1e-6);
        QVERIFY(std::abs(latlonHeightComputed[2] - latLonHeightCoords[i][2]) < 1e-6);
    }

    proj_destroy(reprojector);
}

void testGeoFrames::cleanupTestCase() {
    if (_ctx != nullptr) {
        proj_context_destroy(_ctx);
    }
}

QTEST_MAIN(testGeoFrames)
#include "testGeoFrames.moc"
