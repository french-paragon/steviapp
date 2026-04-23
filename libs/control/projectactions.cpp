#include "projectactions.h"

#include "datablocks/project.h"
#include "datablocks/georeferenceddatablockinterface.h"

#include "geo/localframes.h"

#include "mainwindow.h"

#include <QInputDialog>
#include <QMessageBox>

#include <limits>

namespace StereoVisionApp {

bool setDefaultProjectCRS(Project* p) {

    if (p == nullptr) {
        return false;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false;
    }

    bool ok = true;

    QString crs = QInputDialog::getText(mw,
                                        QObject::tr("Set project default CRS"),
                                        QObject::tr("Default Project CRS:"),
                                        QLineEdit::Normal,
                                        p->defaultProjectCRS(),
                                        &ok);

    if (!ok) {
        return false;
    }

    p->setDefaultProjectCRS(crs);
    return true;

}

bool estimateLocalCoordinateSystem(Project* p) {

    if (p == nullptr) {
        return false;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (p->hasLocalCoordinateFrame()) {
        if (mw != nullptr) {
            QMessageBox::StandardButton button = QMessageBox::question(mw,
                                  QObject::tr("Project already has a local coordinate frame!"),
                                  QObject::tr("Do you want to replace the current frame ? (This will invalidate the solution)"),
                                  QMessageBox::StandardButton::Yes|QMessageBox::StandardButton::No);

            if (button != QMessageBox::StandardButton::Yes) {
                return false;
            }
        }
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

    p->setLocalCoordinateFrame(Geo::getLocalCartesianFrameOnSphere<float>(mean, minDist).cast<double>());

    return true;
}

} //namespace StereoVisionApp
