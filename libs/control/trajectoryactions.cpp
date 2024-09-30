#include "trajectoryactions.h"

#include "datablocks/trajectory.h"

#include "control/mainwindow.h"

#include "gui/editor.h"
#include "gui/sparsealignementeditor.h"

#include "gui/opengl3dsceneviewwidget.h"
#include "gui/openGlDrawables/opengldrawabletrajectory.h"

#include <QMessageBox>

namespace StereoVisionApp {

void viewTrajectory(Trajectory* traj, bool optimized) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (traj == nullptr) {
        return;
    }

    if (mw == nullptr) {
        return; //need main windows to display trajectory
    }

    StereoVisionApp::Editor* e = mw->openEditor(SparseAlignementEditor::staticMetaObject.className());

    SparseAlignementEditor* sae = qobject_cast<SparseAlignementEditor*>(e);

    if (sae == nullptr) {
        return;
    }

    QString traj_drawable_name = QString("TrajectoryView_%1").arg(traj->objectName());

    if (optimized) {
        traj_drawable_name = QString("TrajectoryViewOpt_%1").arg(traj->objectName());
    }

    OpenGlDrawable* drawable = sae->getDrawable(traj_drawable_name);
    OpenGlDrawableTrajectory* drawableTrajectory = qobject_cast<OpenGlDrawableTrajectory*>(drawable);

    if (drawable == nullptr) {
        drawableTrajectory = new OpenGlDrawableTrajectory();
        drawableTrajectory->setSceneScale(sae->sceneScale());

        if (optimized) {
            drawableTrajectory->setBaseColor(QColor(180,200,10));

            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::X,
                                                     QColor(255, 0, 255));
            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::Y,
                                                     QColor(150, 255, 0));
            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::Z,
                                                     QColor(0, 255, 255));
        }

        QObject::connect(sae, &SparseAlignementEditor::sceneScaleChanged,
                         drawableTrajectory, &OpenGlDrawableTrajectory::setSceneScale);

        sae->addDrawable(traj_drawable_name, drawableTrajectory);

    } else if (drawableTrajectory == nullptr) {
        //a drawable already exist with the name and is not a OpenGlDrawableTrajectory

        return; //error, this is not supposed to happen

    }

    std::vector<StereoVision::Geometry::AffineTransform<float>> trajData = traj->loadTrajectoryInProjectLocalFrame(optimized); //get the trajectory
    if (trajData.empty()) {
        QMessageBox::information(mw, QObject::tr("Empty trajectory"), QObject::tr("Empty trajectory, please ensure datasources are correctly configured!"));
    }
    constexpr int nOrientationHandles = 9;
    drawableTrajectory->setTrajectory(trajData, nOrientationHandles);

    QObject::connect(traj, &Trajectory::trajectoryDataChanged, drawableTrajectory, [drawableTrajectory, traj] () {
        drawableTrajectory->setTrajectory(traj);
    });
}

} // namespace StereoVisionApp
