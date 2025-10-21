#ifndef STEREOVISIONAPP_LOCALCOORDINATESYSTEMBASEACTIONMANAGER_H
#define STEREOVISIONAPP_LOCALCOORDINATESYSTEMBASEACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class LocalCoordinateSystem;

class LocalCoordinateSystemBaseActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	LocalCoordinateSystemBaseActionManager(QObject* parent);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
	QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const override;

protected:

    QAction* createAssignToTrajectoryAction(QObject* parent, Project* p, const QVector<LocalCoordinateSystem *> &lcss) const;
    QAction* createAssignToMountingAction(QObject* parent, Project* p, const QVector<LocalCoordinateSystem *> &lcss) const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALCOORDINATESYSTEMBASEACTIONMANAGER_H
