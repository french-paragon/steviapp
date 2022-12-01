#ifndef STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H
#define STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class Landmark;

class LandmarkBaseActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	explicit LandmarkBaseActionManager(QObject *parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
	QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const override;

protected:

	QAction* createAssignToDistanceConstrainAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const;
	QAction* createAssignToAngleConstrainAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const;
	QAction* createAddToLocalCoordinateSystemAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H
