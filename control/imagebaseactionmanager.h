#ifndef STEREOVISIONAPP_IMAGEBASEACTIONMANAGER_H
#define STEREOVISIONAPP_IMAGEBASEACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class Image;

class ImageBaseActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	ImageBaseActionManager(QObject* parent);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
	QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const override;

protected:

	QAction* createAssignToCameraAction(QObject* parent, Project* p, const QVector<Image *> &ims) const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEBASEACTIONMANAGER_H
