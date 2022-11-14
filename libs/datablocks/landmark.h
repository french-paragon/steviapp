#ifndef STEREOVISIONAPP_LANDMARK_H
#define STEREOVISIONAPP_LANDMARK_H

#include "./project.h"
#include "./point3d.h"
#include "./floatparameter.h"

#include <QSet>

namespace StereoVisionApp {

class Landmark : public Point3D
{
	Q_OBJECT
public:

	explicit Landmark(Project* parent = nullptr);

	QVector<qint64> getImagesRefering() const;

	int countImagesRefering(QSet<qint64> const& excluded = {}) const;
	int countImagesRefering(QVector<qint64> const& excluded) const;

	int countImagesReferingInList(QSet<qint64> const& included) const;
	int countImagesReferingInList(QVector<qint64> const& included) const;

	QSet<qint64> getViewingImgInList(QSet<qint64> const& included) const;

protected:

	void extendDataModel();
};

class LandmarkFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit LandmarkFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
	static QString landmarkClassName();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARK_H
