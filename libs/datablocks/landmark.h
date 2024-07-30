#ifndef STEREOVISIONAPP_LANDMARK_H
#define STEREOVISIONAPP_LANDMARK_H

#include "./project.h"
#include "./point3d.h"
#include "./floatparameter.h"
#include "./georeferenceddatablockinterface.h"

#include <QSet>

namespace StereoVisionApp {

class Landmark : public Point3D, public GeoReferencedDataBlockInterface
{
	Q_OBJECT
    Q_INTERFACES(StereoVisionApp::GeoReferencedDataBlockInterface)

public:

	explicit Landmark(Project* parent = nullptr);

	QVector<qint64> getImagesRefering() const;

	int countImagesRefering(QSet<qint64> const& excluded = {}) const;
	int countImagesRefering(QVector<qint64> const& excluded) const;

	int countLocalCoordinateSystemsRefering(QSet<qint64> const& excluded = {}) const;
	int countLocalCoordinateSystemsRefering(QVector<qint64> const& excluded) const;

	int countImagesReferingInList(QSet<qint64> const& included) const;
	int countImagesReferingInList(QVector<qint64> const& included) const;

	QSet<qint64> getViewingImgInList(QSet<qint64> const& included) const;

    bool geoReferenceSupportActive() const override;
    Eigen::Array<float,3, Eigen::Dynamic> getLocalPointsEcef() const override;
    QString getCoordinateReferenceSystemDescr(int role = DefaultCRSRole) const override;

    /*!
     * \brief getOptimizableCoordinates get coordinates of the landmarks in the optimization frame
     * \param optimized
     * \return
     */
    std::optional<Eigen::Vector3f> getOptimizableCoordinates(bool optimized = false) const;
    bool setPositionFromEcef(Eigen::Vector3f const& point, bool optimized = false);

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

    QString _coordinatesReferenceSystem;
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
