#ifndef STEREOVISIONAPP_CORRESPONDANCESSET_H
#define STEREOVISIONAPP_CORRESPONDANCESSET_H

#include "./project.h"
#include "./floatparameter.h"

#include <QPointF>
#include <QVector3D>
#include <QSet>

#include <eigen3/Eigen/Core>

namespace StereoVisionApp {

class Image;
class LocalCoordinateSystem;

class Image2ImageCorrespondence;
class LocalCoordinates2LocalCoordinatesCorrespondence;
class LocalCoordinates2ImageCorrespondence;

/*!
 * \brief The CorrespondancesSet class represent correspondances between images or local coordinate systems
 */
class CorrespondencesSet : public DataBlock
{
    Q_OBJECT
public:
    CorrespondencesSet(Project* parent = nullptr);


    qint64 addImagesCorrespondences(qint64 img1Id, const QPointF &img1Coordinates,
                                    qint64 img2Id, const QPointF &img2Coordinates,
                                    bool uncertain = false, float sigma_pos = 1.0);

    qint64 addLocalCoordinatesCorrespondences(qint64 lc1Id, const QVector3D &sys1Coordinates,
                                              qint64 lc2Id, const QVector3D &sys2Coordinates,
                                              bool uncertain = false, float sigma_pos = 1.0);

    qint64 addLocalCoordinate2ImageCorrespondences(qint64 lcId, const QVector3D &sysCoordinates,
                                                   qint64 imgId, const QPointF &imgCoordinates,
                                                   bool uncertain = false, float sigma_pos_lcs = 1.0, float sigma_pos_img = 1.0);


    QVector<Image2ImageCorrespondence*> getAllImagesCorrespondences() const;
    QVector<Image2ImageCorrespondence*> getImageCorrespondences(qint64 imgId) const;

    QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> getAllLocalFramesCorrespondences() const;
    QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> getLocalFrameCorrespondences(qint64 lcsId) const;

    QVector<LocalCoordinates2ImageCorrespondence*> getAllImages2LocalCoordinatesCorrespondences() const;
    QVector<LocalCoordinates2ImageCorrespondence*> getImage2LocalCoordinatesCorrespondences(qint64 imgId) const;
    QVector<LocalCoordinates2ImageCorrespondence*> getLocalFrame2ImagesCorrespondences(qint64 lcsId) const;


    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

    QJsonObject getJsonRepresentation() const override;
    void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

Q_SIGNALS:

    void im2imAdded(qint64 pt);
    void im2imRemoved(qint64 pt);

    void lcs2lcsAdded(qint64 pt);
    void lcs2lcsRemoved(qint64 pt);

    void lcs2imAdded(qint64 pt);
    void lcs2imRemoved(qint64 pt);

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();
};

class Image2ImageCorrespondence : public DataBlock
{
    Q_OBJECT
public:

    explicit Image2ImageCorrespondence(CorrespondencesSet* parent = nullptr);

    qint64 attachedImg1Id() const;
    void setAttachedImg1Id(qint64 id);
    QString attachedImage1Name() const;
    Image* attachedImage1() const;

    qint64 attachedImg2Id() const;
    void setAttachedImg2Id(qint64 id);
    QString attachedImage2Name() const;
    Image* attachedImage2() const;

    floatParameter xImg1() const;
    void setXImg1(const floatParameter &x);
    void setXImg1(float x);

    floatParameter yImg1() const;
    void setYImg1(const floatParameter &y);
    void setYImg1(float y);

    void setImage1Coordinates(QPointF const& point);
    QPointF image1Coordinates() const;

    floatParameter xImg2() const;
    void setXImg2(const floatParameter &x);
    void setXImg2(float x);

    floatParameter yImg2() const;
    void setYImg2(const floatParameter &y);
    void setYImg2(float y);

    void setImage2Coordinates(QPointF const& point);
    QPointF image2Coordinates() const;

Q_SIGNALS:
    void attachedImg1idChanged(qint64 id);
    void attachedImg2idChanged(qint64 id);

    void img1xCoordChanged(floatParameter);
    void img1yCoordChanged(floatParameter);

    void img2xCoordChanged(floatParameter);
    void img2yCoordChanged(floatParameter);

    void coordsChanged();

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void referedCleared(QVector<qint64> const& referedId) override;

    qint64 _attachedImg1Id;
    qint64 _attachedImg2Id;

    floatParameter _xImg1;
    floatParameter _yImg1;

    floatParameter _xImg2;
    floatParameter _yImg2;

    friend class CorrespondencesSet;
};

class LocalCoordinates2LocalCoordinatesCorrespondence : public DataBlock
{
    Q_OBJECT
public:

    explicit LocalCoordinates2LocalCoordinatesCorrespondence(CorrespondencesSet* parent = nullptr);

    qint64 attachedLcs1Id() const;
    void setAttachedLcs1Id(qint64 id);
    QString attachedLcs1Name() const;
    LocalCoordinateSystem* attachedLcs1() const;

    qint64 attachedLcs2Id() const;
    void setAttachedLcs2Id(qint64 id);
    QString attachedLcs2Name() const;
    LocalCoordinateSystem* attachedLcs2() const;

    floatParameter xLcs1() const;
    void setXLcs1(const floatParameter &x);
    void setXLcs1(float x);

    floatParameter yLcs1() const;
    void setYLcs1(const floatParameter &y);
    void setYLcs1(float y);

    floatParameter zLcs1() const;
    void setZLcs1(const floatParameter &z);
    void setZLcs1(float z);

    void setLcs1Coordinates(QVector3D const& point);
    QVector3D lcs1Coordinates() const;

    floatParameter xLcs2() const;
    void setXLcs2(const floatParameter &x);
    void setXLcs2(float x);

    floatParameter yLcs2() const;
    void setYLcs2(const floatParameter &y);
    void setYLcs2(float y);

    floatParameter zLcs2() const;
    void setZLcs2(const floatParameter &y);
    void setZLcs2(float y);

    void setLcs2Coordinates(QVector3D const& point);
    QVector3D lcs2Coordinates() const;

Q_SIGNALS:
    void attachedLcs1idChanged(qint64 id);
    void attachedLcs2idChanged(qint64 id);

    void lcs1xCoordChanged(floatParameter);
    void lcs1yCoordChanged(floatParameter);
    void lcs1zCoordChanged(floatParameter);

    void lcs2xCoordChanged(floatParameter);
    void lcs2yCoordChanged(floatParameter);
    void lcs2zCoordChanged(floatParameter);

    void coordsChanged();

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void referedCleared(QVector<qint64> const& referedId) override;

    qint64 _attachedLcs1Id;
    qint64 _attachedLcs2Id;

    floatParameter _xLcs1;
    floatParameter _yLcs1;
    floatParameter _zLcs1;

    floatParameter _xLcs2;
    floatParameter _yLcs2;
    floatParameter _zLcs2;

    friend class CorrespondencesSet;
};

class LocalCoordinates2ImageCorrespondence : public DataBlock
{
    Q_OBJECT
public:

    explicit LocalCoordinates2ImageCorrespondence(CorrespondencesSet* parent = nullptr);

    qint64 attachedLcsId() const;
    void setAttachedLcsId(qint64 id);
    QString attachedLcsName() const;
    LocalCoordinateSystem* attachedLcs() const;

    qint64 attachedImgId() const;
    void setAttachedImgId(qint64 id);
    QString attachedImgName() const;
    Image* attachedImg() const;

    floatParameter xLcs() const;
    void setXLcs(const floatParameter &x);
    void setXLcs(float x);

    floatParameter yLcs() const;
    void setYLcs(const floatParameter &y);
    void setYLcs(float y);

    floatParameter zLcs() const;
    void setZLcs(const floatParameter &z);
    void setZLcs(float z);

    void setLcsCoordinates(QVector3D const& point);
    QVector3D lcsCoordinates() const;

    floatParameter xImg() const;
    void setXImg(const floatParameter &x);
    void setXImg(float x);

    floatParameter yImg() const;
    void setYImg(const floatParameter &y);
    void setYImg(float y);

    void setImageCoordinates(QPointF const& point);
    QPointF imageCoordinates() const;

Q_SIGNALS:
    void attachedLcsIdChanged(qint64 id);
    void attachedImgIdChanged(qint64 id);

    void lcsxCoordChanged(floatParameter);
    void lcsyCoordChanged(floatParameter);
    void lcszCoordChanged(floatParameter);

    void imgxCoordChanged(floatParameter);
    void imgyCoordChanged(floatParameter);

    void coordsChanged();

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void referedCleared(QVector<qint64> const& referedId) override;

    qint64 _attachedLcsId;
    qint64 _attachedImgId;

    floatParameter _xLcs;
    floatParameter _yLcs;
    floatParameter _zLcs;

    floatParameter _xImg;
    floatParameter _yImg;

    friend class CorrespondencesSet;
};


class CorrespondencesSetFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit CorrespondencesSetFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORRESPONDANCESSET_H
