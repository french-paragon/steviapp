#ifndef STEREOVISIONAPP_STEREORIG_H
#define STEREOVISIONAPP_STEREORIG_H

#include <QObject>

#include "./project.h"
#include "./floatparameter.h"


namespace StereoVisionApp {

class ImagePair;

class StereoRig : public DataBlock
{
	Q_OBJECT
public:

	static const QString StereoRigClassName;

	StereoRig(Project* parent = nullptr);

	floatParameter offsetX() const;
	floatParameter offsetY() const;
	floatParameter offsetZ() const;

	void setOffsetX(const floatParameter &offset_x);
	void setOffsetY(const floatParameter &offset_y);
	void setOffsetZ(const floatParameter &offset_z);

	floatParameter offsetRotX() const;
	floatParameter offsetRotY() const;
	floatParameter offsetRotZ() const;

	void setOffsetRotX(const floatParameter &offset_rx);
	void setOffsetRotY(const floatParameter &offset_ry);
	void setOffsetRotZ(const floatParameter &offset_rz);

	floatParameterGroup<3> optOffset() const;
	void setOptOffset(floatParameterGroup<3> const& o_pos);
	void clearOptOffset();

	floatParameterGroup<3> optOffsetRot() const;
	void setOptOffsetRot(floatParameterGroup<3> const& o_rot);
	void clearOptOffsetRot();

	ImagePair* getImagePair(qint64 id) const;
	qint64 insertImagePair(qint64 cam1ImgId, qint64 cam2ImgId);
	ImagePair* getPairForImage(qint64 id) const;

Q_SIGNALS:

	void offsetXChanged(floatParameter);
	void offsetYChanged(floatParameter);
	void offsetZChanged(floatParameter);

	void offsetRotXChanged(floatParameter);
	void offsetRotYChanged(floatParameter);
	void offsetRotZChanged(floatParameter);

	void optOffsetChanged();
	void optOffsetRotChanged();

	void imagePairAdded(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _offset_x;
	floatParameter _offset_y;
	floatParameter _offset_z;

	floatParameter _offset_rx;
	floatParameter _offset_ry;
	floatParameter _offset_rz;

	floatParameterGroup<3> _o_offset;
	floatParameterGroup<3> _o_offsetrot;
};

class ImagePair : public DataBlock
{
	Q_OBJECT
public:

	static const QString ImagePairClassName;

	explicit ImagePair(StereoRig* parent = nullptr);

	QString nameCam1() const;
	qint64 idImgCam1() const;
	void setidImgCam1(const qint64 &id_imgCam1);

	QString nameCam2() const;
	qint64 idImgCam2() const;
	void setIdImgCam2(const qint64 &id_imgCam2);

Q_SIGNALS:

	void attachedCam1Changed(quint64 id);
	void attachedCam2Changed(quint64 id);

	void attachedCam1NameChanged();
	void attachedCam2NameChanged();

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void referedCleared(QVector<qint64> const& referedId) override;

	qint64 _id_imgCam1;
	qint64 _id_imgCam2;

	friend class StereoRig;

};

class StereoRigFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit StereoRigFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
	static QString StereoRigClassName();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_STEREORIG_H
