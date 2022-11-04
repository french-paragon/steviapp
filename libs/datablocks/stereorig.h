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


	floatParameter optOffsetX() const;
	floatParameter optOffsetY() const;
	floatParameter optOffsetZ() const;

	floatParameterGroup<3> optOffset() const;
	void setOptOffset(floatParameterGroup<3> const& o_pos);
	void clearOptOffset();


	floatParameter optOffsetRotX() const;
	floatParameter optOffsetRotY() const;
	floatParameter optOffsetRotZ() const;

	floatParameterGroup<3> optOffsetRot() const;
	void setOptOffsetRot(floatParameterGroup<3> const& o_rot);
	void clearOptOffsetRot();


	ImagePair* getImagePair(qint64 id) const;
	qint64 insertImagePair(qint64 cam1ImgId, qint64 cam2ImgId);
	ImagePair* getPairForImage(qint64 id) const;

	/*!
	 * \brief getJsonRepresentation return a JsonObject containing the parameters of the stereo rigs (usefull to export a calibrated stereo rigs).
	 * \return a JsonObject containing the camera parameters.
	 *
	 * N.B this function is not the one used to save the datablock in a project, it is instead use for import/export of stereo rigs.
	 */
	inline QJsonObject getJsonRepresentation() const {

		QJsonObject obj;

		obj.insert("ofsx", floatParameter::toJson(offsetX()));
		obj.insert("ofsy", floatParameter::toJson(offsetY()));
		obj.insert("ofsz", floatParameter::toJson(offsetZ()));

		obj.insert("ofsrx", floatParameter::toJson(offsetRotX()));
		obj.insert("ofsry", floatParameter::toJson(offsetRotY()));
		obj.insert("ofsrz", floatParameter::toJson(offsetRotZ()));

		obj.insert("oofs", floatParameterGroup<3>::toJson(optOffset()));

		obj.insert("oofsr", floatParameterGroup<3>::toJson(optOffsetRot()));

		return obj;
	}

	/*!
	 * \brief setParametersFromJsonRepresenation set the parameters from a json representation
	 * \param rep the json object with the parameters.
	 *
	 * N.B this function is not the one used to load the datablock from a project, it is instead use for import/export of stereo rigs.
	 */
	inline void setParametersFromJsonRepresentation(QJsonObject const& rep) {

		if (rep.contains("ofsx")) {
			_offset_x = floatParameter::fromJson(rep.value("ofsx").toObject());
		}
		if (rep.contains("ofsy")) {
			_offset_y = floatParameter::fromJson(rep.value("ofsy").toObject());
		}
		if (rep.contains("ofsz")) {
			_offset_z = floatParameter::fromJson(rep.value("ofsz").toObject());
		}

		if (rep.contains("ofsrx")) {
			_offset_rx = floatParameter::fromJson(rep.value("ofsrx").toObject());
		}
		if (rep.contains("ofsry")) {
			_offset_ry = floatParameter::fromJson(rep.value("ofsry").toObject());
		}
		if (rep.contains("ofsry")) {
			_offset_rz = floatParameter::fromJson(rep.value("ofsrz").toObject());
		}

		if (rep.contains("oofs")) {
			_o_offset = floatParameterGroup<3>::fromJson(rep.value("oofs").toObject());
		}

		if (rep.contains("oofsr")) {
			_o_offsetrot = floatParameterGroup<3>::fromJson(rep.value("oofsr").toObject());
		}
	}

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
