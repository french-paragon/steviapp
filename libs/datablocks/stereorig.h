#ifndef STEREOVISIONAPP_STEREORIG_H
#define STEREOVISIONAPP_STEREORIG_H

#include <QObject>

#include "./project.h"
#include "./rigidbody.h"
#include "./floatparameter.h"


namespace StereoVisionApp {

class ImagePair;

class StereoRig : public RigidBody
{
	Q_OBJECT
public:

	static const QString StereoRigClassName;

	StereoRig(Project* parent = nullptr);


	ImagePair* getImagePair(qint64 id) const;
	qint64 insertImagePair(qint64 cam1ImgId, qint64 cam2ImgId);
	ImagePair* getPairForImage(qint64 id) const;
	bool removeImagePair(qint64 id);

	/*!
	 * \brief getJsonRepresentation return a JsonObject containing the parameters of the stereo rigs (usefull to export a calibrated stereo rigs).
	 * \return a JsonObject containing the camera parameters.
	 *
	 * N.B this function is not the one used to save the datablock in a project, it is instead use for import/export of stereo rigs.
	 */
	inline QJsonObject getJsonRepresentation() const {

		QJsonObject obj;

		obj.insert("ofsx", floatParameter::toJson(xCoord()));
		obj.insert("ofsy", floatParameter::toJson(yCoord()));
		obj.insert("ofsz", floatParameter::toJson(zCoord()));

		obj.insert("ofsrx", floatParameter::toJson(xRot()));
		obj.insert("ofsry", floatParameter::toJson(yRot()));
		obj.insert("ofsrz", floatParameter::toJson(zRot()));

		obj.insert("oofs", floatParameterGroup<3>::toJson(optPos()));

		obj.insert("oofsr", floatParameterGroup<3>::toJson(optRot()));

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
			setXCoord(floatParameter::fromJson(rep.value("ofsx").toObject()));
		}
		if (rep.contains("ofsy")) {
			setYCoord(floatParameter::fromJson(rep.value("ofsy").toObject()));
		}
		if (rep.contains("ofsz")) {
			setZCoord(floatParameter::fromJson(rep.value("ofsz").toObject()));
		}

		if (rep.contains("ofsrx")) {
			setXRot(floatParameter::fromJson(rep.value("ofsrx").toObject()));
		}
		if (rep.contains("ofsry")) {
			setYRot(floatParameter::fromJson(rep.value("ofsry").toObject()));
		}
		if (rep.contains("ofsry")) {
			setZRot(floatParameter::fromJson(rep.value("ofsrz").toObject()));
		}

		if (rep.contains("oofs")) {
			setOptPos(floatParameterGroup<3>::fromJson(rep.value("oofs").toObject()));
		}

		if (rep.contains("oofsr")) {
			setOptRot(floatParameterGroup<3>::fromJson(rep.value("oofsr").toObject()));
		}
	}

Q_SIGNALS:

	void imagePairAdded(qint64 id);
	void imagePairRemoved(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();
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
