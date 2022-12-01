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
	QJsonObject getJsonRepresentation() const override;

	/*!
	 * \brief setParametersFromJsonRepresenation set the parameters from a json representation
	 * \param rep the json object with the parameters.
	 *
	 * N.B this function is not the one used to load the datablock from a project, it is instead use for import/export of stereo rigs.
	 */
	void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

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
