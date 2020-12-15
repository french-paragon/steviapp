#ifndef STEREOVISIONAPP_CAMERA_H
#define STEREOVISIONAPP_CAMERA_H

#include <QSize>

#include "./project.h"
#include "./floatparameter.h"

namespace StereoVisionApp {

class Camera : public DataBlock
{
	Q_OBJECT
public:
	explicit Camera(Project* parent = nullptr);
	explicit Camera(QSize imSize, Project* parent = nullptr);

	QSize imSize() const;
	void setImSize(const QSize &imSize);

	floatParameter fLen() const;
	float fLenY() const;
	void setFLen(const floatParameter &f_pix);

	floatParameter pixelRatio() const;
	void setPixelRatio(const floatParameter &pix_ratio);

	floatParameter opticalCenterX() const;
	void setOpticalCenterX(const floatParameter &c_x);

	floatParameter opticalCenterY() const;
	void setOpticalCenterY(const floatParameter &c_y);

	bool useRadialDistortionModel() const;
	void setUseRadialDistortionModel(bool useRadialDistortionModel);

	floatParameter k1() const;
	void setK1(const floatParameter &k1);

	floatParameter k2() const;
	void setK2(const floatParameter &k2);

	floatParameter k3() const;
	void setK3(const floatParameter &k3);

	bool useDenominatorRadialCoeffs() const;
	void setUseDenominatorRadialCoeffs(bool useDenominatorRadialCoeffs);

	floatParameter k4() const;
	void setK4(const floatParameter &k4);

	floatParameter k5() const;
	void setK5(const floatParameter &k5);

	floatParameter k6() const;
	void setK6(const floatParameter &k6);

	bool useTangentialDistortionModel() const;
	void setUseTangentialDistortionModel(bool useTangentialDistortionModel);

	floatParameter p1() const;
	void setP1(const floatParameter &p1);

	floatParameter p2() const;
	void setP2(const floatParameter &p2);

	bool useSkewDistortionModel() const;
	void setUseSkewDistortionModel(bool useSkewDistortionModel);

	floatParameter B1() const;
	void setB1(const floatParameter &B1);

	floatParameter B2() const;
	void setB2(const floatParameter &B2);

	void clearOptimized() override;
	bool hasOptimizedParameters() const override;

	floatParameter optimizedFLen() const;
	void setOptimizedFLen(const floatParameter &o_f_pix);

	floatParameter optimizedPixelRatio() const;
	void setOptimizedPixelRatio(const floatParameter &o_pix_ratio);

	floatParameter optimizedOpticalCenterX() const;
	void setOptimizedOpticalCenterX(const floatParameter &o_c_x);

	floatParameter optimizedOpticalCenterY() const;
	void setOptimizedOpticalCenterY(const floatParameter &o_c_y);

	floatParameter optimizedK1() const;
	void setOptimizedK1(const floatParameter &o_k1);

	floatParameter optimizedK2() const;
	void setOptimizedK2(const floatParameter &o_k2);

	floatParameter optimizedK3() const;
	void setOptimizedK3(const floatParameter &o_k3);

	floatParameter optimizedK4() const;
	void setOptimizedK4(const floatParameter &o_k4);

	floatParameter optimizedK5() const;
	void setOptimizedK5(const floatParameter &o_k5);

	floatParameter optimizedK6() const;
	void setOptimizedK6(const floatParameter &o_k6);

	floatParameter optimizedP1() const;
	void setOptimizedP1(const floatParameter &o_p1);

	floatParameter optimizedP2() const;
	void setOptimizedP2(const floatParameter &o_p2);

	floatParameter optimizedB1() const;
	void setOptimizedB1(const floatParameter &o_B1);

	floatParameter optimizedB2() const;
	void setOptimizedB2(const floatParameter &o_B2);

Q_SIGNALS:

	void imSizeChanged();

	void FLenChanged(floatParameter);
	void pixelRatioChanged(floatParameter);

	void opticalCenterXChanged(floatParameter);
	void opticalCenterYChanged(floatParameter);

	void useRadialDistortionChanged(bool);

	void k1Changed(floatParameter);
	void k2Changed(floatParameter);
	void k3Changed(floatParameter);

	void useDenominatorRadialCoeffsChanged(bool);

	void k4Changed(floatParameter);
	void k5Changed(floatParameter);
	void k6Changed(floatParameter);

	void useTangentialDistortionChanged(bool);

	void p1Changed(floatParameter);
	void p2Changed(floatParameter);

	void useSkewDistortionChanged(bool);

	void B1Changed(floatParameter);
	void B2Changed(floatParameter);


	void optimizedFLenChanged(floatParameter);
	void optimizedPixelRatioChanged(floatParameter);

	void optimizedOpticalCenterXChanged(floatParameter);
	void optimizedOpticalCenterYChanged(floatParameter);

	void optimizedK1Changed(floatParameter);
	void optimizedK2Changed(floatParameter);
	void optimizedK3Changed(floatParameter);

	void optimizedK4Changed(floatParameter);
	void optimizedK5Changed(floatParameter);
	void optimizedK6Changed(floatParameter);

	void optimizedP1Changed(floatParameter);
	void optimizedP2Changed(floatParameter);

	void optimizedB1Changed(floatParameter);
	void optimizedB2Changed(floatParameter);

protected:

	QSize _imSize;

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _f_pix;

	floatParameter _pix_ratio;

	floatParameter _c_x;
	floatParameter _c_y;

	bool _useRadialDistortionModel;

	floatParameter _k1;
	floatParameter _k2;
	floatParameter _k3;

	bool _useDenominatorRadialCoeffs;

	floatParameter _k4;
	floatParameter _k5;
	floatParameter _k6;

	bool _useTangentialDistortionModel;

	floatParameter _p1;
	floatParameter _p2;

	bool _useSkewDistortionModel;

	floatParameter _B1;
	floatParameter _B2;


	floatParameter _o_f_pix;

	floatParameter _o_pix_ratio;

	floatParameter _o_c_x;
	floatParameter _o_c_y;

	floatParameter _o_k1;
	floatParameter _o_k2;
	floatParameter _o_k3;

	floatParameter _o_k4;
	floatParameter _o_k5;
	floatParameter _o_k6;

	floatParameter _o_p1;
	floatParameter _o_p2;

	floatParameter _o_B1;
	floatParameter _o_B2;

};

class CameraFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit CameraFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
	static QString cameraClassName();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMERA_H
