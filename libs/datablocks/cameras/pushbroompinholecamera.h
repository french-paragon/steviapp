#ifndef STEREOVISIONAPP_PUSHBROOMPINHOLECAMERA_H
#define STEREOVISIONAPP_PUSHBROOMPINHOLECAMERA_H


#include "../project.h"
#include "../floatparameter.h"

namespace StereoVisionApp {

class PushBroomPinholeCamera : public DataBlock
{
    Q_OBJECT
public:
    PushBroomPinholeCamera(Project* parent = nullptr);

    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

    int imWidth() const;
    void setImWidth(int width);

    floatParameter fLen() const;
    void setFLen(const floatParameter &f_pix);

    floatParameter opticalCenterX() const;
    void setOpticalCenterX(const floatParameter &c_x);

    floatParameter a0() const;
    void setA0(const floatParameter &a0);

    floatParameter a1() const;
    void setA1(const floatParameter &a1);

    floatParameter a2() const;
    void setA2(const floatParameter &a2);

    floatParameter a3() const;
    void setA3(const floatParameter &a3);

    floatParameter a4() const;
    void setA4(const floatParameter &a4);

    floatParameter a5() const;
    void setA5(const floatParameter &a5);

    floatParameter b0() const;
    void setB0(const floatParameter &b0);

    floatParameter b1() const;
    void setB1(const floatParameter &b1);

    floatParameter b2() const;
    void setB2(const floatParameter &b2);

    floatParameter b3() const;
    void setB3(const floatParameter &b3);

    floatParameter b4() const;
    void setB4(const floatParameter &b4);

    floatParameter b5() const;
    void setB5(const floatParameter &b5);


    StereoVisionApp::floatParameter optimizedFLen() const;
    void setOptimizedFLen(StereoVisionApp::floatParameter const& o_f_pix);
    StereoVisionApp::floatParameter optimizedOpticalCenterX() const;
    void setOptimizedOpticalCenterX(StereoVisionApp::floatParameter const& o_c_x);

    StereoVisionApp::floatParameter optimizedA0() const;
    void setOptimizedA0(StereoVisionApp::floatParameter const& o_a0);
    StereoVisionApp::floatParameter optimizedA1() const;
    void setOptimizedA1(StereoVisionApp::floatParameter const& o_a1);
    StereoVisionApp::floatParameter optimizedA2() const;
    void setOptimizedA2(StereoVisionApp::floatParameter const& o_a2);
    StereoVisionApp::floatParameter optimizedA3() const;
    void setOptimizedA3(StereoVisionApp::floatParameter const& o_a3);
    StereoVisionApp::floatParameter optimizedA4() const;
    void setOptimizedA4(StereoVisionApp::floatParameter const& o_a4);
    StereoVisionApp::floatParameter optimizedA5() const;
    void setOptimizedA5(StereoVisionApp::floatParameter const& o_a5);

    StereoVisionApp::floatParameter optimizedB0() const;
    void setOptimizedB0(StereoVisionApp::floatParameter const& o_b0);
    StereoVisionApp::floatParameter optimizedB1() const;
    void setOptimizedB1(StereoVisionApp::floatParameter const& o_b1);
    StereoVisionApp::floatParameter optimizedB2() const;
    void setOptimizedB2(StereoVisionApp::floatParameter const& o_b2);
    StereoVisionApp::floatParameter optimizedB3() const;
    void setOptimizedB3(StereoVisionApp::floatParameter const& o_b3);
    StereoVisionApp::floatParameter optimizedB4() const;
    void setOptimizedB4(StereoVisionApp::floatParameter const& o_b4);
    StereoVisionApp::floatParameter optimizedB5() const;
    void setOptimizedB5(StereoVisionApp::floatParameter const& o_b5);

    /*!
     * \brief getSensorViewDirections return the view vector, in sensor frame, for all pixels
     * \param optimized if true use optimized parameters, else used basic parameters.
     * \return the list of view directions (in sensor frame).
     */
    std::vector<std::array<double, 3>> getSensorViewDirections(bool optimized=true);
    std::array<double, 3> getSensorViewDirection(float u, bool optimized=true);

Q_SIGNALS:

    void imWidthChanged(int);

    void opticalParametersChanged();

    void fLenChanged(StereoVisionApp::floatParameter);
    void opticalCenterXChanged(StereoVisionApp::floatParameter);

    void a0Changed(StereoVisionApp::floatParameter);
    void a1Changed(StereoVisionApp::floatParameter);
    void a2Changed(StereoVisionApp::floatParameter);
    void a3Changed(StereoVisionApp::floatParameter);
    void a4Changed(StereoVisionApp::floatParameter);
    void a5Changed(StereoVisionApp::floatParameter);

    void b0Changed(StereoVisionApp::floatParameter);
    void b1Changed(StereoVisionApp::floatParameter);
    void b2Changed(StereoVisionApp::floatParameter);
    void b3Changed(StereoVisionApp::floatParameter);
    void b4Changed(StereoVisionApp::floatParameter);
    void b5Changed(StereoVisionApp::floatParameter);

    void optimizedOpticalParametersChanged();

    void optimizedFLenChanged(StereoVisionApp::floatParameter);
    void optimizedOpticalCenterXChanged(StereoVisionApp::floatParameter);

    void optimizedA0Changed(StereoVisionApp::floatParameter);
    void optimizedA1Changed(StereoVisionApp::floatParameter);
    void optimizedA2Changed(StereoVisionApp::floatParameter);
    void optimizedA3Changed(StereoVisionApp::floatParameter);
    void optimizedA4Changed(StereoVisionApp::floatParameter);
    void optimizedA5Changed(StereoVisionApp::floatParameter);

    void optimizedB0Changed(StereoVisionApp::floatParameter);
    void optimizedB1Changed(StereoVisionApp::floatParameter);
    void optimizedB2Changed(StereoVisionApp::floatParameter);
    void optimizedB3Changed(StereoVisionApp::floatParameter);
    void optimizedB4Changed(StereoVisionApp::floatParameter);
    void optimizedB5Changed(StereoVisionApp::floatParameter);

protected:

    virtual QJsonObject encodeJson() const override;
    virtual void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    int _im_width;

    //Basic parameters

    StereoVisionApp::floatParameter _f_pix;
    StereoVisionApp::floatParameter _c_x;

    //Horizontal distortions
    StereoVisionApp::floatParameter _a0;
    StereoVisionApp::floatParameter _a1;
    StereoVisionApp::floatParameter _a2;
    StereoVisionApp::floatParameter _a3;
    StereoVisionApp::floatParameter _a4;
    StereoVisionApp::floatParameter _a5;

    //vertical distortions
    StereoVisionApp::floatParameter _b0;
    StereoVisionApp::floatParameter _b1;
    StereoVisionApp::floatParameter _b2;
    StereoVisionApp::floatParameter _b3;
    StereoVisionApp::floatParameter _b4;
    StereoVisionApp::floatParameter _b5;

    //optimized parameters

    StereoVisionApp::floatParameter _o_f_pix;
    StereoVisionApp::floatParameter _o_c_x;

    //Horizontal distortions
    StereoVisionApp::floatParameter _o_a0;
    StereoVisionApp::floatParameter _o_a1;
    StereoVisionApp::floatParameter _o_a2;
    StereoVisionApp::floatParameter _o_a3;
    StereoVisionApp::floatParameter _o_a4;
    StereoVisionApp::floatParameter _o_a5;

    //vertical distortions
    StereoVisionApp::floatParameter _o_b0;
    StereoVisionApp::floatParameter _o_b1;
    StereoVisionApp::floatParameter _o_b2;
    StereoVisionApp::floatParameter _o_b3;
    StereoVisionApp::floatParameter _o_b4;
    StereoVisionApp::floatParameter _o_b5;

};

class PushBroomPinholeCameraFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit PushBroomPinholeCameraFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const override;
    virtual FactorizableFlags factorizable() const override;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const override;

    virtual QString itemClassName() const override;
    static QString cameraClassName();
    virtual QString proxyClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_PUSHBROOMPINHOLECAMERA_H
