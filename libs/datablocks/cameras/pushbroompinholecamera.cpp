#include "pushbroompinholecamera.h"

#include "itemdatamodel.h"

#include "../camera.h"

namespace StereoVisionApp {

PushBroomPinholeCamera::PushBroomPinholeCamera(Project *parent) :
    DataBlock(parent),
    _im_width(0)
{
    extendDataModel();

    connect(this, &PushBroomPinholeCamera::fLenChanged, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::opticalCenterXChanged, this, &PushBroomPinholeCamera::opticalParametersChanged);

    connect(this, &PushBroomPinholeCamera::a0Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::a1Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::a2Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::a3Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::a4Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::a5Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);

    connect(this, &PushBroomPinholeCamera::b0Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::b1Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::b2Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::b3Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::b4Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::b5Changed, this, &PushBroomPinholeCamera::opticalParametersChanged);

    connect(this, &PushBroomPinholeCamera::optimizedFLenChanged, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedOpticalCenterXChanged, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);

    connect(this, &PushBroomPinholeCamera::optimizedA0Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedA1Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedA2Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedA3Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedA4Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedA5Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);

    connect(this, &PushBroomPinholeCamera::optimizedB0Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedB1Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedB2Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedB3Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedB4Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
    connect(this, &PushBroomPinholeCamera::optimizedB5Changed, this, &PushBroomPinholeCamera::optimizedOpticalParametersChanged);
}

void PushBroomPinholeCamera::clearOptimized() {
    setOptimizedFLen(floatParameter());
    setOptimizedOpticalCenterX(floatParameter());

    setOptimizedA0(floatParameter());
    setOptimizedA1(floatParameter());
    setOptimizedA2(floatParameter());
    setOptimizedA3(floatParameter());
    setOptimizedA4(floatParameter());
    setOptimizedA5(floatParameter());

    setOptimizedB0(floatParameter());
    setOptimizedB1(floatParameter());
    setOptimizedB2(floatParameter());
    setOptimizedB3(floatParameter());
    setOptimizedB4(floatParameter());
    setOptimizedB5(floatParameter());
}
bool PushBroomPinholeCamera::hasOptimizedParameters() const {

    bool isSet = _o_f_pix.isSet();
    isSet = isSet or _o_c_x.isSet();

    isSet = isSet or _o_a0.isSet();
    isSet = isSet or _o_a1.isSet();
    isSet = isSet or _o_a2.isSet();
    isSet = isSet or _o_a3.isSet();
    isSet = isSet or _o_a4.isSet();
    isSet = isSet or _o_a5.isSet();

    isSet = isSet or _o_b0.isSet();
    isSet = isSet or _o_b1.isSet();
    isSet = isSet or _o_b2.isSet();
    isSet = isSet or _o_b3.isSet();
    isSet = isSet or _o_b4.isSet();
    isSet = isSet or _o_b5.isSet();

    return isSet;
}

int PushBroomPinholeCamera::imWidth() const {
    return _im_width;
}
void PushBroomPinholeCamera::setImWidth(int width) {
    if (_im_width != width) {
        _im_width = width;
        Q_EMIT imWidthChanged(width);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::fLen() const {
    return _f_pix;
}
void PushBroomPinholeCamera::setFLen(const floatParameter &f_pix) {
    if (_f_pix != f_pix) {
        _f_pix = f_pix;
        Q_EMIT fLenChanged(_f_pix);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::opticalCenterX() const {
    return _c_x;
}
void PushBroomPinholeCamera::setOpticalCenterX(const floatParameter &c_x) {
    if (_c_x != c_x) {
        _c_x = c_x;
        Q_EMIT opticalCenterXChanged(_c_x);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a0() const {
    return _a0;
}
void PushBroomPinholeCamera::setA0(const floatParameter &a0) {
    if (_a0 != a0) {
        _a0 = a0;
        Q_EMIT a0Changed(_a0);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a1() const {
    return _a1;
}
void PushBroomPinholeCamera::setA1(const floatParameter &a1) {
    if (_a1 != a1) {
        _a1 = a1;
        Q_EMIT a1Changed(_a1);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a2() const {
    return _a2;
}
void PushBroomPinholeCamera::setA2(const floatParameter &a2) {
    if (_a2 != a2) {
        _a2 = a2;
        Q_EMIT a2Changed(_a2);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a3() const {
    return _a3;
}
void PushBroomPinholeCamera::setA3(const floatParameter &a3) {
    if (_a3 != a3) {
        _a3 = a3;
        Q_EMIT a3Changed(_a3);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a4() const {
    return _a4;
}
void PushBroomPinholeCamera::setA4(const floatParameter &a4) {
    if (_a4 != a4) {
        _a4 = a4;
        Q_EMIT a4Changed(_a4);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::a5() const {
    return _a5;
}
void PushBroomPinholeCamera::setA5(const floatParameter &a5) {
    if (_a5 != a5) {
        _a5 = a5;
        Q_EMIT a5Changed(_a5);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b0() const {
    return _b0;
}
void PushBroomPinholeCamera::setB0(const floatParameter &b0) {
    if (_b0 != b0) {
        _b0 = b0;
        Q_EMIT b0Changed(_b0);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b1() const {
    return _b1;
}
void PushBroomPinholeCamera::setB1(const floatParameter &b1) {
    if (_b1 != b1) {
        _b1 = b1;
        Q_EMIT b1Changed(_b1);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b2() const {
    return _b2;
}
void PushBroomPinholeCamera::setB2(const floatParameter &b2) {
    if (_b2 != b2) {
        _b2 = b2;
        Q_EMIT b2Changed(_b2);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b3() const {
    return _b3;
}
void PushBroomPinholeCamera::setB3(const floatParameter &b3) {
    if (_b3 != b3) {
        _b3 = b3;
        Q_EMIT b3Changed(_b3);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b4() const {
    return _b4;
}
void PushBroomPinholeCamera::setB4(const floatParameter &b4) {
    if (_b4 != b4) {
        _b4 = b4;
        Q_EMIT b4Changed(_b4);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::b5() const {
    return _b5;
}
void PushBroomPinholeCamera::setB5(const floatParameter &b5) {
    if (_b5 != b5) {
        _b5 = b5;
        Q_EMIT b5Changed(_b5);
        isChanged();
    }
}


floatParameter PushBroomPinholeCamera::optimizedFLen() const {
    return _o_f_pix;
}
void PushBroomPinholeCamera::setOptimizedFLen(floatParameter const& o_f_pix) {
    if (_o_f_pix != o_f_pix) {
        _o_f_pix = o_f_pix;
        Q_EMIT optimizedFLenChanged(_o_f_pix);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedOpticalCenterX() const {
    return _o_c_x;
}
void PushBroomPinholeCamera::setOptimizedOpticalCenterX(floatParameter const& o_c_x) {
    if (_o_c_x != o_c_x) {
        _o_c_x = o_c_x;
        Q_EMIT optimizedOpticalCenterXChanged(_o_c_x);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::optimizedA0() const {
    return _o_a0;
}
void PushBroomPinholeCamera::setOptimizedA0(floatParameter const& o_a0) {
    if (_o_a0 != o_a0) {
        _o_a0 = o_a0;
        Q_EMIT optimizedA0Changed(_o_a0);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedA1() const {
    return _o_a1;
}
void PushBroomPinholeCamera::setOptimizedA1(floatParameter const& o_a1) {
    if (_o_a1 != o_a1) {
        _o_a1 = o_a1;
        Q_EMIT optimizedA1Changed(_o_a1);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedA2() const {
    return _o_a2;
}
void PushBroomPinholeCamera::setOptimizedA2(floatParameter const& o_a2) {
    if (_o_a2 != o_a2) {
        _o_a2 = o_a2;
        Q_EMIT optimizedA2Changed(_o_a2);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedA3() const {
    return _o_a3;
}
void PushBroomPinholeCamera::setOptimizedA3(floatParameter const& o_a3) {
    if (_o_a3 != o_a3) {
        _o_a3 = o_a3;
        Q_EMIT optimizedA3Changed(_o_a3);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedA4() const {
    return _o_a4;
}
void PushBroomPinholeCamera::setOptimizedA4(floatParameter const& o_a4) {
    if (_o_a4 != o_a4) {
        _o_a4 = o_a4;
        Q_EMIT optimizedA4Changed(_o_a4);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedA5() const {
    return _o_a5;
}
void PushBroomPinholeCamera::setOptimizedA5(floatParameter const& o_a5) {
    if (_o_a5 != o_a5) {
        _o_a5 = o_a5;
        Q_EMIT optimizedA5Changed(_o_a5);
        isChanged();
    }
}

floatParameter PushBroomPinholeCamera::optimizedB0() const {
    return _o_b0;
}
void PushBroomPinholeCamera::setOptimizedB0(floatParameter const& o_b0) {
    if (_o_b0 != o_b0) {
        _o_b0 = o_b0;
        Q_EMIT optimizedB0Changed(_o_b0);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedB1() const {
    return _o_b1;
}
void PushBroomPinholeCamera::setOptimizedB1(floatParameter const& o_b1) {
    if (_o_b1 != o_b1) {
        _o_b1 = o_b1;
        Q_EMIT optimizedB1Changed(_o_b1);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedB2() const {
    return _o_b2;
}
void PushBroomPinholeCamera::setOptimizedB2(floatParameter const& o_b2) {
    if (_o_b2 != o_b2) {
        _o_b2 = o_b2;
        Q_EMIT optimizedB2Changed(_o_b2);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedB3() const {
    return _o_b3;
}
void PushBroomPinholeCamera::setOptimizedB3(floatParameter const& o_b3) {
    if (_o_b3 != o_b3) {
        _o_b3 = o_b3;
        Q_EMIT optimizedB3Changed(_o_b3);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedB4() const {
    return _o_b4;
}
void PushBroomPinholeCamera::setOptimizedB4(floatParameter const& o_b4) {
    if (_o_b4 != o_b4) {
        _o_b4 = o_b4;
        Q_EMIT optimizedB4Changed(_o_b4);
        isChanged();
    }
}
floatParameter PushBroomPinholeCamera::optimizedB5() const {
    return _o_b5;
}
void PushBroomPinholeCamera::setOptimizedB5(floatParameter const& o_b5) {
    if (_o_b5 != o_b5) {
        _o_b5 = o_b5;
        Q_EMIT optimizedB5Changed(_o_b5);
        isChanged();
    }
}


QJsonObject PushBroomPinholeCamera::encodeJson() const {

    QJsonObject obj;

    //camera parameter
    obj.insert("im_width", imWidth());

    //initial parameters
    obj.insert("f", StereoVisionApp::floatParameter::toJson(fLen()));
    obj.insert("cx", StereoVisionApp::floatParameter::toJson(opticalCenterX()));

    obj.insert("a0", StereoVisionApp::floatParameter::toJson(a0()));
    obj.insert("a1", StereoVisionApp::floatParameter::toJson(a1()));
    obj.insert("a2", StereoVisionApp::floatParameter::toJson(a2()));
    obj.insert("a3", StereoVisionApp::floatParameter::toJson(a3()));
    obj.insert("a4", StereoVisionApp::floatParameter::toJson(a4()));
    obj.insert("a5", StereoVisionApp::floatParameter::toJson(a5()));

    obj.insert("b0", StereoVisionApp::floatParameter::toJson(b0()));
    obj.insert("b1", StereoVisionApp::floatParameter::toJson(b1()));
    obj.insert("b2", StereoVisionApp::floatParameter::toJson(b2()));
    obj.insert("b3", StereoVisionApp::floatParameter::toJson(b3()));
    obj.insert("b4", StereoVisionApp::floatParameter::toJson(b4()));
    obj.insert("b5", StereoVisionApp::floatParameter::toJson(b5()));

    //optimized parameters

    obj.insert("of", StereoVisionApp::floatParameter::toJson(optimizedFLen()));
    obj.insert("ocx", StereoVisionApp::floatParameter::toJson(optimizedOpticalCenterX()));

    obj.insert("oa0", StereoVisionApp::floatParameter::toJson(optimizedA0()));
    obj.insert("oa1", StereoVisionApp::floatParameter::toJson(optimizedA1()));
    obj.insert("oa2", StereoVisionApp::floatParameter::toJson(optimizedA2()));
    obj.insert("oa3", StereoVisionApp::floatParameter::toJson(optimizedA3()));
    obj.insert("oa4", StereoVisionApp::floatParameter::toJson(optimizedA4()));
    obj.insert("oa5", StereoVisionApp::floatParameter::toJson(optimizedA5()));

    obj.insert("ob0", StereoVisionApp::floatParameter::toJson(optimizedB0()));
    obj.insert("ob1", StereoVisionApp::floatParameter::toJson(optimizedB1()));
    obj.insert("ob2", StereoVisionApp::floatParameter::toJson(optimizedB2()));
    obj.insert("ob3", StereoVisionApp::floatParameter::toJson(optimizedB3()));
    obj.insert("ob4", StereoVisionApp::floatParameter::toJson(optimizedB4()));
    obj.insert("ob5", StereoVisionApp::floatParameter::toJson(optimizedB5()));

    return obj;

}

void PushBroomPinholeCamera::configureFromJson(QJsonObject const& data) {

    //camera parameters
    if (data.contains("im_width")) {
        _im_width = data.value("im_width").toInt();
    }

    //initial parameters
    if (data.contains("f")) {
        _f_pix = StereoVisionApp::floatParameter::fromJson(data.value("f").toObject());
    }

    if (data.contains("cx")) {
        _c_x = StereoVisionApp::floatParameter::fromJson(data.value("cx").toObject());
    }



    if (data.contains("a0")) {
        _a0 = StereoVisionApp::floatParameter::fromJson(data.value("a0").toObject());
    }

    if (data.contains("a1")) {
        _a1 = StereoVisionApp::floatParameter::fromJson(data.value("a1").toObject());
    }

    if (data.contains("a2")) {
        _a2 = StereoVisionApp::floatParameter::fromJson(data.value("a2").toObject());
    }

    if (data.contains("a3")) {
        _a3 = StereoVisionApp::floatParameter::fromJson(data.value("a3").toObject());
    }

    if (data.contains("a4")) {
        _a4 = StereoVisionApp::floatParameter::fromJson(data.value("a4").toObject());
    }

    if (data.contains("a5")) {
        _a5 = StereoVisionApp::floatParameter::fromJson(data.value("a5").toObject());
    }



    if (data.contains("b0")) {
        _b0 = StereoVisionApp::floatParameter::fromJson(data.value("b0").toObject());
    }

    if (data.contains("b1")) {
        _b1 = StereoVisionApp::floatParameter::fromJson(data.value("b1").toObject());
    }

    if (data.contains("b2")) {
        _b2 = StereoVisionApp::floatParameter::fromJson(data.value("b2").toObject());
    }

    if (data.contains("b3")) {
        _b3 = StereoVisionApp::floatParameter::fromJson(data.value("b3").toObject());
    }

    if (data.contains("b4")) {
        _b4 = StereoVisionApp::floatParameter::fromJson(data.value("b4").toObject());
    }

    if (data.contains("b5")) {
        _b5 = StereoVisionApp::floatParameter::fromJson(data.value("b5").toObject());
    }

    //optimized parameters

    if (data.contains("of")) {
        _o_f_pix = StereoVisionApp::floatParameter::fromJson(data.value("of").toObject());
    }

    if (data.contains("ocx")) {
        _o_c_x = StereoVisionApp::floatParameter::fromJson(data.value("ocx").toObject());
    }



    if (data.contains("oa0")) {
        _o_a0 = StereoVisionApp::floatParameter::fromJson(data.value("oa0").toObject());
    }

    if (data.contains("oa1")) {
        _o_a1 = StereoVisionApp::floatParameter::fromJson(data.value("oa1").toObject());
    }

    if (data.contains("oa2")) {
        _o_a2 = StereoVisionApp::floatParameter::fromJson(data.value("oa2").toObject());
    }

    if (data.contains("oa3")) {
        _o_a3 = StereoVisionApp::floatParameter::fromJson(data.value("oa3").toObject());
    }

    if (data.contains("oa4")) {
        _o_a4 = StereoVisionApp::floatParameter::fromJson(data.value("oa4").toObject());
    }

    if (data.contains("oa5")) {
        _o_a5 = StereoVisionApp::floatParameter::fromJson(data.value("oa5").toObject());
    }



    if (data.contains("ob0")) {
        _o_b0 = StereoVisionApp::floatParameter::fromJson(data.value("ob0").toObject());
    }

    if (data.contains("ob1")) {
        _o_b1 = StereoVisionApp::floatParameter::fromJson(data.value("ob1").toObject());
    }

    if (data.contains("ob2")) {
        _o_b2 = StereoVisionApp::floatParameter::fromJson(data.value("ob2").toObject());
    }

    if (data.contains("ob3")) {
        _o_b3 = StereoVisionApp::floatParameter::fromJson(data.value("ob3").toObject());
    }

    if (data.contains("ob4")) {
        _o_b4 = StereoVisionApp::floatParameter::fromJson(data.value("ob4").toObject());
    }

    if (data.contains("ob5")) {
        _o_b5 = StereoVisionApp::floatParameter::fromJson(data.value("ob5").toObject());
    }
}

void PushBroomPinholeCamera::extendDataModel() {

    ItemDataModel::Category* p = _dataModel->addCategory(tr("Pinhole properties"));

    p->addCatProperty<int,
            PushBroomPinholeCamera,
            false,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Image width"),
             &PushBroomPinholeCamera::imWidth,
             &PushBroomPinholeCamera::setImWidth,
             &PushBroomPinholeCamera::imWidthChanged);

    ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

    optCat->addCatProperty<bool,
            DataBlock,
            false,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Fixed"),
             &DataBlock::isFixed,
             &DataBlock::setFixed,
             &DataBlock::isFixedChanged);

    ItemDataModel::Category* ip = _dataModel->addCategory(tr("intrisic parameters"));

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Focal length [px]"),
             &PushBroomPinholeCamera::fLen,
             &PushBroomPinholeCamera::setFLen,
             &PushBroomPinholeCamera::fLenChanged);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Optical center X"),
             &PushBroomPinholeCamera::opticalCenterX,
             &PushBroomPinholeCamera::setOpticalCenterX,
             &PushBroomPinholeCamera::opticalCenterXChanged);



    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a0"),
             &PushBroomPinholeCamera::a0,
             &PushBroomPinholeCamera::setA0,
             &PushBroomPinholeCamera::a0Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a1"),
             &PushBroomPinholeCamera::a1,
             &PushBroomPinholeCamera::setA1,
             &PushBroomPinholeCamera::a1Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a2"),
             &PushBroomPinholeCamera::a2,
             &PushBroomPinholeCamera::setA2,
             &PushBroomPinholeCamera::a2Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a3"),
             &PushBroomPinholeCamera::a3,
             &PushBroomPinholeCamera::setA3,
             &PushBroomPinholeCamera::a3Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a4"),
             &PushBroomPinholeCamera::a4,
             &PushBroomPinholeCamera::setA4,
             &PushBroomPinholeCamera::a4Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a5"),
             &PushBroomPinholeCamera::a5,
             &PushBroomPinholeCamera::setA5,
             &PushBroomPinholeCamera::a5Changed);



    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b0"),
             &PushBroomPinholeCamera::b0,
             &PushBroomPinholeCamera::setB0,
             &PushBroomPinholeCamera::b0Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b1"),
             &PushBroomPinholeCamera::b1,
             &PushBroomPinholeCamera::setB1,
             &PushBroomPinholeCamera::b1Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b2"),
             &PushBroomPinholeCamera::b2,
             &PushBroomPinholeCamera::setB2,
             &PushBroomPinholeCamera::b2Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b3"),
             &PushBroomPinholeCamera::b3,
             &PushBroomPinholeCamera::setB3,
             &PushBroomPinholeCamera::b3Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b4"),
             &PushBroomPinholeCamera::b4,
             &PushBroomPinholeCamera::setB4,
             &PushBroomPinholeCamera::b4Changed);

    ip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b5"),
             &PushBroomPinholeCamera::b5,
             &PushBroomPinholeCamera::setB5,
             &PushBroomPinholeCamera::b5Changed);

    ItemDataModel::Category* oip = _dataModel->addCategory(tr("Optimized intrisic parameters"));

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Focal length [px]"),
             &PushBroomPinholeCamera::optimizedFLen,
             nullptr,
             &PushBroomPinholeCamera::optimizedFLenChanged);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Optical center X"),
             &PushBroomPinholeCamera::optimizedOpticalCenterX,
             nullptr,
             &PushBroomPinholeCamera::optimizedOpticalCenterXChanged);



    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a0"),
             &PushBroomPinholeCamera::optimizedA0,
             nullptr,
             &PushBroomPinholeCamera::optimizedA0Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a1"),
             &PushBroomPinholeCamera::optimizedA1,
             nullptr,
             &PushBroomPinholeCamera::optimizedA1Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a2"),
             &PushBroomPinholeCamera::optimizedA2,
             nullptr,
             &PushBroomPinholeCamera::optimizedA2Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a3"),
             &PushBroomPinholeCamera::optimizedA3,
             nullptr,
             &PushBroomPinholeCamera::optimizedA3Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a4"),
             &PushBroomPinholeCamera::optimizedA4,
             nullptr,
             &PushBroomPinholeCamera::optimizedA4Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a5"),
             &PushBroomPinholeCamera::optimizedA5,
             nullptr,
             &PushBroomPinholeCamera::optimizedA5Changed);



    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b0"),
             &PushBroomPinholeCamera::optimizedB0,
             nullptr,
             &PushBroomPinholeCamera::optimizedB0Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b2"),
             &PushBroomPinholeCamera::optimizedB2,
             nullptr,
             &PushBroomPinholeCamera::optimizedB2Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b3"),
             &PushBroomPinholeCamera::optimizedB3,
             nullptr,
             &PushBroomPinholeCamera::optimizedB3Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b4"),
             &PushBroomPinholeCamera::optimizedB4,
             nullptr,
             &PushBroomPinholeCamera::optimizedB4Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b5"),
             &PushBroomPinholeCamera::optimizedB5,
             nullptr,
             &PushBroomPinholeCamera::optimizedB5Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b1"),
             &PushBroomPinholeCamera::optimizedB1,
             nullptr,
             &PushBroomPinholeCamera::optimizedB1Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b2"),
             &PushBroomPinholeCamera::optimizedB2,
             nullptr,
             &PushBroomPinholeCamera::optimizedB2Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b3"),
             &PushBroomPinholeCamera::optimizedB3,
             nullptr,
             &PushBroomPinholeCamera::optimizedB3Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b4"),
             &PushBroomPinholeCamera::optimizedB4,
             nullptr,
             &PushBroomPinholeCamera::optimizedB4Changed);

    oip->addCatProperty<floatParameter,
            PushBroomPinholeCamera,
            true,
            ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b5"),
             &PushBroomPinholeCamera::optimizedB5,
             nullptr,
             &PushBroomPinholeCamera::optimizedB5Changed);

}

PushBroomPinholeCameraFactory::PushBroomPinholeCameraFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString PushBroomPinholeCameraFactory::TypeDescrName() const {
    return tr("Pinhole pushbroom camera");
}

DataBlockFactory::FactorizableFlags PushBroomPinholeCameraFactory::factorizable() const {
    return FactorizableFlag::RootDataBlock;
}

DataBlock* PushBroomPinholeCameraFactory::factorizeDataBlock(Project *parent) const {
    return new PushBroomPinholeCamera(parent);
}

QString PushBroomPinholeCameraFactory::itemClassName() const {
    return PushBroomPinholeCamera::staticMetaObject.className();
}
QString PushBroomPinholeCameraFactory::proxyClassName() const {
    return CameraFactory::cameraClassName();
}

} // namespace StereoVisionApp
