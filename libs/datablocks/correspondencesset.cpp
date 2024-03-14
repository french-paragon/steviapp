#include "correspondencesset.h"

#include "./image.h"
#include "./localcoordinatesystem.h"

#include "./itemdatamodel.h"

namespace StereoVisionApp {

CorrespondencesSet::CorrespondencesSet(Project *parent) :
    DataBlock(parent)
{
    extendDataModel();
}


qint64 CorrespondencesSet::addImagesCorrespondences(qint64 img1Id, const QPointF &img1Coordinates,
                                                    qint64 img2Id, const QPointF &img2Coordinates,
                                                    bool uncertain, float sigma_pos) {

    Image2ImageCorrespondence* iml = new Image2ImageCorrespondence(this);

    iml->stopTrackingChanges(true);

    floatParameter xImg1(img1Coordinates.x(), pFloatType(sigma_pos));
    floatParameter yImg1(img1Coordinates.y(), pFloatType(sigma_pos));

    floatParameter xImg2(img2Coordinates.x(), pFloatType(sigma_pos));
    floatParameter yImg2(img2Coordinates.y(), pFloatType(sigma_pos));

    if (!uncertain) {
        xImg1.clearUncertainty();
        yImg1.clearUncertainty();

        xImg2.clearUncertainty();
        yImg2.clearUncertainty();
    }

    iml->setXImg1(xImg1);
    iml->setYImg1(yImg1);

    iml->setXImg2(xImg2);
    iml->setYImg2(yImg2);

    insertSubItem(iml);

    if (iml->internalId() >= 0) {
        iml->setAttachedImg1Id(img1Id);
        iml->setAttachedImg2Id(img2Id);
        emit im2imAdded(iml->internalId());

        iml->stopTrackingChanges(false);
        return iml->internalId();
    } else {
        iml->clear();
        iml->deleteLater();
    }

    return -1;

}

qint64 CorrespondencesSet::addLocalCoordinatesCorrespondences(qint64 lc1Id, const QVector3D &sys1Coordinates,
                                                              qint64 lc2Id, const QVector3D &sys2Coordinates,
                                                              bool uncertain, float sigma_pos) {

    LocalCoordinates2LocalCoordinatesCorrespondence* lcsc = new LocalCoordinates2LocalCoordinatesCorrespondence(this);

    lcsc->stopTrackingChanges(true);

    floatParameter xLcs1(sys1Coordinates.x(), pFloatType(sigma_pos));
    floatParameter yLcs1(sys1Coordinates.y(), pFloatType(sigma_pos));
    floatParameter zLcs1(sys1Coordinates.z(), pFloatType(sigma_pos));

    floatParameter xLcs2(sys2Coordinates.x(), pFloatType(sigma_pos));
    floatParameter yLcs2(sys2Coordinates.y(), pFloatType(sigma_pos));
    floatParameter zLcs2(sys2Coordinates.y(), pFloatType(sigma_pos));

    if (!uncertain) {
        xLcs1.clearUncertainty();
        yLcs1.clearUncertainty();
        zLcs1.clearUncertainty();

        xLcs2.clearUncertainty();
        yLcs2.clearUncertainty();
        zLcs2.clearUncertainty();
    }

    lcsc->setXLcs1(xLcs1);
    lcsc->setYLcs1(yLcs1);
    lcsc->setZLcs1(zLcs1);

    lcsc->setXLcs2(xLcs2);
    lcsc->setYLcs2(yLcs2);
    lcsc->setZLcs2(zLcs2);

    insertSubItem(lcsc);

    if (lcsc->internalId() >= 0) {
        lcsc->setAttachedLcs1Id(lc1Id);
        lcsc->setAttachedLcs2Id(lc2Id);
        emit lcs2lcsAdded(lcsc->internalId());

        lcsc->stopTrackingChanges(false);
        return lcsc->internalId();
    } else {
        lcsc->clear();
        lcsc->deleteLater();
    }

    return -1;

}

qint64 CorrespondencesSet::addLocalCoordinate2ImageCorrespondences(qint64 lcId, const QVector3D &sysCoordinates,
                                                                   qint64 imgId, const QPointF &imgCoordinates,
                                                                   bool uncertain, float sigma_pos_lcs, float sigma_pos_img) {

    LocalCoordinates2ImageCorrespondence* lcsimc = new LocalCoordinates2ImageCorrespondence(this);

    lcsimc->stopTrackingChanges(true);

    floatParameter xLcs(sysCoordinates.x(), pFloatType(sigma_pos_lcs));
    floatParameter yLcs(sysCoordinates.y(), pFloatType(sigma_pos_lcs));
    floatParameter zLcs(sysCoordinates.z(), pFloatType(sigma_pos_lcs));

    floatParameter xImg(imgCoordinates.x(), pFloatType(sigma_pos_img));
    floatParameter yImg(imgCoordinates.y(), pFloatType(sigma_pos_img));

    if (!uncertain) {
        xLcs.clearUncertainty();
        yLcs.clearUncertainty();
        zLcs.clearUncertainty();

        xImg.clearUncertainty();
        yImg.clearUncertainty();
    }

    lcsimc->setXLcs(xLcs);
    lcsimc->setYLcs(yLcs);
    lcsimc->setZLcs(zLcs);

    lcsimc->setXImg(xImg);
    lcsimc->setYImg(yImg);

    insertSubItem(lcsimc);

    if (lcsimc->internalId() >= 0) {
        lcsimc->setAttachedLcsId(lcId);
        lcsimc->setAttachedImgId(imgId);
        emit lcs2imAdded(lcsimc->internalId());

        lcsimc->stopTrackingChanges(false);
        return lcsimc->internalId();
    } else {
        lcsimc->clear();
        lcsimc->deleteLater();
    }

    return -1;

}

QVector<Image2ImageCorrespondence*> CorrespondencesSet::getAllImagesCorrespondences() const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(Image2ImageCorrespondence::staticMetaObject.className());
    QVector<Image2ImageCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        Image2ImageCorrespondence* corr = qobject_cast<Image2ImageCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}

QVector<Image2ImageCorrespondence*> CorrespondencesSet::getImageCorrespondences(qint64 imgId) const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(Image2ImageCorrespondence::staticMetaObject.className());
    QVector<Image2ImageCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        Image2ImageCorrespondence* corr = qobject_cast<Image2ImageCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        if (corr->attachedImg1Id() != imgId and corr->attachedImg2Id() != imgId) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}

QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> CorrespondencesSet::getAllLocalFramesCorrespondences() const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(LocalCoordinates2LocalCoordinatesCorrespondence::staticMetaObject.className());
    QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        LocalCoordinates2LocalCoordinatesCorrespondence* corr = qobject_cast<LocalCoordinates2LocalCoordinatesCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}
QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> CorrespondencesSet::getLocalFrameCorrespondences(qint64 lcsId) const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(LocalCoordinates2LocalCoordinatesCorrespondence::staticMetaObject.className());
    QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        LocalCoordinates2LocalCoordinatesCorrespondence* corr = qobject_cast<LocalCoordinates2LocalCoordinatesCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        if (corr->attachedLcs1Id() != lcsId and corr->attachedLcs2Id() != lcsId) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}

QVector<LocalCoordinates2ImageCorrespondence*> CorrespondencesSet::getAllImages2LocalCoordinatesCorrespondences() const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(LocalCoordinates2ImageCorrespondence::staticMetaObject.className());
    QVector<LocalCoordinates2ImageCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        LocalCoordinates2ImageCorrespondence* corr = qobject_cast<LocalCoordinates2ImageCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}
QVector<LocalCoordinates2ImageCorrespondence*> CorrespondencesSet::getImage2LocalCoordinatesCorrespondences(qint64 imgId) const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(LocalCoordinates2ImageCorrespondence::staticMetaObject.className());
    QVector<LocalCoordinates2ImageCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        LocalCoordinates2ImageCorrespondence* corr = qobject_cast<LocalCoordinates2ImageCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        if (corr->attachedImgId() != imgId) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}
QVector<LocalCoordinates2ImageCorrespondence*> CorrespondencesSet::getLocalFrame2ImagesCorrespondences(qint64 lcsId) const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(LocalCoordinates2ImageCorrespondence::staticMetaObject.className());
    QVector<LocalCoordinates2ImageCorrespondence*> ret;
    ret.reserve(lmIds.size());

    for (qint64 id : lmIds) {
        LocalCoordinates2ImageCorrespondence* corr = qobject_cast<LocalCoordinates2ImageCorrespondence*>(getById(id));

        if (corr == nullptr) {
            continue;
        }

        if (corr->attachedLcsId() != lcsId) {
            continue;
        }

        ret.push_back(corr);
    }

    return ret;
}


void CorrespondencesSet::clearOptimized() {
    return; //Nothing to optimize
}
bool CorrespondencesSet::hasOptimizedParameters() const {
    return false;
}

QJsonObject CorrespondencesSet::getJsonRepresentation() const {
    return encodeJson();
}
void CorrespondencesSet::setParametersFromJsonRepresentation(QJsonObject const& rep) {
    configureFromJson(rep);
}

QJsonObject CorrespondencesSet::encodeJson() const {

    QJsonObject obj;

    QJsonArray img2img;

    QVector<Image2ImageCorrespondence*> imgCorrespondances = getAllImagesCorrespondences();
    for(Image2ImageCorrespondence* corr : qAsConst(imgCorrespondances)) {
        img2img.push_back(corr->toJson());
    }

    obj.insert("img2img", img2img);

    QJsonArray lcs2lcs;

    QVector<LocalCoordinates2LocalCoordinatesCorrespondence*> lcsCorrespondances = getAllLocalFramesCorrespondences();
    for(LocalCoordinates2LocalCoordinatesCorrespondence* corr : qAsConst(lcsCorrespondances)) {
        lcs2lcs.push_back(corr->toJson());
    }

    obj.insert("lcs2lcs", lcs2lcs);

    QJsonArray lcs2im;

    QVector<LocalCoordinates2ImageCorrespondence*> lcsimCorrespondances = getAllImages2LocalCoordinatesCorrespondences();
    for(LocalCoordinates2ImageCorrespondence* corr : qAsConst(lcsimCorrespondances)) {
        lcs2im.push_back(corr->toJson());
    }

    obj.insert("lcs2im", lcs2im);

    return obj;
}

void CorrespondencesSet::configureFromJson(QJsonObject const& data) {

    if (data.contains("img2img")) {
        QJsonArray arr = data.value("img2img").toArray();

        for (QJsonValue const& v : arr) {
            QJsonObject o = v.toObject();

            Image2ImageCorrespondence* im2im = new Image2ImageCorrespondence(this);
            im2im->setFromJson(o);

            if (im2im->internalId() >= 0) {
                insertSubItem(im2im);
            } else {
                im2im->deleteLater();
            }
        }
    }

    if (data.contains("lcs2lcs")) {
        QJsonArray arr = data.value("lcs2lcs").toArray();

        for (QJsonValue const& v : arr) {
            QJsonObject o = v.toObject();

            LocalCoordinates2LocalCoordinatesCorrespondence* lcs2lcs = new LocalCoordinates2LocalCoordinatesCorrespondence(this);
            lcs2lcs->setFromJson(o);

            if (lcs2lcs->internalId() >= 0) {
                insertSubItem(lcs2lcs);
            } else {
                lcs2lcs->deleteLater();
            }
        }
    }

    if (data.contains("lcs2im")) {
        QJsonArray arr = data.value("lcs2im").toArray();

        for (QJsonValue const& v : arr) {
            QJsonObject o = v.toObject();

            LocalCoordinates2ImageCorrespondence* lcs2im = new LocalCoordinates2ImageCorrespondence(this);
            lcs2im->setFromJson(o);

            if (lcs2im->internalId() >= 0) {
                insertSubItem(lcs2im);
            } else {
                lcs2im->deleteLater();
            }
        }
    }
}

void CorrespondencesSet::extendDataModel() {

    ItemDataModel::SubItemCollectionManager::TitleFunc im2imTitleFunc = [] (DataBlock* b) -> QString {
       Image2ImageCorrespondence* l = qobject_cast<Image2ImageCorrespondence*>(b);
       if (l != nullptr) {
           return l->attachedImage1Name() + " - " + l->attachedImage2Name();
       }
       return tr("Unvalid image 2 image match");
    };

    ItemDataModel::SubItemCollectionManager::TitleFunc lcs2lcsTitleFunc = [] (DataBlock* b) -> QString {
       LocalCoordinates2LocalCoordinatesCorrespondence* l = qobject_cast<LocalCoordinates2LocalCoordinatesCorrespondence*>(b);
       if (l != nullptr) {
           return l->attachedLcs1Name() + " - " + l->attachedLcs2Name();
       }
       return tr("Unvalid local coordinate 2 local coordinate match");
    };

    ItemDataModel::SubItemCollectionManager::TitleFunc lcs2imgTitleFunc = [] (DataBlock* b) -> QString {
       LocalCoordinates2ImageCorrespondence* l = qobject_cast<LocalCoordinates2ImageCorrespondence*>(b);
       if (l != nullptr) {
           return l->attachedLcsName() + " - " + l->attachedImgName();
       }
       return tr("Unvalid local coordinate 2 image match");
    };

    ItemDataModel::SubItemCollectionManager* im2im_c =
            _dataModel->addCollectionManager(tr("Image 2 Image matches"),
                                             Image2ImageCorrespondence::staticMetaObject.className(),
                                             im2imTitleFunc);

    im2im_c->addCatProperty<floatParameter, Image2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos img1"),
             &Image2ImageCorrespondence::xImg1,
             &Image2ImageCorrespondence::setXImg1,
             &Image2ImageCorrespondence::img1xCoordChanged);

    im2im_c->addCatProperty<floatParameter, Image2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos img1"),
             &Image2ImageCorrespondence::yImg1,
             &Image2ImageCorrespondence::setYImg1,
             &Image2ImageCorrespondence::img1yCoordChanged);

    im2im_c->addCatProperty<floatParameter, Image2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos img2"),
             &Image2ImageCorrespondence::xImg2,
             &Image2ImageCorrespondence::setXImg2,
             &Image2ImageCorrespondence::img1xCoordChanged);

    im2im_c->addCatProperty<floatParameter, Image2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos img2"),
             &Image2ImageCorrespondence::yImg2,
             &Image2ImageCorrespondence::setYImg2,
             &Image2ImageCorrespondence::img2yCoordChanged);

    ItemDataModel::SubItemCollectionManager* lcs2lcs_c =
            _dataModel->addCollectionManager(tr("Local coordinate 2 Local coordinate matches"),
                                             LocalCoordinates2LocalCoordinatesCorrespondence::staticMetaObject.className(),
                                             lcs2lcsTitleFunc);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos lcs1"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::xLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1xCoordChanged);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos lcs1"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::yLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1yCoordChanged);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Z pos lcs1"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::zLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs1,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1zCoordChanged);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos lcs2"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::xLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2xCoordChanged);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos lcs2"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::yLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2yCoordChanged);

    lcs2lcs_c->addCatProperty<floatParameter, LocalCoordinates2LocalCoordinatesCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Z pos lcs2"),
             &LocalCoordinates2LocalCoordinatesCorrespondence::zLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs2,
             &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2zCoordChanged);

    ItemDataModel::SubItemCollectionManager* lcs2im_c =
            _dataModel->addCollectionManager(tr("Local coordinate 2 Image matches"),
                                             LocalCoordinates2ImageCorrespondence::staticMetaObject.className(),
                                             lcs2imgTitleFunc);

    lcs2im_c->addCatProperty<floatParameter, LocalCoordinates2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos lcs"),
             &LocalCoordinates2ImageCorrespondence::xLcs,
             &LocalCoordinates2ImageCorrespondence::setXLcs,
             &LocalCoordinates2ImageCorrespondence::lcsxCoordChanged);

    lcs2im_c->addCatProperty<floatParameter, LocalCoordinates2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos lcs"),
             &LocalCoordinates2ImageCorrespondence::yLcs,
             &LocalCoordinates2ImageCorrespondence::setYLcs,
             &LocalCoordinates2ImageCorrespondence::lcsyCoordChanged);

    lcs2im_c->addCatProperty<floatParameter, LocalCoordinates2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Z pos lcs"),
             &LocalCoordinates2ImageCorrespondence::zLcs,
             &LocalCoordinates2ImageCorrespondence::setZLcs,
             &LocalCoordinates2ImageCorrespondence::lcszCoordChanged);

    lcs2im_c->addCatProperty<floatParameter, LocalCoordinates2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X pos img"),
             &LocalCoordinates2ImageCorrespondence::xImg,
             &LocalCoordinates2ImageCorrespondence::setXImg,
             &LocalCoordinates2ImageCorrespondence::imgxCoordChanged);

    lcs2im_c->addCatProperty<floatParameter, LocalCoordinates2ImageCorrespondence, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y pos img"),
             &LocalCoordinates2ImageCorrespondence::yImg,
             &LocalCoordinates2ImageCorrespondence::setYImg,
             &LocalCoordinates2ImageCorrespondence::imgyCoordChanged);

}



Image2ImageCorrespondence::Image2ImageCorrespondence(CorrespondencesSet* parent) :
    DataBlock(parent)
{
    connect(this, &Image2ImageCorrespondence::img1xCoordChanged, this, &Image2ImageCorrespondence::coordsChanged);
    connect(this, &Image2ImageCorrespondence::img1yCoordChanged, this, &Image2ImageCorrespondence::coordsChanged);

    connect(this, &Image2ImageCorrespondence::img2xCoordChanged, this, &Image2ImageCorrespondence::coordsChanged);
    connect(this, &Image2ImageCorrespondence::img2yCoordChanged, this, &Image2ImageCorrespondence::coordsChanged);
}

qint64 Image2ImageCorrespondence::attachedImg1Id() const {
    return _attachedImg1Id;
}

void Image2ImageCorrespondence::setAttachedImg1Id(qint64 id) {

    if (id < 0 and _attachedImg1Id >= 0) {
        removeRefered({_attachedImg1Id});
        _attachedImg1Id = -1;
        emit attachedImg1idChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedImg1Id) {
        return;
    }

    if (_attachedImg1Id >= 0) {
        removeRefered({_attachedImg1Id});
    }
    _attachedImg1Id = id;
    if (_attachedImg1Id >= 0) {
        addRefered({_attachedImg1Id});
    }
    emit attachedImg1idChanged(_attachedImg1Id);
    return;

}

QString Image2ImageCorrespondence::attachedImage1Name() const {
    Image* img1 = attachedImage1();

    if (img1 != nullptr) {
        return img1->objectName();
    }

    return "";
}

Image* Image2ImageCorrespondence::attachedImage1() const {
    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    Image* im = qobject_cast<Image*>(p->getById(attachedImg1Id()));

    return im;
}

qint64 Image2ImageCorrespondence::attachedImg2Id() const {
    return _attachedImg2Id;
}

void Image2ImageCorrespondence::setAttachedImg2Id(qint64 id) {

    if (id < 0 and _attachedImg2Id >= 0) {
        removeRefered({_attachedImg2Id});
        _attachedImg2Id = -1;
        emit attachedImg1idChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedImg2Id) {
        return;
    }

    if (_attachedImg2Id >= 0) {
        removeRefered({_attachedImg2Id});
    }
    _attachedImg2Id = id;
    if (_attachedImg2Id >= 0) {
        addRefered({_attachedImg2Id});
    }
    emit attachedImg1idChanged(_attachedImg2Id);
    return;
}

QString Image2ImageCorrespondence::attachedImage2Name() const {
    Image* img2 = attachedImage2();

    if (img2 != nullptr) {
        return img2->objectName();
    }

    return "";
}

Image* Image2ImageCorrespondence::attachedImage2() const {
    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    Image* im = qobject_cast<Image*>(p->getById(attachedImg2Id()));

    return im;
}

floatParameter Image2ImageCorrespondence::xImg1() const {
    return _xImg1;
}

void Image2ImageCorrespondence::setXImg1(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xImg1, 1e-4)) {
        _xImg1 = x;
        emit img1xCoordChanged(x);
        isChanged();
    }
}

void Image2ImageCorrespondence::setXImg1(float x) {

    if (!_xImg1.isApproximatlyEqual(x, 1e-4) or !_xImg1.isSet()) {
        _xImg1.setIsSet(x);
        emit img1xCoordChanged(_xImg1);
        isChanged();
    }
}

floatParameter Image2ImageCorrespondence::yImg1() const {
    return _yImg1;
}

void Image2ImageCorrespondence::setYImg1(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yImg1, 1e-4)) {
        _yImg1 = y;
        emit img1yCoordChanged(y);
        isChanged();
    }
}

void Image2ImageCorrespondence::setYImg1(float y) {

    if (!_yImg1.isApproximatlyEqual(y, 1e-4) or !_yImg1.isSet()) {
        _yImg1.setIsSet(y);
        emit img1yCoordChanged(_yImg1);
        isChanged();
    }
}

void Image2ImageCorrespondence::setImage1Coordinates(QPointF const& point) {
    setXImg1(point.x());
    setYImg1(point.y());
}

QPointF Image2ImageCorrespondence::image1Coordinates() const {
    return QPointF(_xImg1.value(), _yImg1.value());
}

floatParameter Image2ImageCorrespondence::xImg2() const {
    return _xImg2;
}

void Image2ImageCorrespondence::setXImg2(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xImg2, 1e-4)) {
        _xImg2 = x;
        emit img2xCoordChanged(x);
        isChanged();
    }
}

void Image2ImageCorrespondence::setXImg2(float x) {

    if (!_xImg2.isApproximatlyEqual(x, 1e-4) or !_xImg2.isSet()) {
        _xImg2.setIsSet(x);
        emit img2xCoordChanged(_xImg2);
        isChanged();
    }
}

floatParameter Image2ImageCorrespondence::yImg2() const {
    return _yImg2;
}

void Image2ImageCorrespondence::setYImg2(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yImg2, 1e-4)) {
        _yImg2 = y;
        emit img2yCoordChanged(y);
        isChanged();
    }
}

void Image2ImageCorrespondence::setYImg2(float y) {

    if (!_yImg2.isApproximatlyEqual(y, 1e-4) or !_yImg2.isSet()) {
        _yImg2.setIsSet(y);
        emit img2yCoordChanged(_yImg2);
        isChanged();
    }
}

void Image2ImageCorrespondence::setImage2Coordinates(QPointF const& point) {
    setXImg2(point.x());
    setYImg2(point.y());
}

QPointF Image2ImageCorrespondence::image2Coordinates() const {
    return QPointF(_xImg2.value(), _yImg2.value());
}

QJsonObject Image2ImageCorrespondence::encodeJson() const {
    QJsonObject obj;

    obj.insert("img1Id", attachedImg1Id());
    obj.insert("img2Id", attachedImg2Id());

    obj.insert("xImg1", floatParameter::toJson(xImg1()));
    obj.insert("yImg1", floatParameter::toJson(yImg1()));

    obj.insert("xImg2", floatParameter::toJson(xImg2()));
    obj.insert("yImg2", floatParameter::toJson(yImg2()));

    return obj;
}

void Image2ImageCorrespondence::configureFromJson(QJsonObject const& data) {

    if (data.contains("xImg1")) {
        _xImg1 = floatParameter::fromJson(data.value("xImg1").toObject());
    }
    if (data.contains("yImg1")) {
        _yImg1 = floatParameter::fromJson(data.value("yImg1").toObject());
    }

    if (data.contains("xImg2")) {
        _xImg2 = floatParameter::fromJson(data.value("xImg1").toObject());
    }
    if (data.contains("yImg2")) {
        _yImg2 = floatParameter::fromJson(data.value("yImg1").toObject());
    }

    if (data.contains("img1Id")) {
        _attachedImg1Id = data.value("img1Id").toInt(-1);
    }

    if (data.contains("img2Id")) {
        _attachedImg2Id = data.value("img2Id").toInt(-1);
    }

}

void Image2ImageCorrespondence::referedCleared(QVector<qint64> const& referedId) {

    DataBlock::referedCleared(referedId);

    if (referedId.front() == _attachedImg1Id) {
        _attachedImg1Id = -1;
        emit attachedImg1idChanged(-1);
    }

    if (referedId.front() == _attachedImg2Id) {
        _attachedImg2Id = -1;
        emit attachedImg2idChanged(-1);
    }

}

LocalCoordinates2LocalCoordinatesCorrespondence::LocalCoordinates2LocalCoordinatesCorrespondence(CorrespondencesSet* parent) :
    DataBlock(parent)
{
    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1xCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1yCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs1zCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);

    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2xCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2yCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2LocalCoordinatesCorrespondence::lcs2zCoordChanged,
            this, &LocalCoordinates2LocalCoordinatesCorrespondence::coordsChanged);
}

qint64 LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs1Id() const {
    return _attachedLcs1Id;
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setAttachedLcs1Id(qint64 id) {

    if (id < 0 and _attachedLcs1Id >= 0) {
        removeRefered({_attachedLcs1Id});
        _attachedLcs1Id = -1;
        emit attachedLcs1idChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedLcs1Id) {
        return;
    }

    if (_attachedLcs1Id >= 0) {
        removeRefered({_attachedLcs1Id});
    }
    _attachedLcs1Id = id;
    if (_attachedLcs1Id >= 0) {
        addRefered({_attachedLcs1Id});
    }
    emit attachedLcs1idChanged(_attachedLcs1Id);
    return;
}
QString LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs1Name() const {

    LocalCoordinateSystem* lcs1 = attachedLcs1();

    if (lcs1 != nullptr) {
        return lcs1->objectName();
    }

    return "";
}
LocalCoordinateSystem* LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs1() const {

    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(p->getById(attachedLcs1Id()));

    return lcs;
}

qint64 LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs2Id() const {
    return _attachedLcs2Id;
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setAttachedLcs2Id(qint64 id) {

    if (id < 0 and _attachedLcs2Id >= 0) {
        removeRefered({_attachedLcs2Id});
        _attachedLcs2Id = -1;
        emit attachedLcs1idChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedLcs2Id) {
        return;
    }

    if (_attachedLcs2Id >= 0) {
        removeRefered({_attachedLcs2Id});
    }
    _attachedLcs2Id = id;
    if (_attachedLcs2Id >= 0) {
        addRefered({_attachedLcs2Id});
    }
    emit attachedLcs1idChanged(_attachedLcs2Id);
    return;
}

QString LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs2Name() const {

    LocalCoordinateSystem* lcs2 = attachedLcs2();

    if (lcs2 != nullptr) {
        return lcs2->objectName();
    }

    return "";
}

LocalCoordinateSystem* LocalCoordinates2LocalCoordinatesCorrespondence::attachedLcs2() const {

    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(p->getById(attachedLcs2Id()));

    return lcs;
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::xLcs1() const {
    return _xLcs1;
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs1(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xLcs1, 1e-4)) {
        _xLcs1 = x;
        emit lcs1xCoordChanged(x);
        isChanged();
    }
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs1(float x) {

    if (!_xLcs1.isApproximatlyEqual(x, 1e-4) or !_xLcs1.isSet()) {
        _xLcs1.setIsSet(x);
        emit lcs1xCoordChanged(_xLcs1);
        isChanged();
    }
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::yLcs1() const {
    return _yLcs1;
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs1(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yLcs1, 1e-4)) {
        _yLcs1 = y;
        emit lcs1yCoordChanged(y);
        isChanged();
    }
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs1(float y) {

    if (!_yLcs1.isApproximatlyEqual(y, 1e-4) or !_yLcs1.isSet()) {
        _yLcs1.setIsSet(y);
        emit lcs1yCoordChanged(_yLcs1);
        isChanged();
    }
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::zLcs1() const {
    return _zLcs1;
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs1(const floatParameter &z) {

    if (!z.isApproximatlyEqual(_zLcs1, 1e-4)) {
        _zLcs1 = z;
        emit lcs1zCoordChanged(z);
        isChanged();
    }
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs1(float z) {

    if (!_zLcs1.isApproximatlyEqual(z, 1e-4) or !_zLcs1.isSet()) {
        _zLcs1.setIsSet(z);
        emit lcs1zCoordChanged(_zLcs1);
        isChanged();
    }
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setLcs1Coordinates(QVector3D const& point) {
    setXLcs1(point.x());
    setYLcs1(point.y());
    setZLcs1(point.z());
}

QVector3D LocalCoordinates2LocalCoordinatesCorrespondence::lcs1Coordinates() const {
    return QVector3D(_xLcs1.value(), _yLcs1.value(), _zLcs1.value());
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::xLcs2() const {
    return _xLcs2;
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs2(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xLcs2, 1e-4)) {
        _xLcs2 = x;
        emit lcs2xCoordChanged(x);
        isChanged();
    }
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setXLcs2(float x) {

    if (!_xLcs2.isApproximatlyEqual(x, 1e-4) or !_xLcs2.isSet()) {
        _xLcs2.setIsSet(x);
        emit lcs2xCoordChanged(_xLcs2);
        isChanged();
    }
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::yLcs2() const {
    return _yLcs2;
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs2(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yLcs2, 1e-4)) {
        _yLcs2 = y;
        emit lcs2yCoordChanged(y);
        isChanged();
    }
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setYLcs2(float y) {

    if (!_yLcs2.isApproximatlyEqual(y, 1e-4) or !_yLcs2.isSet()) {
        _yLcs2.setIsSet(y);
        emit lcs2yCoordChanged(_yLcs2);
        isChanged();
    }
}

floatParameter LocalCoordinates2LocalCoordinatesCorrespondence::zLcs2() const {
    return _zLcs2;
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs2(const floatParameter &z) {

    if (!z.isApproximatlyEqual(_zLcs2, 1e-4)) {
        _zLcs2 = z;
        emit lcs2zCoordChanged(z);
        isChanged();
    }
}
void LocalCoordinates2LocalCoordinatesCorrespondence::setZLcs2(float z) {

    if (!_zLcs2.isApproximatlyEqual(z, 1e-4) or !_zLcs2.isSet()) {
        _zLcs2.setIsSet(z);
        emit lcs2zCoordChanged(_zLcs2);
        isChanged();
    }
}

void LocalCoordinates2LocalCoordinatesCorrespondence::setLcs2Coordinates(QVector3D const& point) {
    setXLcs2(point.x());
    setYLcs2(point.y());
    setZLcs2(point.z());
}

QVector3D LocalCoordinates2LocalCoordinatesCorrespondence::lcs2Coordinates() const {
    return QVector3D(_xLcs2.value(), _yLcs2.value(), _zLcs2.value());
}


QJsonObject LocalCoordinates2LocalCoordinatesCorrespondence::encodeJson() const {

    QJsonObject obj;

    obj.insert("lcs1Id", attachedLcs1Id());
    obj.insert("lcs2Id", attachedLcs2Id());

    obj.insert("xLcs1", floatParameter::toJson(xLcs1()));
    obj.insert("yLcs1", floatParameter::toJson(yLcs1()));
    obj.insert("zLcs1", floatParameter::toJson(zLcs1()));

    obj.insert("xLcs2", floatParameter::toJson(xLcs2()));
    obj.insert("yLcs2", floatParameter::toJson(yLcs2()));
    obj.insert("zLcs2", floatParameter::toJson(zLcs2()));

    return obj;

}

void LocalCoordinates2LocalCoordinatesCorrespondence::configureFromJson(QJsonObject const& data) {

    if (data.contains("xLcs1")) {
        _xLcs1 = floatParameter::fromJson(data.value("xLcs1").toObject());
    }
    if (data.contains("yLcs1")) {
        _yLcs1 = floatParameter::fromJson(data.value("yLcs1").toObject());
    }
    if (data.contains("zLcs1")) {
        _zLcs1 = floatParameter::fromJson(data.value("zLcs1").toObject());
    }

    if (data.contains("xLcs2")) {
        _xLcs2 = floatParameter::fromJson(data.value("xLcs2").toObject());
    }
    if (data.contains("yLcs2")) {
        _yLcs2 = floatParameter::fromJson(data.value("yLcs2").toObject());
    }
    if (data.contains("zLcs2")) {
        _zLcs2 = floatParameter::fromJson(data.value("zLcs2").toObject());
    }

    if (data.contains("lcs1Id")) {
        _attachedLcs1Id = data.value("lcs1Id").toInt(-1);
    }

    if (data.contains("lcs2Id")) {
        _attachedLcs2Id = data.value("lcs2Id").toInt(-1);
    }

}

void LocalCoordinates2LocalCoordinatesCorrespondence::referedCleared(QVector<qint64> const& referedId) {

    DataBlock::referedCleared(referedId);

    if (referedId.front() == _attachedLcs1Id) {
        _attachedLcs1Id = -1;
        emit attachedLcs1idChanged(-1);
    }

    if (referedId.front() == _attachedLcs2Id) {
        _attachedLcs2Id = -1;
        emit attachedLcs2idChanged(-1);
    }
}

LocalCoordinates2ImageCorrespondence::LocalCoordinates2ImageCorrespondence(CorrespondencesSet* parent) :
    DataBlock(parent)
{
    connect(this, &LocalCoordinates2ImageCorrespondence::lcsxCoordChanged,
            this, &LocalCoordinates2ImageCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2ImageCorrespondence::lcsyCoordChanged,
            this, &LocalCoordinates2ImageCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2ImageCorrespondence::lcszCoordChanged,
            this, &LocalCoordinates2ImageCorrespondence::coordsChanged);

    connect(this, &LocalCoordinates2ImageCorrespondence::imgxCoordChanged,
            this, &LocalCoordinates2ImageCorrespondence::coordsChanged);
    connect(this, &LocalCoordinates2ImageCorrespondence::imgyCoordChanged,
            this, &LocalCoordinates2ImageCorrespondence::coordsChanged);
}

qint64 LocalCoordinates2ImageCorrespondence::attachedLcsId() const {
    return _attachedLcsId;
}

void LocalCoordinates2ImageCorrespondence::setAttachedLcsId(qint64 id) {

    if (id < 0 and _attachedLcsId >= 0) {
        removeRefered({_attachedLcsId});
        _attachedLcsId = -1;
        emit attachedLcsIdChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedLcsId) {
        return;
    }

    if (_attachedLcsId >= 0) {
        removeRefered({_attachedLcsId});
    }
    _attachedLcsId = id;
    if (_attachedLcsId >= 0) {
        addRefered({_attachedLcsId});
    }
    emit attachedLcsIdChanged(_attachedLcsId);
    return;
}

QString LocalCoordinates2ImageCorrespondence::attachedLcsName() const {

    LocalCoordinateSystem* lcs = attachedLcs();

    if (lcs != nullptr) {
        return lcs->objectName();
    }

    return "";
}

LocalCoordinateSystem* LocalCoordinates2ImageCorrespondence::attachedLcs() const {

    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(p->getById(attachedLcsId()));

    return lcs;
}

qint64 LocalCoordinates2ImageCorrespondence::attachedImgId() const {
    return _attachedImgId;
}

void LocalCoordinates2ImageCorrespondence::setAttachedImgId(qint64 id) {

    if (id < 0 and _attachedImgId >= 0) {
        removeRefered({_attachedImgId});
        _attachedImgId = -1;
        emit attachedImgIdChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedImgId) {
        return;
    }

    if (_attachedImgId >= 0) {
        removeRefered({_attachedImgId});
    }
    _attachedImgId = id;
    if (_attachedImgId >= 0) {
        addRefered({_attachedImgId});
    }
    emit attachedImgIdChanged(_attachedImgId);
    return;
}
QString LocalCoordinates2ImageCorrespondence::attachedImgName() const {

    Image* img = attachedImg();

    if (img != nullptr) {
        return img->objectName();
    }

    return "";
}
Image* LocalCoordinates2ImageCorrespondence::attachedImg() const {

    Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    Image* img = qobject_cast<Image*>(p->getById(attachedImgId()));

    return img;
}

floatParameter LocalCoordinates2ImageCorrespondence::xLcs() const {
    return _xLcs;
}

void LocalCoordinates2ImageCorrespondence::setXLcs(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xLcs, 1e-4)) {
        _xLcs = x;
        emit lcsxCoordChanged(x);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setXLcs(float x) {

    if (!_xLcs.isApproximatlyEqual(x, 1e-4) or !_xLcs.isSet()) {
        _xLcs.setIsSet(x);
        emit lcsxCoordChanged(_xLcs);
        isChanged();
    }
}

floatParameter LocalCoordinates2ImageCorrespondence::yLcs() const {
    return _yLcs;
}

void LocalCoordinates2ImageCorrespondence::setYLcs(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yLcs, 1e-4)) {
        _yLcs = y;
        emit lcsyCoordChanged(y);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setYLcs(float y) {

    if (!_yLcs.isApproximatlyEqual(y, 1e-4) or !_yLcs.isSet()) {
        _yLcs.setIsSet(y);
        emit lcsyCoordChanged(_yLcs);
        isChanged();
    }
}

floatParameter LocalCoordinates2ImageCorrespondence::zLcs() const {
    return _zLcs;
}

void LocalCoordinates2ImageCorrespondence::setZLcs(const floatParameter &z) {

    if (!z.isApproximatlyEqual(_zLcs, 1e-4)) {
        _zLcs = z;
        emit lcszCoordChanged(z);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setZLcs(float z) {

    if (!_zLcs.isApproximatlyEqual(z, 1e-4) or !_zLcs.isSet()) {
        _zLcs.setIsSet(z);
        emit lcszCoordChanged(_zLcs);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setLcsCoordinates(QVector3D const& point) {
    setXLcs(point.x());
    setYLcs(point.y());
    setZLcs(point.z());
}

QVector3D LocalCoordinates2ImageCorrespondence::lcsCoordinates() const {
    return QVector3D(_xLcs.value(), _yLcs.value(), _zLcs.value());
}

floatParameter LocalCoordinates2ImageCorrespondence::xImg() const {
    return _xImg;
}

void LocalCoordinates2ImageCorrespondence::setXImg(const floatParameter &x) {

    if (!x.isApproximatlyEqual(_xImg, 1e-4)) {
        _xImg = x;
        emit imgxCoordChanged(x);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setXImg(float x) {

    if (!_xImg.isApproximatlyEqual(x, 1e-4) or !_xImg.isSet()) {
        _xImg.setIsSet(x);
        emit imgxCoordChanged(_xImg);
        isChanged();
    }
}

floatParameter LocalCoordinates2ImageCorrespondence::yImg() const {
    return _yImg;
}
void LocalCoordinates2ImageCorrespondence::setYImg(const floatParameter &y) {

    if (!y.isApproximatlyEqual(_yImg, 1e-4)) {
        _yImg = y;
        emit imgyCoordChanged(y);
        isChanged();
    }
}
void LocalCoordinates2ImageCorrespondence::setYImg(float y) {

    if (!_yImg.isApproximatlyEqual(y, 1e-4) or !_yImg.isSet()) {
        _yImg.setIsSet(y);
        emit imgyCoordChanged(_yImg);
        isChanged();
    }
}

void LocalCoordinates2ImageCorrespondence::setImageCoordinates(QPointF const& point) {
    setXImg(point.x());
    setYImg(point.y());
}
QPointF LocalCoordinates2ImageCorrespondence::imageCoordinates() const {
    return QPointF(_xImg.value(), _yImg.value());
}

QJsonObject LocalCoordinates2ImageCorrespondence::encodeJson() const {

    QJsonObject obj;

    obj.insert("lcsId", attachedLcsId());
    obj.insert("imgId", attachedImgId());

    obj.insert("xLcs", floatParameter::toJson(xLcs()));
    obj.insert("yLcs", floatParameter::toJson(yLcs()));
    obj.insert("zLcs", floatParameter::toJson(zLcs()));

    obj.insert("xImg", floatParameter::toJson(xImg()));
    obj.insert("yImg", floatParameter::toJson(yImg()));

    return obj;

}

void LocalCoordinates2ImageCorrespondence::configureFromJson(QJsonObject const& data) {

    if (data.contains("xImg")) {
        _xImg = floatParameter::fromJson(data.value("xImg").toObject());
    }
    if (data.contains("yImg")) {
        _yImg = floatParameter::fromJson(data.value("yImg").toObject());
    }

    if (data.contains("xLcs")) {
        _xLcs = floatParameter::fromJson(data.value("xLcs").toObject());
    }
    if (data.contains("yLcs")) {
        _yLcs = floatParameter::fromJson(data.value("yLcs").toObject());
    }
    if (data.contains("zLcs")) {
        _zLcs = floatParameter::fromJson(data.value("zLcs").toObject());
    }

    if (data.contains("lcsId")) {
        _attachedLcsId = data.value("lcsId").toInt(-1);
    }

    if (data.contains("imgId")) {
        _attachedImgId = data.value("imgId").toInt(-1);
    }
}

void LocalCoordinates2ImageCorrespondence::referedCleared(QVector<qint64> const& referedId) {


    DataBlock::referedCleared(referedId);

    if (referedId.front() == _attachedLcsId) {
        _attachedLcsId = -1;
        emit attachedLcsIdChanged(-1);
    }

    if (referedId.front() == _attachedImgId) {
        _attachedImgId = -1;
        emit attachedImgIdChanged(-1);
    }
}

CorrespondencesSetFactory::CorrespondencesSetFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString CorrespondencesSetFactory::TypeDescrName() const {
    return tr("Correspondances set");
}
DataBlockFactory::FactorizableFlags CorrespondencesSetFactory::factorizable() const {
    return DataBlockFactory::RootDataBlock;
}
DataBlock* CorrespondencesSetFactory::factorizeDataBlock(Project *parent) const {
    return new CorrespondencesSet(parent);
}

QString CorrespondencesSetFactory::itemClassName() const {
    return CorrespondencesSet::staticMetaObject.className();
}


} // namespace StereoVisionApp
