#include "mounting.h"

#include "itemdatamodel.h"

namespace StereoVisionApp {

Mounting::Mounting(Project *parent) :
    RigidBody(parent)
{
    extendDataModel();
}

void Mounting::extendDataModel() {

    ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

    //Position
    auto* xPosProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
                                                                                                                 &RigidBody::xCoord,
                                                                                                                 &RigidBody::setXCoord,
                                                                                                                 &RigidBody::xCoordChanged);

    xPosProp->setNumericPrecision(4);

    auto* yPosProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
                                                                                                                 &RigidBody::yCoord,
                                                                                                                 &RigidBody::setYCoord,
                                                                                                                 &RigidBody::yCoordChanged);

    yPosProp->setNumericPrecision(4);

    auto* zPosProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
                                                                                                                 &RigidBody::zCoord,
                                                                                                                 &RigidBody::setZCoord,
                                                                                                                 &RigidBody::zCoordChanged);

    zPosProp->setNumericPrecision(4);

    //Rotation
    auto* xRaxisProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X Raxis"),
                                                                                                                 &RigidBody::xRot,
                                                                                                                 &RigidBody::setXRot,
                                                                                                                 &RigidBody::xRotChanged);
    xRaxisProp->setNumericPrecision(6);

    auto* yRaxisProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y Raxis"),
                                                                                                                 &RigidBody::yRot,
                                                                                                                 &RigidBody::setYRot,
                                                                                                                 &RigidBody::yRotChanged);
    yRaxisProp->setNumericPrecision(6);

    auto* zRaxisProp = g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z Raxis"),
                                                                                                                 &RigidBody::zRot,
                                                                                                                 &RigidBody::setZRot,
                                                                                                                 &RigidBody::zRotChanged);
    zRaxisProp->setNumericPrecision(6);



    ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

    optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Fixed"),
                                                                                                          &DataBlock::isFixed,
                                                                                                          &DataBlock::setFixed,
                                                                                                          &DataBlock::isFixedChanged);

    ItemDataModel::Category* og = _dataModel->addCategory(tr("Optimized geometry"));

    //Position
    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
                                                                                                  &RigidBody::optXCoord,
                                                                                                  &RigidBody::setOptXCoord,
                                                                                                  &RigidBody::optPosChanged);

    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
                                                                                                  &RigidBody::optYCoord,
                                                                                                  &RigidBody::setOptYCoord,
                                                                                                  &RigidBody::optPosChanged);

    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
                                                                                                  &RigidBody::optZCoord,
                                                                                                  &RigidBody::setOptZCoord,
                                                                                                  &RigidBody::optPosChanged);

    //Rotation
    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X Raxis"),
                                                                                                  &RigidBody::optXRot,
                                                                                                  &RigidBody::setOptXRot,
                                                                                                  &RigidBody::optRotChanged);

    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y Raxis"),
                                                                                                  &RigidBody::optYRot,
                                                                                                  &RigidBody::setOptYRot,
                                                                                                  &RigidBody::optRotChanged);

    og->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z Raxis"),
                                                                                                  &RigidBody::optZRot,
                                                                                                  &RigidBody::setOptZRot,
                                                                                                  &RigidBody::optRotChanged);
}

MountingFactory::MountingFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString MountingFactory::TypeDescrName() const {
    return tr("Mounting");
}
DataBlockFactory::FactorizableFlags MountingFactory::factorizable() const {
    return DataBlockFactory::RootDataBlock;
}
DataBlock* MountingFactory::factorizeDataBlock(Project *parent) const {
    return new Mounting(parent);
}

QString MountingFactory::itemClassName() const {
    return Mounting::staticMetaObject.className();
}

} // namespace StereoVisionApp
