#ifndef STEREOVISIONAPP_MOUNTING_H
#define STEREOVISIONAPP_MOUNTING_H

#include "./project.h"
#include "./rigidbody.h"

namespace StereoVisionApp {

class Mounting : public RigidBody
{
    Q_OBJECT
public:
    Mounting(Project *parent = nullptr);

protected:

    void extendDataModel();
};

class MountingFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit MountingFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MOUNTING_H
