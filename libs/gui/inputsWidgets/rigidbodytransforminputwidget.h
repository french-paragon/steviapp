#ifndef STEREOVISIONAPP_RIGIDBODYTRANSFORMINPUTWIDGET_H
#define STEREOVISIONAPP_RIGIDBODYTRANSFORMINPUTWIDGET_H

#include <optional>

#include <QWidget>
#include <QDialog>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

class QGridLayout;
class QDoubleSpinBox;
class QLineEdit;

namespace StereoVisionApp {

class RigidBodyTransformInputWidget : public QWidget
{
    Q_OBJECT
public:

    RigidBodyTransformInputWidget(QWidget* parent = nullptr);

    StereoVision::Geometry::AffineTransform<double> getAffineTransform() const;
    void setAffineTransform(StereoVision::Geometry::AffineTransform<double> const& transform);

    StereoVision::Geometry::RigidBodyTransform<double> getRigidBodyTransform() const;
    void setRigidBodyTransform(StereoVision::Geometry::RigidBodyTransform<double> const& transform);

Q_SIGNALS:

    void transformChanged();

protected:

    void setupRotationPart();

    QGridLayout* _mainLayout;
    QGridLayout* _rLayout;
    QGridLayout* _tLayout;

    QDoubleSpinBox* _r11;
    QDoubleSpinBox* _r12;
    QDoubleSpinBox* _r13;
    QDoubleSpinBox* _r21;
    QDoubleSpinBox* _r22;
    QDoubleSpinBox* _r23;
    QDoubleSpinBox* _r31;
    QDoubleSpinBox* _r32;
    QDoubleSpinBox* _r33;

    QDoubleSpinBox* _t1;
    QDoubleSpinBox* _t2;
    QDoubleSpinBox* _t3;
};


class RigidBodyTransformInputDialog : public QDialog
{
    Q_OBJECT
public:
    RigidBodyTransformInputDialog(QWidget* parent = nullptr);

    static std::optional<StereoVision::Geometry::AffineTransform<double>> inputAffineTransform(QString title, QWidget* parent = nullptr);
    static std::optional<StereoVision::Geometry::RigidBodyTransform<double>> inputRigidTransform(QString title, QWidget* parent = nullptr);

    inline StereoVision::Geometry::AffineTransform<double> getAffineTransform() const {
        return _widget->getAffineTransform();
    }
    inline void setAffineTransform(StereoVision::Geometry::AffineTransform<double> const& transform) {
        _widget->setAffineTransform(transform);
    }

    inline StereoVision::Geometry::RigidBodyTransform<double> getRigidBodyTransform() const {
        return _widget->getRigidBodyTransform();
    }
    inline void setRigidBodyTransform(StereoVision::Geometry::RigidBodyTransform<double> const& transform) {
        _widget->setRigidBodyTransform(transform);
    }

Q_SIGNALS:

    void transformChanged();

protected:

    RigidBodyTransformInputWidget* _widget;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_RIGIDBODYTRANSFORMINPUTWIDGET_H
