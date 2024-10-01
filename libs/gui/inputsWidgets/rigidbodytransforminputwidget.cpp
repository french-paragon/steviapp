#include "rigidbodytransforminputwidget.h"

#include <QVBoxLayout>
#include <QGridLayout>

#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <QAbstractButton>

namespace StereoVisionApp {

RigidBodyTransformInputWidget::RigidBodyTransformInputWidget(QWidget *parent) :
    QWidget(parent)
{
    _mainLayout = new QGridLayout(this);
    _mainLayout->setSpacing(5);

    QLabel* rotationPartLabel = new QLabel(tr("Rotation"), this);
    _mainLayout->addWidget(rotationPartLabel, 0, 0, Qt::AlignLeft|Qt::AlignVCenter);

    QLabel* translationPartLabel = new QLabel(tr("Translation"), this);
    _mainLayout->addWidget(translationPartLabel, 0, 1, Qt::AlignLeft|Qt::AlignVCenter);

    _rLayout = new QGridLayout(this);

    _r11 = new QDoubleSpinBox(this);
    _r12 = new QDoubleSpinBox(this);
    _r13 = new QDoubleSpinBox(this);

    _r21 = new QDoubleSpinBox(this);
    _r22 = new QDoubleSpinBox(this);
    _r23 = new QDoubleSpinBox(this);

    _r31 = new QDoubleSpinBox(this);
    _r32 = new QDoubleSpinBox(this);
    _r33 = new QDoubleSpinBox(this);

    _r11->setMinimum(-1);
    _r11->setMaximum(1);
    _r11->setValue(1);

    _r12->setMinimum(-1);
    _r12->setMaximum(1);
    _r12->setValue(0);

    _r13->setMinimum(-1);
    _r13->setMaximum(1);
    _r13->setValue(0);

    _r21->setMinimum(-1);
    _r21->setMaximum(1);
    _r21->setValue(0);

    _r22->setMinimum(-1);
    _r22->setMaximum(1);
    _r22->setValue(1);

    _r23->setMinimum(-1);
    _r23->setMaximum(1);
    _r23->setValue(0);

    _r31->setMinimum(-1);
    _r31->setMaximum(1);
    _r31->setValue(0);

    _r32->setMinimum(-1);
    _r32->setMaximum(1);
    _r32->setValue(0);

    _r33->setMinimum(-1);
    _r33->setMaximum(1);
    _r33->setValue(1);

    connect(_r11, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r12, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r13, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r21, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r22, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r23, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r31, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r32, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_r33, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);

    setupRotationPart();

    _tLayout = new QGridLayout(this);

    _t1 = new QDoubleSpinBox(this);
    _t2 = new QDoubleSpinBox(this);
    _t3 = new QDoubleSpinBox(this);

    _t1->setMinimum(-99999999999);
    _t1->setMaximum(99999999999);
    _t1->setValue(0);

    _t2->setMinimum(-99999999999);
    _t2->setMaximum(99999999999);
    _t2->setValue(0);

    _t3->setMinimum(-99999999999);
    _t3->setMaximum(99999999999);
    _t3->setValue(0);

    _tLayout->addWidget(_t1, 0, 0);
    _tLayout->addWidget(_t2, 0, 1);
    _tLayout->addWidget(_t3, 0, 2);

    _mainLayout->addLayout(_rLayout, 1, 0, Qt::AlignHCenter|Qt::AlignVCenter);
    _mainLayout->addLayout(_tLayout, 1, 1, Qt::AlignHCenter|Qt::AlignVCenter);

    connect(_t1, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_t2, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);
    connect(_t3, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RigidBodyTransformInputWidget::transformChanged);

}

StereoVision::Geometry::AffineTransform<double> RigidBodyTransformInputWidget::getAffineTransform() const {

    StereoVision::Geometry::AffineTransform<double> ret;

    ret.R(0,0) = _r11->value();
    ret.R(0,1) = _r12->value();
    ret.R(0,2) = _r13->value();

    ret.R(1,0) = _r21->value();
    ret.R(1,1) = _r22->value();
    ret.R(1,2) = _r23->value();

    ret.R(2,0) = _r31->value();
    ret.R(2,1) = _r32->value();
    ret.R(2,2) = _r33->value();

    ret.t[0] = _t1->value();
    ret.t[1] = _t2->value();
    ret.t[2] = _t3->value();

    return ret;

}
void RigidBodyTransformInputWidget::setAffineTransform(StereoVision::Geometry::AffineTransform<double> const& transform) {

    blockSignals(true);

    _r11->setValue(transform.R(0,0));
    _r12->setValue(transform.R(0,1));
    _r13->setValue(transform.R(0,2));

    _r21->setValue(transform.R(1,0));
    _r22->setValue(transform.R(1,1));
    _r23->setValue(transform.R(1,2));

    _r31->setValue(transform.R(2,0));
    _r32->setValue(transform.R(2,1));
    _r33->setValue(transform.R(2,2));

    _t1->setValue(transform.t[0]);
    _t1->setValue(transform.t[1]);
    _t1->setValue(transform.t[2]);

    blockSignals(false);

    Q_EMIT transformChanged();

}

StereoVision::Geometry::RigidBodyTransform<double> RigidBodyTransformInputWidget::getRigidBodyTransform() const {
    return StereoVision::Geometry::RigidBodyTransform<double>(getAffineTransform());
}
void RigidBodyTransformInputWidget::setRigidBodyTransform(StereoVision::Geometry::RigidBodyTransform<double> const& transform) {
    setAffineTransform(transform.toAffineTransform());
}

void RigidBodyTransformInputWidget::setupRotationPart() {

    _rLayout->addWidget(_r11, 0, 0, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r12, 0, 1, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r13, 0, 2, Qt::AlignHCenter|Qt::AlignVCenter);

    _rLayout->addWidget(_r21, 1, 0, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r22, 1, 1, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r23, 1, 2, Qt::AlignHCenter|Qt::AlignVCenter);

    _rLayout->addWidget(_r31, 2, 0, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r32, 2, 1, Qt::AlignHCenter|Qt::AlignVCenter);
    _rLayout->addWidget(_r33, 2, 2, Qt::AlignHCenter|Qt::AlignVCenter);

}

RigidBodyTransformInputDialog::RigidBodyTransformInputDialog(QWidget* parent) :
    QDialog(parent)
{

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    _widget = new RigidBodyTransformInputWidget(this);

    mainLayout->addWidget(_widget);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);

    buttonBox->addButton(tr("Ok"), QDialogButtonBox::ApplyRole);
    buttonBox->addButton(tr("Cancel"), QDialogButtonBox::RejectRole);

    mainLayout->addWidget(buttonBox);

    setResult(QDialogButtonBox::RejectRole); //default result is reject

    connect(buttonBox, &QDialogButtonBox::clicked, this, [this, buttonBox] (QAbstractButton *button) {
        QDialogButtonBox::ButtonRole role = buttonBox->buttonRole(button);
        done(role);
    });

}

std::optional<StereoVision::Geometry::AffineTransform<double>> RigidBodyTransformInputDialog::inputAffineTransform(QString title, QWidget* parent) {

    RigidBodyTransformInputDialog dialog(parent);
    dialog.setWindowTitle(title);
    dialog.setModal(true);

    dialog.exec();

    int result = dialog.result();

    if (result != QDialogButtonBox::ApplyRole) {
        return std::nullopt;
    }

    return dialog.getAffineTransform();

}

std::optional<StereoVision::Geometry::RigidBodyTransform<double>> RigidBodyTransformInputDialog::inputRigidTransform(QString title, QWidget* parent) {

    RigidBodyTransformInputDialog dialog(parent);
    dialog.setWindowTitle(title);
    dialog.setModal(true);

    dialog.exec();

    int result = dialog.result();

    if (result != QDialogButtonBox::ApplyRole) {
        return std::nullopt;
    }

    return dialog.getRigidBodyTransform();

}

} // namespace StereoVisionApp
