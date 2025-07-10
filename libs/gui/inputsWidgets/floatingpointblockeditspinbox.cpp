#include "floatingpointblockeditspinbox.h"

namespace StereoVisionApp {

FloatingPointBlockEditSpinBox::FloatingPointBlockEditSpinBox(QWidget *parent)
    : QDoubleSpinBox{parent}
{
    connect(this, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &FloatingPointBlockEditSpinBox::floatingPointBlockChanged);
}

EditableFloatingPointBlock FloatingPointBlockEditSpinBox::floatingPointBlock() const {
    return EditableFloatingPointBlock{value(), minimum(), maximum(), decimals(), suffix()};
}
void FloatingPointBlockEditSpinBox::setEditableFloatingPointBlock(EditableFloatingPointBlock const& block) {
    setDecimals(block.precision);
    setSuffix(block.suffix);
    setMinimum(block.min);
    setMaximum(block.max);
    setValue(block.value);
}

QWidget * FloatingPointBlockEditorCreator::createWidget(QWidget *parent) const {
    return new FloatingPointBlockEditSpinBox(parent);
}
QByteArray FloatingPointBlockEditorCreator::valuePropertyName() const {
    return "floatingPointBlock";
}

} // namespace StereoVisionApp
