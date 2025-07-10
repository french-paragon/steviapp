#ifndef FLOATINGPOINTBLOCKEDITSPINBOX_H
#define FLOATINGPOINTBLOCKEDITSPINBOX_H

#include <QDoubleSpinBox>
#include <QItemEditorCreatorBase>

#include "../../utils/editablefloatingpointblock.h"

namespace StereoVisionApp {

class FloatingPointBlockEditSpinBox : public QDoubleSpinBox
{
    Q_OBJECT
public:

    Q_PROPERTY(EditableFloatingPointBlock floatingPointBlock READ floatingPointBlock WRITE setEditableFloatingPointBlock NOTIFY floatingPointBlockChanged FINAL)

    explicit FloatingPointBlockEditSpinBox(QWidget *parent = nullptr);

    EditableFloatingPointBlock floatingPointBlock() const;
    void setEditableFloatingPointBlock(EditableFloatingPointBlock const& block);

Q_SIGNALS:
    void floatingPointBlockChanged();
};

class FloatingPointBlockEditorCreator: public QItemEditorCreatorBase {
public:
    virtual QWidget * createWidget(QWidget *parent) const override;
    virtual QByteArray valuePropertyName() const override;
};

} // namespace StereoVisionApp

#endif // FLOATINGPOINTBLOCKEDITSPINBOX_H
