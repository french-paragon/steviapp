#include "autoformatstyleditemdelegate.h"

#include "utils/editablefloatingpointblock.h"

#include "gui/inputsWidgets/floatingpointblockeditspinbox.h"

#include "utils/optionlistvalue.h"

#include <QComboBox>

namespace StereoVisionApp {

AutoFormatStyledItemDelegate::AutoFormatStyledItemDelegate(QObject *parent)
    : QStyledItemDelegate{parent}
{}

QWidget* AutoFormatStyledItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
    QVariant data = index.data(Qt::EditRole);

    if (data.userType() == qMetaTypeId<EditableFloatingPointBlock>()) {
        FloatingPointBlockEditSpinBox* editor = new FloatingPointBlockEditSpinBox(parent);
        return editor;
    } else if (data.userType() == qMetaTypeId<OptionListValue>()) {
        QComboBox* editor = new QComboBox(parent);
        editor->addItems(qvariant_cast<OptionListValue>(data).possibleValues);
        return editor;
    }

    return QStyledItemDelegate::createEditor(parent, option, index);
}

void AutoFormatStyledItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
    QVariant data = index.data(Qt::EditRole);

    if (data.userType() == qMetaTypeId<EditableFloatingPointBlock>()) {
        FloatingPointBlockEditSpinBox* floating_point_block_editor = qobject_cast<FloatingPointBlockEditSpinBox*>(editor);
        floating_point_block_editor->setEditableFloatingPointBlock(qvariant_cast<EditableFloatingPointBlock>(data));
        return;
    } else if (data.userType() == qMetaTypeId<OptionListValue>()) {
        OptionListValue val = qvariant_cast<OptionListValue>(data);
        QComboBox* box = qobject_cast<QComboBox*>(editor);
        box->setCurrentText(val.value);
        return;
    }

    QStyledItemDelegate::setEditorData(editor, index);

}

void AutoFormatStyledItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {

    FloatingPointBlockEditSpinBox* floating_point_block_editor = qobject_cast<FloatingPointBlockEditSpinBox*>(editor);
    QComboBox* combobox = qobject_cast<QComboBox*>(editor);

    if (floating_point_block_editor != nullptr) {
        model->setData(index, QVariant::fromValue(floating_point_block_editor->floatingPointBlock()));
        return;
    } else if (combobox != nullptr) {
        QVariant currentVal = combobox->currentData();

        if (!currentVal.isNull()) {
             model->setData(index, currentVal);
        } else {
             model->setData(index, combobox->currentText());
        }
    }

    QStyledItemDelegate::setModelData(editor, model, index);
}

} // namespace StereoVisionApp
