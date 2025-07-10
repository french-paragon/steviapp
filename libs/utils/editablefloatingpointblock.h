#ifndef EDITABLEFLOATINGPOINTBLOCK_H
#define EDITABLEFLOATINGPOINTBLOCK_H

#include <QString>
#include <QMetaType>

namespace StereoVisionApp {

/*!
 * \brief The EditableFloatingPointBlock class contain an editable value along with more data to format the editor
 */
struct EditableFloatingPointBlock
{
    double value;
    double min;
    double max;
    int precision;
    QString suffix;
private:
    static const int registrationCode;
};

} // namespace StereoVisionApp

Q_DECLARE_METATYPE(StereoVisionApp::EditableFloatingPointBlock)

#endif // EDITABLEFLOATINGPOINTBLOCK_H
