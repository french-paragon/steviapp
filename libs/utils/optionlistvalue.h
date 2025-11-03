#ifndef OPTIONLISTVALUE_H
#define OPTIONLISTVALUE_H

#include <QString>
#include <QStringList>
#include <QMetaType>

namespace StereoVisionApp {

struct OptionListValue
{
    QString value;
    QStringList possibleValues;
private:
    static const int registrationCode;
};

} // namespace StereoVisionApp

Q_DECLARE_METATYPE(StereoVisionApp::OptionListValue)

#endif // OPTIONLISTVALUE_H
