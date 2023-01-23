#include "exportablestereosequence.h"

#include <QJsonObject>

namespace StereoVisionApp {

ExportableStereoSequence::ExportableStereoSequence(Project* parent) :
	DataBlock(parent),
	_img_out_folder(""),
	_erosion_amount(0),
	_opening_amount(0)
{

}

QString ExportableStereoSequence::exportFolder() const {
	return _img_out_folder;
}
void ExportableStereoSequence::setExportFolder(QString const& folder) {

	if (_img_out_folder != folder) {
		_img_out_folder = folder;
		Q_EMIT exportFolderChanged();
	}

}

int ExportableStereoSequence::erosionAmount() const {
	return _erosion_amount;
}
void ExportableStereoSequence::setErosionAmount(int erosion) {
	if (_erosion_amount != erosion) {
		_erosion_amount = erosion;
		Q_EMIT erosionAmountChanged(erosion);
	}
}

int ExportableStereoSequence::openingAmount() const {
	return _opening_amount;
}
void ExportableStereoSequence::setOpeningAmount(int opening) {
	if (_opening_amount != opening) {
		_opening_amount = opening;
		Q_EMIT openingAmountChanged(opening);
	}
}



QJsonObject ExportableStereoSequence::encodeJson() const {

	QJsonObject obj;

	obj.insert("exportFolder", _img_out_folder);
	obj.insert("erosionAmount", _erosion_amount);
	obj.insert("openingAmount", _opening_amount);

	return obj;
}
void ExportableStereoSequence::configureFromJson(QJsonObject const& data) {

	setExportFolder(data.value("exportFolder").toString(""));
	setErosionAmount(data.value("erosionAmount").toInt(0));
	setOpeningAmount(data.value("openingAmount").toInt(0));

}

} // namespace StereoVisionApp
