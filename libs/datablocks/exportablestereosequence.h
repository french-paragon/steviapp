#ifndef STEREOVISIONAPP_EXPORTABLESTEREOSEQUENCE_H
#define STEREOVISIONAPP_EXPORTABLESTEREOSEQUENCE_H

#include "./project.h"

namespace StereoVisionApp {

class ExportableStereoSequence : public DataBlock
{
	Q_OBJECT
public:
	ExportableStereoSequence(Project* parent);

	QString exportFolder() const;
	void setExportFolder(QString const& folder);

	int erosionAmount() const;
	void setErosionAmount(int erosion);

	int openingAmount() const;
	void setOpeningAmount(int opening);

Q_SIGNALS:

	void exportFolderChanged();

	void erosionAmountChanged(qint64 erosion);
	void openingAmountChanged(qint64 opening);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	QString _img_out_folder;

	int _erosion_amount;
	int _opening_amount;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EXPORTABLESTEREOSEQUENCE_H
