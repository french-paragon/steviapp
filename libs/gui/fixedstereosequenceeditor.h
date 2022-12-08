#ifndef FIXEDSTEREOSEQUENCEEDITOR_H
#define FIXEDSTEREOSEQUENCEEDITOR_H

#include "./editor.h"


namespace StereoVisionApp {

class DataBlock;
class FixedColorStereoSequence;
class FixedStereoPlusColorSequence;

namespace Ui {
class FixedStereoSequenceEditor;
}

class FixedStereoSequenceEditor : public Editor
{
	Q_OBJECT

public:
	explicit FixedStereoSequenceEditor(QWidget *parent = nullptr);
	~FixedStereoSequenceEditor();

	void setSequence(DataBlock* sequence);

Q_SIGNALS:

	void rgbImagesExportTriggered(FixedColorStereoSequence* sequence, QVector<int> rows);
	void imagesWithRGBExportTriggered(FixedStereoPlusColorSequence* sequence, QVector<int> rows);

private:

	void refreshDisplay();
	void loadImages(QString directory);

	void leftViewSelectionChanged(int row);
	void rgbViewSelectionChanged(int row);
	void rightViewSelectionChanged(int row);

	void treeViewContextMenuRequested(const QPoint &pos);

	Ui::FixedStereoSequenceEditor *ui;

	DataBlock* _sequence;
};

class FixedStereoSequenceEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit FixedStereoSequenceEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;


};

}

#endif // FIXEDSTEREOSEQUENCEEDITOR_H
