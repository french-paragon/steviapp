#ifndef FIXEDSTEREOSEQUENCEEDITOR_H
#define FIXEDSTEREOSEQUENCEEDITOR_H

#include "./editor.h"


namespace StereoVisionApp {

class DataBlock;

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

private:

	void refreshDisplay();
	void loadImages(QString directory);

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
