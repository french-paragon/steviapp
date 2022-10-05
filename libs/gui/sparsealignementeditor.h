#ifndef SPARSEALIGNEMENTEDITOR_H
#define SPARSEALIGNEMENTEDITOR_H

#include <QWidget>

#include "./editor.h"

namespace StereoVisionApp {

class ProjectSparseAlignementDataInterface;

namespace Ui {
class SparseAlignementEditor;
}

class SparseAlignementEditor : public Editor
{
	Q_OBJECT

public:

	static const QString SparseAlignementEditorClassName;

	explicit SparseAlignementEditor(QWidget *parent = nullptr);
	~SparseAlignementEditor();

	void reloadLandmarks();
	void clearLandmarks();

protected:

	void beforeProjectChange(Project* np) override;

private:
	Ui::SparseAlignementEditor *ui;

	ProjectSparseAlignementDataInterface* _viewerInterface;
};

class SparseAlignementEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit SparseAlignementEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;


};

}//namespace StereoVisionApp

#endif // SPARSEALIGNEMENTEDITOR_H
