#ifndef SPARSEALIGNEMENTEDITOR_H
#define SPARSEALIGNEMENTEDITOR_H

#include <QWidget>
#include <QString>
#include <QMap>

#include "./editor.h"

namespace StereoVisionApp {

class OpenGlDrawable;
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

    /*!
     * \brief addDrawable add a drawable to the editor
     * \param name the name for the editor
     * \param drawable the editor to add
     * \return true if the editor could be added, false otherwise
     *
     * The drawable will fail to add if it is nullptr, or if an editor with the same name exist already
     *
     * The editor takes ownership of the drawable. If the addition fail, the drawable is scheduled for deletion.
     */
    bool addDrawable(QString const& name, OpenGlDrawable* drawable);

    /*!
     * \brief getDrawable get a drawable from a name
     * \param name the name of the drawable
     * \return the drawable, or nullptr if it does not exist
     */
    OpenGlDrawable* getDrawable(QString const& name);

    /*!
     * \brief removeDrawable remove the drawable from the editor
     * \param name the name of the drawable to remove
     *
     * The associated drawable will be deleted
     */
    void removeDrawable(QString const& name);

protected:

	void beforeProjectChange(Project* np) override;

private:
	Ui::SparseAlignementEditor *ui;

	ProjectSparseAlignementDataInterface* _viewerInterface;
    QMap<QString, OpenGlDrawable*> _additionalDrawables;
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
