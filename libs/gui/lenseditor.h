#ifndef LENSEEDITOR_H
#define LENSEEDITOR_H

#include "./editor.h"

class QCustomPlot;

namespace StereoVisionApp {

class Camera;
class PushBroomPinholeCamera;

namespace Ui {
class LenseEditor;
}

class LensEditor : public Editor
{
	Q_OBJECT

public:

	static const QString LensEditorClassName;

	explicit LensEditor(QWidget *parent = nullptr);
	~LensEditor();

	void setCamera(Camera* cam);

private:

	void activeCameraDestroyed();

	void activeCameraFlenChanged();
	void activeCameraPixRatioChanged();

	void activeCameraOpticalCenterXChanged();
	void activeCameraOpticalCenterYChanged();

	void activeCameraK1Changed();
	void activeCameraK2Changed();
	void activeCameraK3Changed();

	void activeCameraP1Changed();
	void activeCameraP2Changed();

	void activeCameraB1Changed();
	void activeCameraB2Changed();

	void activeCameraOptimizedFlenChanged();
	void activeCameraOptimizedPixRatioChanged();

	void activeCameraOptimizedOpticalCenterXChanged();
	void activeCameraOptimizedOpticalCenterYChanged();

	void activeCameraOptimizedK1Changed();
	void activeCameraOptimizedK2Changed();
	void activeCameraOptimizedK3Changed();

	void activeCameraOptimizedP1Changed();
	void activeCameraOptimizedP2Changed();

	void activeCameraOptimizedB1Changed();
	void activeCameraOptimizedB2Changed();

	void onParameterFLenChanged();
	void onParameterPixSizeChanged();

	void OnParameterOpticalCenterXChanged();
	void OnParameterOpticalCenterYChanged();

	void onParameterRadialK1Changed();
	void onParameterRadialK2Changed();
	void onParameterRadialK3Changed();

	void onParameterTangentialP1Changed();
	void onParameterTangentialP2Changed();

	void onParameterSkewB1Changed();
	void onParameterSkewB2Changed();

	void onTabSwitch();

	Ui::LenseEditor *ui;

	Camera* _cam;
};


class LensEditorFactory : public EditorFactory
{
	Q_OBJECT
public:

	explicit LensEditorFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual QString itemClassName() const;
	virtual Editor* factorizeEditor(QWidget* parent) const;

};


class PushBroomLenseEditor : public Editor {

    Q_OBJECT

public:

    explicit PushBroomLenseEditor(QWidget *parent = nullptr);
    ~PushBroomLenseEditor();

    void setCamera(PushBroomPinholeCamera* cam);

protected:

    void replot();

    QCustomPlot* _plot;
    PushBroomPinholeCamera* _cam;

};

class PushBroomLenseEditorFactory : public EditorFactory
{
    Q_OBJECT
public:

    explicit PushBroomLenseEditorFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

}//namespace StereoVisionApp

#endif // LENSEEDITOR_H
