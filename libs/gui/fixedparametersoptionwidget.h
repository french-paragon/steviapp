#ifndef FIXEDPARAMETERSOPTIONWIDGET_H
#define FIXEDPARAMETERSOPTIONWIDGET_H

#include <QWidget>

#include "sparsesolver/fixedpreoptimizedparameters.h"

namespace StereoVisionApp {

namespace Ui {
class FixedParametersOptionWidget;
}

class FixedParametersOptionWidget : public QWidget
{
	Q_OBJECT

public:
	explicit FixedParametersOptionWidget(QWidget *parent = nullptr);
	~FixedParametersOptionWidget();

	FixedParameters getFixedParameters() const;
	void setFixedParameters(FixedParameters parameters);

private:
	Ui::FixedParametersOptionWidget *ui;
};

} // namespace StereoVisionApp

#endif // FIXEDPARAMETERSOPTIONWIDGET_H
