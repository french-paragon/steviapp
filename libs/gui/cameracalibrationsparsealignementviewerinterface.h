#ifndef STEREOVISIONAPP_CAMERACALIBRATIONSPARSEALIGNEMENTVIEWERINTERFACE_H
#define STEREOVISIONAPP_CAMERACALIBRATIONSPARSEALIGNEMENTVIEWERINTERFACE_H

#include "../datablocks/cameracalibration.h"
#include "./sparsealignementviewer.h"

namespace StereoVisionApp {

class CameraCalibrationSparseAlignementViewerInterface : public AbstractSparseAlignementDataInterface
{
	Q_OBJECT
public:
	explicit CameraCalibrationSparseAlignementViewerInterface(float gridSize, QObject *parent = nullptr);

	void setCalibration(CameraCalibration* p);
	void clearCalibration();

    int nCameras() const override;
    int nPoints() const override;
    int nLocalSystems() const override;

    QMatrix4x4 getCameraTransform(int idx) const override;
    QMatrix4x4 getLocalSystemTransform(int idx) const override;
	QVector3D getPointPos(int idx) const override;

	float getGridSize() const;
	void setGridSize(float grid_size);

	void reload() override;

Q_SIGNALS:

protected:

    void hooverPoint(int idx) const override;
    void hooverCam(int idx) const override;
    void hooverLocalCoord(int idx) const override;

	void clickPoint(int idx) const override;
	void clickCam(int idx) const override;
    void clickLocalCoordinateSystem(int idx) const override;

	CameraCalibration* _currentCalibration;
	float _grid_size;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMERACALIBRATIONSPARSEALIGNEMENTVIEWERINTERFACE_H
