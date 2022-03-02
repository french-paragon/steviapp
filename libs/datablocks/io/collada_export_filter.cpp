#include "collada_export_filter.h"

#include "../project.h"
#include "../camera.h"
#include "../landmark.h"
#include "../image.h"

#include "geometry/rotations.h"

namespace StereoVisionApp {

void writeCameraToCollada(QTextStream & oStream, Camera* cam) {

	QString headerTemplate("<camera id=\"Camera%1\" name=\"%2\">\n");

	oStream << headerTemplate.arg(cam->internalId()).arg(cam->objectName()).toUtf8();

	oStream << QString("<optics>\n").toUtf8();
	oStream << QString("<technique_common>\n").toUtf8();
	oStream << QString("<perspective>\n").toUtf8();

	QString xfovTemplate("<xfov sid=\"xfov\">%1</xfov>\n");
	QString aspectRatioTemplate("<aspect_ratio>%1</aspect_ratio>\n");

	QSize s = cam->imSize();

	floatParameter flenParam = cam->optimizedFLen();

	if (!flenParam.isSet()) {
		flenParam = cam->fLen();
	}

	float fLenPix = flenParam.value();
	float xfov = 2*std::atan((float(s.width())/2)/fLenPix);
	oStream << xfovTemplate.arg(xfov/M_PI*180).toUtf8();


	float aspect = float(s.width())/float(s.height());
	oStream << aspectRatioTemplate.arg(aspect).toUtf8();

	oStream << QString("<znear sid=\"znear\">0.1</znear>\n").toUtf8();
	oStream << QString("<zfar sid=\"zfar\">1000</zfar>\n").toUtf8();
	oStream << QString("</perspective>\n").toUtf8();
	oStream << QString("</technique_common>\n").toUtf8();
	oStream << QString("</optics>\n").toUtf8();
	oStream << QString("</camera>\n").toUtf8();
}


void writeLandmarkToCollada(QTextStream & oStream, Landmark* lm) {

	QString headerTemplate("<node id=\"Landmark%1\" name=\"%2\" type=\"NODE\">\n");
	oStream << headerTemplate.arg(lm->internalId()).arg(lm->objectName()).toUtf8();

	float pX;
	float pY;
	float pZ;

	floatParameterGroup<3> ot = lm->optPos();

	if (ot.isSet()) {
		pX = ot.value(0);
		pY = ot.value(1);
		pZ = ot.value(2);
	} else {
		pX = lm->xCoord().value();
		pY = lm->yCoord().value();
		pZ = lm->zCoord().value();
	}

	QString matrixTemplate("<matrix sid=\"transform\">1 0 0 %1 0 1 0 %2 0 0 1 %3 0 0 0 1</matrix>\n");
	oStream << matrixTemplate.arg(pX).arg(pY).arg(pZ).toUtf8();

	oStream << QString("</node>\n").toUtf8();
}

void writeViewToCollada(QTextStream & oStream, Image* view) {

	QString headerTemplate("<node id=\"View%1\" name=\"%2\" type=\"NODE\">\n");
	oStream << headerTemplate.arg(view->internalId()).arg(view->objectName()).toUtf8();

	float pX;
	float pY;
	float pZ;

	floatParameterGroup<3> ot = view->optPos();

	if (ot.isSet()) {
		pX = ot.value(0);
		pY = ot.value(1);
		pZ = ot.value(2);
	} else {
		pX = view->xCoord().value();
		pY = view->yCoord().value();
		pZ = view->zCoord().value();
	}

	float rX;
	float rY;
	float rZ;

	floatParameterGroup<3> oR = view->optRot();

	if (oR.isSet()) {
		rX = oR.value(0);
		rY = oR.value(1);
		rZ = oR.value(2);
	} else {
		rX = view->xRot().value();
		rY = view->yRot().value();
		rZ = view->zRot().value();
	}

	Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
	S(1,1) = -1;
	S(2,2) = -1;

	Eigen::Matrix3f R = StereoVision::Geometry::rodriguezFormula(Eigen::Vector3f(rX, rY, rZ))*S;
	Eigen::Vector3f t(pX, pY, pZ);

	QString matrixLineTemplate("%1 %2 %3 %4");
	QString matrixTemplate("<matrix sid=\"transform\">%1 %2 %3 0 0 0 1</matrix>\n");

	for (int i = 0; i < 3; i++) {
		QString l = matrixLineTemplate.arg(R(i,0)).arg(R(i,1)).arg(R(i,2)).arg(t[i]);
		matrixTemplate = matrixTemplate.arg(l);
	}

	oStream << matrixTemplate.toUtf8();

	QString camInstanceTemplate("<instance_camera url=\"#Camera%1\"/>\n");
	oStream << camInstanceTemplate.arg(view->assignedCamera());

	oStream << QString("</node>\n").toUtf8();
}

void exportOptimizedToCollada(QTextStream & oStream, Project* p, bool optimizedOnly = true) {

	oStream << QString("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n").toUtf8();
	oStream << QString("<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n").toUtf8();


	oStream << QString("<library_cameras>\n").toUtf8();

	QVector<qint64> cams = p->getIdsByClass(Camera::staticMetaObject.className());

	for (qint64 id : cams) {
		Camera* cam = p->getDataBlock<Camera>(id);
		if (cam != nullptr) {
			writeCameraToCollada(oStream, cam);
		}
	}

	oStream << QString("</library_cameras>\n").toUtf8();

	oStream << QString("<library_images/>\n");

	oStream << QString("<library_visual_scenes>\n");
	oStream << QString("<visual_scene id=\"Scene\" name=\"Scene\">\n");


	QVector<qint64> landmarks = p->getIdsByClass(Landmark::staticMetaObject.className());

	for (qint64 id : landmarks) {
		Landmark* lm = p->getDataBlock<Landmark>(id);
		if (lm != nullptr) {
			if (optimizedOnly) {
				if (!lm->hasOptimizedParameters()) {
					continue;
				}
			}

			writeLandmarkToCollada(oStream, lm);
		}
	}


	QVector<qint64> views = p->getIdsByClass(Image::staticMetaObject.className());

	for (qint64 id : views) {
		Image* im = p->getDataBlock<Image>(id);
		if (im != nullptr) {
			if (optimizedOnly) {
				if (!im->hasOptimizedParameters()) {
					continue;
				}
			}

			writeViewToCollada(oStream, im);
		}
	}

	oStream << QString("</visual_scene>\n");
	oStream << QString("</library_visual_scenes>\n");

	oStream << QString("</COLLADA>").toUtf8();

}

} //namespace StereoVisionApp
