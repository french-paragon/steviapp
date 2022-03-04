#ifndef STEREOVISIONAPP_VERTEXLANDMARKPOS_H
#define STEREOVISIONAPP_VERTEXLANDMARKPOS_H

#include "g2o/types/sba/types_sba.h"

#include "../../datablocks/project.h"

namespace StereoVisionApp {

class VertexLandmarkPos : public g2o::VertexSBAPointXYZ, public DataBlockReference
{
public:
	VertexLandmarkPos();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_VERTEXLANDMARKPOS_H
