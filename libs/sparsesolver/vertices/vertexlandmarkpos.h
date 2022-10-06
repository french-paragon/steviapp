#ifndef STEREOVISIONAPP_VERTEXLANDMARKPOS_H
#define STEREOVISIONAPP_VERTEXLANDMARKPOS_H

#include "g2o/types/slam3d/vertex_pointxyz.h"

#include "../../datablocks/project.h"

namespace StereoVisionApp {

class VertexLandmarkPos : public g2o::VertexPointXYZ, public DataBlockReference
{
public:
	VertexLandmarkPos();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_VERTEXLANDMARKPOS_H
