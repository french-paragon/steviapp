#version 150

attribute vec3 in_location;

uniform mat4 matrixViewProjection;
uniform mat4 matrixCamToScene;

uniform float sceneScale;

out vec4 lineColor;

void main(void)
{
    lineColor = vec4(in_location.x > 0 ? 1.0 : 0,
                     in_location.y > 0 ? 1.0 : 0,
                     in_location.z > 0 ? 1.0 : 0,
                     1.0);

    vec4 p = matrixCamToScene * vec4(in_location, 1);
    p.w = 1/sceneScale;
    gl_Position = matrixViewProjection * p;
}
