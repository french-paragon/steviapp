#version 150

attribute vec3 in_location;

uniform mat4 matrixViewProjection;

uniform float sceneScale;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1/sceneScale);
    gl_PointSize = 3.0;
}
