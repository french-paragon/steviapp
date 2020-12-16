#version 150

attribute vec3 in_location;

uniform mat4 matrixViewProjection;
uniform mat4 matrixCamToScene;

uniform float sceneScale;

void main(void)
{
    vec4 p = matrixCamToScene * vec4(in_location, 1);
    p.w = 1/sceneScale;
    gl_Position = matrixViewProjection * p;
}
