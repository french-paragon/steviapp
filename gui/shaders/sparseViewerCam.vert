#version 150

attribute vec3 in_location;

uniform mat4 matrixViewProjection;
uniform mat4 matrixCamToScene;

void main(void)
{
    gl_Position = matrixViewProjection * matrixCamToScene * vec4(in_location, 1);
}
