#version 150

in vec3 in_location;
uniform vec2 in_id;
uniform float typeColorIndex;

uniform mat4 matrixViewProjection;
uniform mat4 matrixObjToScene;

uniform float sceneScale;

uniform float pointScale;

flat out vec3 vertex_id;

void main(void)
{
    vec4 p = matrixObjToScene * vec4(in_location, 1);
    p.w = 1/sceneScale;
    gl_Position = matrixViewProjection * p;

    gl_PointSize = pointScale; // set the size for points.
    vertex_id = vec3(in_id, typeColorIndex);
}
