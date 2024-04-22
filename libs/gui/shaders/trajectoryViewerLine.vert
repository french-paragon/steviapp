#version 150

in vec3 in_location;
in int in_id;

uniform mat4 matrixViewProjection;

uniform float sceneScale;

out float sequence_pos;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1/sceneScale);
    gl_PointSize = 3.0;

    sequence_pos = float(in_id);
}
