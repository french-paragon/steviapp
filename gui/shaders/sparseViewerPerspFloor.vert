#version 150

attribute vec2 in_location;

uniform mat4 matrixViewProjection;

out vec4 lineColor;

void main(void)
{
    lineColor = vec4(0.9, 0.9, 0.9, 1.0);

    if (abs(in_location.y) < 1e-3) {
        lineColor.rgb = vec3(0.9, 0.2, 0.2);
    } else if (abs(in_location.x) < 1e-3) {
        lineColor.rgb = vec3(0.2, 0.9, 0.2);
    }

    gl_Position = matrixViewProjection * vec4(in_location, 0, 1);
}
