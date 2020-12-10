#version 150

attribute vec2 in_location;

uniform mat4 frameTransform;

void main(void)
{
    gl_Position = frameTransform * vec4(in_location, 0, 1);
}
