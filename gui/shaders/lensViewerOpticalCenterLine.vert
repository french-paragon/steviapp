#version 150

attribute vec2 in_location;

uniform mat4 frameTransform;
uniform vec2 opticalCenter;
uniform vec2 frameSize;

void main(void)
{
    vec4 v = vec4(in_location, 0, 1);

    if (v.x > 0.25 && v.x < 0.75) {
        v.x = opticalCenter.x;
    } else if (v.x > 0.75) {
        v.x = frameSize.x;
    } else {
        v.x = 0;
    }

    if (v.y > 0.25 && v.y < 0.75) {
        v.y = opticalCenter.y;
    } else if (v.y > 0.75) {
        v.y = frameSize.y;
    } else {
        v.y = 0;
    }

    gl_Position = frameTransform * v;
}
