#version 150

in float sequence_pos;

uniform float segmentStart;
uniform float segmentEnd;

uniform vec4 baseColor;
uniform vec4 segmentColor;

void main(void)
{
    gl_FragColor = baseColor;

    if (sequence_pos <= segmentEnd && sequence_pos >= segmentStart) {
        gl_FragColor = segmentColor;
    }
}
