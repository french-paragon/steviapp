#version 150

in vec4 segment_color;

void main(void)
{
    gl_FragColor = segment_color;
}
