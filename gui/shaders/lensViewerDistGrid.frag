#version 150

in vec2 dist;

void main(void)
{
    float slide = clamp(dot(dist, dist)/16, 0., 1.);
    gl_FragColor = vec4(slide, 0.2, 0.3 + (1-slide)*0.7, 1.0);
}
