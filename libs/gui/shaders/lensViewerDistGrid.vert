#version 150

attribute vec2 in_location;

uniform mat4 frameTransform;

uniform float f;
uniform vec2 pp;
uniform vec3 k123;
uniform vec3 k456;
uniform vec2 p12;
uniform vec2 B12;

out vec2 dist;

vec2 lensDistortion(vec2 px,
                    vec2 pp,
                    vec3 k123,
                    vec3 k456,
                    vec2 p12,
                    vec2 B12) {

    vec2 delta_n = (px - pp)/f;

    float r2 = dot(delta_n, delta_n);

    //radial
    vec2 dist = dot(k123, vec3(r2, r2*r2, r2*r2*r2))/dot(vec4(1, k456), vec4(1, r2, r2*r2, r2*r2*r2))*delta_n;

    //tangential
    dist.x += p12.y*(r2 + 2*delta_n.x*delta_n.x) + 2*p12.x*delta_n.x*delta_n.y;
    dist.y += p12.x*(r2 + 2*delta_n.y*delta_n.y) + 2*p12.y*delta_n.x*delta_n.y;

    dist.x += dot(delta_n + dist, B12);

    return dist*f;
}

void main(void)
{
    dist = lensDistortion(in_location, pp, k123, k456, p12, B12);

    if (gl_VertexID%2 == 1) {
        gl_Position = frameTransform * vec4(in_location + dist, 0, 1);
    } else {
        gl_Position = frameTransform * vec4(in_location, 0, 1);
    }
}
