#version 150

in vec3 in_location;
in int in_id;

uniform mat4 matrixViewProjection;
uniform mat4 matrixHandleTransform;

uniform float sceneScale;
uniform float camScale;

out vec4 segment_color;

uniform int mode;

uniform float segmentStart;
uniform float segmentEnd;

uniform vec4 baseColor;
uniform vec4 segmentColor;

uniform vec4 orientHandleXColor;
uniform vec4 orientHandleYColor;
uniform vec4 orientHandleZColor;

void main(void)
{
    if (mode == 0) { //drawing the lines
        gl_Position = matrixViewProjection * vec4(in_location, 1/sceneScale);
        gl_PointSize = 3.0;

        float sequence_pos = float(in_id);

        segment_color = baseColor;

        if (sequence_pos <= segmentEnd && sequence_pos >= segmentStart) {
            segment_color = segmentColor;
        }

    } else { //drawing the orientations handles

        vec4 p  = matrixHandleTransform * vec4(in_location, 1);
        p.w = 1/sceneScale;
        gl_Position = matrixViewProjection * p;
        gl_PointSize = 4.0;

        segment_color = orientHandleXColor; //red for x axis (default)

        if (in_id == 1) { //green for y axis
            segment_color = orientHandleYColor;
        }

        if (in_id == 2) { //blue for z axis
            segment_color = orientHandleZColor;
        }
    }
}
