#version 150

flat in vec3 vertex_id;

out vec3 id_color;

void main(void)
{
    id_color = vertex_id;
}
