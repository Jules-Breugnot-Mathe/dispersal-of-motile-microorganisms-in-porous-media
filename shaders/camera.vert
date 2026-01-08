#version 330 core

layout (location = 0) in vec2 aPos;

uniform mat4 MVP;
uniform vec2 modelOffset;

void main()
{
    vec2 worldPos = aPos + modelOffset;
    gl_Position = MVP * vec4(worldPos, 0.0, 1.0);
}
