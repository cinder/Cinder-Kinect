#version 150

in vec4 ciPosition;
in vec2 ciTexCoord0;

uniform sampler2D  uDepthTexture;
uniform mat4       ciModelViewProjection;

out float vDepth;

void main()
{
    vec4 pos		= ciPosition;
    vDepth			= texture( uDepthTexture, ciTexCoord0 ).r;
    pos.z           = vDepth * 1000.;
	gl_Position		= ciModelViewProjection * pos;
}
