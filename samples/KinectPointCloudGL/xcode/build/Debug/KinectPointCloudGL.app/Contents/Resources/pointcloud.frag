#version 150

in float vDepthColor;

out vec4 FragColor;

void main()
{
	//if( vDepth < 0.1 ) discard;
	FragColor.rgb	= vec3( vDepthColor );
	FragColor.a		= 1.0;
}





