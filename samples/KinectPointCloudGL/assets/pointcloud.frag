#version 150

in float vDepth;

out vec4 FragColor;

void main()
{
	//if( vDepth < 0.1 ) discard;
	FragColor.rgb	= vec3( vDepth );
	FragColor.a		= 1.0;
}





