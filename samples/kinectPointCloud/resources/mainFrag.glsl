#version 110
varying float depth;

void main()
{
//	if( depth < 0.1 ) discard;

	gl_FragColor.rgb	= vec3( depth );
	gl_FragColor.a		= 1.0;
}





