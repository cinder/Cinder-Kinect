uniform sampler2D depthTex;
varying vec4 vVertex;
varying float depth;

void main()
{
	gl_TexCoord[0]		= gl_MultiTexCoord0;
	vVertex				= vec4( gl_Vertex );
	
	depth				= texture2D( depthTex, gl_TexCoord[0].st ).b;
	
	vVertex.z			+= depth * 1000.0;

	gl_Position			= gl_ModelViewProjectionMatrix * vVertex;
}
