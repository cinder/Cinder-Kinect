#include "cinder/app/AppBasic.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIo.h"
#include "Kinect.h"
#include "Resources.h"

static const int VBO_X_RES  = 640;
static const int VBO_Y_RES  = 480;

using namespace ci;
using namespace ci::app;
using namespace std;

class kinectPointCloudApp : public AppBasic {
  public:
	void prepareSettings( Settings* settings );
	void setup();
	void createVbo();
	void update();
	void draw();
	
	// PARAMS
	params::InterfaceGl	mParams;
	
	// CAMERA
	CameraPersp		mCam;
	Quatf			mSceneRotation;
	Vec3f			mEye, mCenter, mUp;
	float			mCameraDistance;
	float			mKinectTilt;
	
	// KINECT AND TEXTURES
	Kinect			mKinect;
	gl::Texture		mDepthTexture;
	float			mScale;
	float			mXOff, mYOff;
	
	// VBO AND SHADER
	gl::VboMesh		mVboMesh;
	gl::GlslProg	mShader;
};

void kinectPointCloudApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1280, 720 );
}

void kinectPointCloudApp::setup()
{	
	// SETUP PARAMS
	mParams = params::InterfaceGl( "KinectPointCloud", Vec2i( 200, 180 ) );
	mParams.addParam( "Scene Rotation", &mSceneRotation, "opened=1" );
	mParams.addParam( "Cam Distance", &mCameraDistance, "min=100.0 max=5000.0 step=100.0 keyIncr=s keyDecr=w" );
	mParams.addParam( "Kinect Tilt", &mKinectTilt, "min=-31 max=31 keyIncr=T keyDecr=t" );
	
	// SETUP CAMERA
	mCameraDistance = 1000.0f;
	mEye			= Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCenter			= Vec3f::zero();
	mUp				= Vec3f::yAxis();
	mCam.setPerspective( 75.0f, getWindowAspectRatio(), 1.0f, 8000.0f );
	
	// SETUP KINECT AND TEXTURES
	console() << "There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;
	mKinect			= Kinect( Kinect::Device() ); // use the default Kinect
	mDepthTexture	= gl::Texture( 640, 480 );
	
	// SETUP VBO AND SHADER	
	createVbo();
	mShader	= gl::GlslProg( loadResource( RES_VERT_ID ), loadResource( RES_FRAG_ID ) );
	
	// SETUP GL
	gl::enableDepthWrite();
	gl::enableDepthRead();
}

void kinectPointCloudApp::createVbo()
{
	gl::VboMesh::Layout layout;
	
	layout.setStaticPositions();
	layout.setStaticTexCoords2d();
	layout.setStaticIndices();
	
	std::vector<Vec3f> positions;
	std::vector<Vec2f> texCoords;
	std::vector<uint32_t> indices; 
	
	int numVertices = VBO_X_RES * VBO_Y_RES;
	int numShapes	= ( VBO_X_RES - 1 ) * ( VBO_Y_RES - 1 );

	mVboMesh		= gl::VboMesh( numVertices, numShapes, layout, GL_POINTS );
	
	for( int x=0; x<VBO_X_RES; ++x ){
		for( int y=0; y<VBO_Y_RES; ++y ){
			indices.push_back( x * VBO_Y_RES + y );

			float xPer	= x / (float)(VBO_X_RES-1);
			float yPer	= y / (float)(VBO_Y_RES-1);
			
			positions.push_back( Vec3f( ( xPer * 2.0f - 1.0f ) * VBO_X_RES, ( yPer * 2.0f - 1.0f ) * VBO_Y_RES, 0.0f ) );
			texCoords.push_back( Vec2f( xPer, yPer ) );			
		}
	}
	
	mVboMesh.bufferPositions( positions );
	mVboMesh.bufferIndices( indices );
	mVboMesh.bufferTexCoords2d( 0, texCoords );
}

void kinectPointCloudApp::update()
{
	if( mKinect.checkNewDepthFrame() )
		mDepthTexture = mKinect.getDepthImage();
	
	// This sample does not use the color data
	//if( mKinect.checkNewVideoFrame() )
	//	mColorTexture = mKinect.getVideoImage();

	if( mKinectTilt != mKinect.getTilt() )
		mKinect.setTilt( mKinectTilt );
		
	mEye = Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCam.lookAt( mEye, mCenter, mUp );
	gl::setMatrices( mCam );
}

void kinectPointCloudApp::draw()
{
	gl::clear( Color( 0.0f, 0.0f, 0.0f ), true );
	
	gl::pushMatrices();
		gl::scale( Vec3f( -1.0f, -1.0f, 1.0f ) );
		gl::rotate( mSceneRotation );
		mDepthTexture.bind( 0 );
		mShader.bind();
		mShader.uniform( "depthTex", 0 );
		gl::draw( mVboMesh );
		mShader.unbind();
	gl::popMatrices();
	
	params::InterfaceGl::draw();
}


CINDER_APP_BASIC( kinectPointCloudApp, RendererGl )
