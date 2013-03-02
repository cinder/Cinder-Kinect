#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Surface.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Rand.h"

#include "CinderFreenect.h"

using namespace ci;
using namespace ci::app;
using namespace std;


class kinectBasicApp : public AppBasic {
  public:
	void prepareSettings( Settings* settings );
	void setup();
	void mouseUp( MouseEvent event );
	void update();
	void draw();
	
	KinectRef		mKinect;
	gl::Texture		mColorTexture, mDepthTexture;	
};

void kinectBasicApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1280, 480 );
}

void kinectBasicApp::setup()
{
	console() << "There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;

	mKinect = Kinect::create();
}

void kinectBasicApp::mouseUp( MouseEvent event )
{
	writeImage( getHomeDirectory() / "kinect_video.png", mKinect->getVideoImage() );
	writeImage( getHomeDirectory() / "kinect_depth.png", mKinect->getDepthImage() );
	
	// set tilt to random angle
//	mKinect->setTilt( Rand::randFloat() * 62 - 31 );

	// make the LED yellow
//	mKinect->setLedColor( Kinect::LED_YELLOW );
	
	// toggle infrared video
	mKinect->setVideoInfrared( ! mKinect->isVideoInfrared() );
}

void kinectBasicApp::update()
{	
	if( mKinect->checkNewDepthFrame() )
		mDepthTexture = mKinect->getDepthImage();
	
	if( mKinect->checkNewVideoFrame() )
		mColorTexture = mKinect->getVideoImage();
	
//	console() << "Accel: " << mKinect.getAccel() << std::endl;
}

void kinectBasicApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	if( mDepthTexture )
		gl::draw( mDepthTexture );
	if( mColorTexture )
		gl::draw( mColorTexture, Vec2i( 640, 0 ) );
}


CINDER_APP_BASIC( kinectBasicApp, RendererGl )
