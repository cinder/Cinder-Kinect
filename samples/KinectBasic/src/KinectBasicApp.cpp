#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
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


class kinectBasicApp : public App {
  public:

    kinectBasicApp();
    void mouseUp( MouseEvent event )override;
	void update()override;
	void draw()override;
	
	KinectRef           mKinect;
	gl::TextureRef		mColorTexture, mDepthTexture;
};



kinectBasicApp::kinectBasicApp()
{
	console() << "There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;

    mKinect = Kinect::create();

    mColorTexture = gl::Texture::create(mKinect->getWidth(), mKinect->getHeight(), gl::Texture::Format().internalFormat(GL_RGB).dataType(GL_UNSIGNED_BYTE) );
    
    mDepthTexture = gl::Texture::create(mKinect->getWidth(), mKinect->getHeight(), gl::Texture::Format().internalFormat(GL_RED).dataType(GL_UNSIGNED_SHORT) );
    
}

void kinectBasicApp::mouseUp( MouseEvent event )
{
	writeImage( getHomeDirectory() / "kinect_video.png", mKinect->getVideoImage() );
	writeImage( getHomeDirectory() / "kinect_depth.png", mKinect->getDepthImage() );
	
	// make the LED yellow
    mKinect->setLedColor( Kinect::LED_YELLOW );
	
	// toggle infrared video
	mKinect->setVideoInfrared( ! mKinect->isVideoInfrared() );
}

void kinectBasicApp::update()
{
    
	if( mKinect->checkNewDepthFrame() )
		mDepthTexture->update( Channel16u( mKinect->getDepthImage() ) );
	
	if( mKinect->checkNewVideoFrame() )
		mColorTexture->update( Surface8u( mKinect->getVideoImage() ) );
	
	console() << "Accel: " << mKinect->getAccel() << std::endl;
}

void kinectBasicApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
    
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
    
	if( mDepthTexture )
		gl::draw( mDepthTexture );
	if( mColorTexture )
		gl::draw( mColorTexture, ivec2( 640, 0 ) );
}

void prepareSettings( App::Settings* settings )
{
    settings->setWindowSize( 1280, 480 );
}

CINDER_APP( kinectBasicApp, RendererGl, prepareSettings )
