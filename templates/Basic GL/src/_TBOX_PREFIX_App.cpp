#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Surface.h"

#include "CinderFreenect.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_App : public AppBasic {
  public:
	void prepareSettings( Settings* settings );
	void setup();
	void update();
	void draw();
	
	Kinect			mKinect;
	gl::Texture		mColorTexture, mDepthTexture;	
};

void _TBOX_PREFIX_App::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1280, 480 );
}

void _TBOX_PREFIX_App::setup()
{
	console() << "There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;

    Kinect::FreenectParams params;
    params.mDepthRegister = true;
    params.mDeviceIndex = 0;
    
	mKinect = Kinect( Kinect::Device(params) ); // the default Device implies the first Kinect connected
}

void _TBOX_PREFIX_App::update()
{	
	if( mKinect.checkNewDepthFrame() )
		mDepthTexture = mKinect.getDepthImage();
	
	if( mKinect.checkNewVideoFrame() )
		mColorTexture = mKinect.getVideoImage();
	
//	console() << "Accel: " << mKinect.getAccel() << std::endl;
}

void _TBOX_PREFIX_App::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	if( mDepthTexture )
		gl::draw( mDepthTexture );
	if( mColorTexture )
		gl::draw( mColorTexture, Vec2i( 640, 0 ) );
}

CINDER_APP_BASIC( _TBOX_PREFIX_App, RendererGl )
