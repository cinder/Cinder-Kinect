/*
 Copyright (c) 2010, The Cinder Project, All rights reserved.

 This code is intended for use with the Cinder C++ library: http://libcinder.org

 Portions copyright Rui Madeira: http://ruim.pt

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/


#include "Kinect.h"
#include "libfreenect.h"
using namespace std;

namespace cinder {

// statics
std::mutex			Kinect::sContextMutex;
freenect_context*	Kinect::sContext = 0;

class ImageSourceKinectColor : public ImageSource {
  public:
	ImageSourceKinectColor( uint8_t *buffer, shared_ptr<Kinect::Obj> ownerObj )
		: ImageSource(), mData( buffer ), mOwnerObj( ownerObj )
	{
		setSize( 640, 480 );
		setColorModel( ImageIo::CM_RGB );
		setChannelOrder( ImageIo::RGB );
		setDataType( ImageIo::UINT8 );
	}

	~ImageSourceKinectColor()
	{
		// let the owner know we are done with the buffer
		mOwnerObj->mColorBuffers.derefBuffer( mData );
	}

	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
		
		for( int32_t row = 0; row < 480; ++row )
			((*this).*func)( target, row, mData + row * 640 * 3 );
	}
	
  protected:
	shared_ptr<Kinect::Obj>		mOwnerObj;
	uint8_t						*mData;
};

class ImageSourceKinectDepth : public ImageSource {
  public:
	ImageSourceKinectDepth( uint16_t *buffer, shared_ptr<Kinect::Obj> ownerObj )
		: ImageSource(), mData( buffer ), mOwnerObj( ownerObj )
	{
		setSize( 640, 480 );
		setColorModel( ImageIo::CM_GRAY );
		setChannelOrder( ImageIo::Y );
		setDataType( ImageIo::UINT16 );
	}

	~ImageSourceKinectDepth()
	{
		// let the owner know we are done with the buffer
		mOwnerObj->mDepthBuffers.derefBuffer( mData );
	}

	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
		
		for( int32_t row = 0; row < 480; ++row )
			((*this).*func)( target, row, mData + row * 640 );
	}
	
  protected:
	shared_ptr<Kinect::Obj>		mOwnerObj;
	uint16_t					*mData;
};

template<typename T>
class KinectDataDeleter {
  public:
	KinectDataDeleter( Kinect::Obj::BufferManager<T> *bufferMgr, shared_ptr<Kinect::Obj> ownerObj )
		: mBufferMgr( bufferMgr ), mOwnerObj( ownerObj )
	{}
	
	void operator()( T *data ) {
		mBufferMgr->derefBuffer( data );
	}
	
	shared_ptr<Kinect::Obj>			mOwnerObj; // to prevent deletion of our parent Obj
	Kinect::Obj::BufferManager<T> *mBufferMgr;
};

Kinect::Kinect( int deviceIndex )
	: mObj( new Obj( deviceIndex ) )
{
}

Kinect::Obj::Obj( int deviceIndex )
	: mShouldDie( false ), mNewColorFrame( false ), mNewDepthFrame( false ), 
		mColorBuffers( 640 * 480 * 3, this ), mDepthBuffers( 640 * 480, this )
{
	if( freenect_open_device( getContext(), &mDevice, deviceIndex ) < 0 )
		throw ExcFailedOpenDevice();

	freenect_set_user( mDevice, this );
	freenect_set_tilt_degs( mDevice, 0 );
	freenect_set_led( mDevice, ::LED_GREEN );
	freenect_set_depth_callback( mDevice, depthImageCB );
	freenect_set_rgb_callback( mDevice, colorImageCB );
	freenect_set_rgb_format( mDevice, FREENECT_FORMAT_RGB );
	freenect_set_depth_format( mDevice, FREENECT_FORMAT_11_BIT );

	mTilt = 0;
	
	mThread = shared_ptr<thread>( new thread( threadedFunc, this ) );
}

Kinect::Obj::~Obj()
{
	mShouldDie = true;
	mThread->join();
}

void Kinect::colorImageCB( freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp )
{
	Kinect::Obj *kinectObj = reinterpret_cast<Kinect::Obj*>( freenect_get_user( dev ) );
	uint8_t *destPixels = kinectObj->mColorBuffers.getNewBuffer();
	{
		lock_guard<mutex> lock( kinectObj->mMutex );
		memcpy( destPixels, rgb, 640 * 480 * 3 * sizeof(uint8_t) );
		kinectObj->mNewColorFrame = true;
	}
	kinectObj->mColorBuffers.setActiveBuffer( destPixels );
}

void Kinect::depthImageCB( freenect_device *dev, freenect_depth *depth, uint32_t timestamp )
{
	Kinect::Obj *kinectObj = reinterpret_cast<Kinect::Obj*>( freenect_get_user( dev ) );
	uint16_t *destPixels = kinectObj->mDepthBuffers.getNewBuffer();
	{
		lock_guard<mutex> lock( kinectObj->mMutex );
		for( size_t p = 0; p < 640 * 480; ++p ) { // out = 1.0 - ( in / 2048 ) ^ 2
			uint32_t v = depth[p];
			destPixels[p] = 65535 - ( v * v ) >> 4; // 1 / ( 2^10 * 2^10 ) * 2^16 = 2^-4
		}
		kinectObj->mNewDepthFrame = true;
	}
	kinectObj->mDepthBuffers.setActiveBuffer( destPixels );
}

void Kinect::threadedFunc( Kinect::Obj *kinectObj )
{
	freenect_start_depth( kinectObj->mDevice );
	freenect_start_rgb( kinectObj->mDevice );

	while( ( ! kinectObj->mShouldDie ) && ( freenect_process_events( getContext() ) >= 0 ) )
		;
		
	freenect_close_device( kinectObj->mDevice );
}

freenect_context* Kinect::getContext()
{
	// ultimately this should be replaced with a call_once
	lock_guard<mutex> contextLock( sContextMutex );
	if( ! sContext ) {
		if( freenect_init( &sContext, NULL ) < 0 );
			; // throw ExcFailedFreenectInit(); // this seems to always fail
	}
	return sContext;
}

int	Kinect::getNumDevices()
{
	try {
		return freenect_num_devices( getContext() );
	}
	catch( ExcFailedFreenectInit &e ) { // a failed initialization implies 0 kinects.
		return 0;
	}
}

bool Kinect::checkNewColorFrame()
{
	lock_guard<mutex> lock( mObj->mMutex );
	bool oldValue = mObj->mNewColorFrame;
	mObj->mNewColorFrame = false;
	return oldValue;
}

bool Kinect::checkNewDepthFrame()
{
	lock_guard<mutex> lock( mObj->mMutex );
	bool oldValue = mObj->mNewDepthFrame;
	mObj->mNewDepthFrame = false;
	return oldValue;
}

void Kinect::setTilt( float degrees )
{
	mObj->mTilt = math<float>::clamp( degrees, -31, 31 );
	freenect_set_tilt_degs( mObj->mDevice, mObj->mTilt );
}

float Kinect::getTilt() const
{
	return mObj->mTilt;
}

void Kinect::setLedColor( LedColor ledColorCode )
{
	int code = ledColorCode;
	freenect_set_led( mObj->mDevice, (freenect_led_options)code );
}

Vec3f Kinect::getAccel() const
{
	Vec3d raw;
	freenect_get_mks_accel( mObj->mDevice, &raw.x, &raw.y, &raw.z );
	return Vec3f( raw );
}

ImageSourceRef Kinect::getColorImage()
{
	// register a reference to the active buffer
	uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
	return ImageSourceRef( new ImageSourceKinectColor( activeColor, this->mObj ) );
}

ImageSourceRef Kinect::getDepthImage()
{
	// register a reference to the active buffer
	uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
	return ImageSourceRef( new ImageSourceKinectDepth( activeDepth, this->mObj ) );
}

std::shared_ptr<uint8_t> Kinect::getColorData()
{
	// register a reference to the active buffer
	uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
	return shared_ptr<uint8_t>( activeColor, KinectDataDeleter<uint8_t>( &mObj->mColorBuffers, mObj ) );	
}

std::shared_ptr<uint16_t> Kinect::getDepthData()
{
	// register a reference to the active buffer
	uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
	return shared_ptr<uint16_t>( activeDepth, KinectDataDeleter<uint16_t>( &mObj->mDepthBuffers, mObj ) );	
}

// Buffer management
template<typename T>
Kinect::Obj::BufferManager<T>::~BufferManager()
{
	for( typename map<T*,size_t>::iterator bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
		delete [] bufIt->first;
	}
}

template<typename T>
T* Kinect::Obj::BufferManager<T>::getNewBuffer()
{
	lock_guard<mutex> lock( mKinectObj->mMutex );

	typename map<T*,size_t>::iterator bufIt;
	for( bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
		if( bufIt->second == 0 ) // 0 means free buffer
			break;
	}
	if( bufIt != mBuffers.end() ) {
		bufIt->second = 1;
		return bufIt->first;
	}
	else { // there were no available buffers - add a new one and return it
		T *newBuffer = new T[mAllocationSize];
		mBuffers[newBuffer] = 1;
		return newBuffer;
	}
}

template<typename T>
void Kinect::Obj::BufferManager<T>::setActiveBuffer( T *buffer )
{
	lock_guard<mutex> lock( mKinectObj->mMutex );
	// decrement use count on current active buffer
	mBuffers[mActiveBuffer]--;
	// assign new active buffer
	mActiveBuffer = buffer;
}

template<typename T>
T* Kinect::Obj::BufferManager<T>::refActiveBuffer()
{
	lock_guard<mutex> lock( mKinectObj->mMutex );
	mBuffers[mActiveBuffer]++;
	return mActiveBuffer;
}

template<typename T>
void Kinect::Obj::BufferManager<T>::derefBuffer( T *buffer )
{
	lock_guard<mutex> lock( mKinectObj->mMutex );
	mBuffers[buffer]--;
}

} // namespace cinder