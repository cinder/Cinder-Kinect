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

#pragma once

#include "cinder/Cinder.h"
#include "cinder/Thread.h"
#include "cinder/Vector.h"
#include "cinder/Area.h"
#include "cinder/Exception.h"
#include "cinder/ImageIo.h"
#include <map>

// Forward declarations from freenect
//! @cond
struct _freenect_context;
typedef struct _freenect_context freenect_context;
struct _freenect_device;
typedef struct _freenect_device freenect_device;
typedef uint16_t freenect_depth;
typedef uint8_t freenect_pixel;
//! @endcond

namespace cinder {

typedef std::shared_ptr<class Kinect>	KinectRef;

class Kinect {
  public:
    
    // initialization parameters
    struct FreenectParams {
        
        FreenectParams() {
            mDeviceIndex = 0;
            mDepthRegister = false;
        }
        
        int 	mDeviceIndex;
        bool 	mDepthRegister;
    };
    
	//! Represents the identifier for a particular Kinect
	struct Device {
		Device( FreenectParams params = FreenectParams() )
			: mIndex( params.mDeviceIndex ),
              mDepthRegister ( params.mDepthRegister )
		{}
		
		int		mIndex;
        bool    mDepthRegister;
	};

	static KinectRef	create( const Device &device = Device() ) { return std::shared_ptr<Kinect>( new Kinect( device ) ); }
  
	//! Returns whether there is a new video frame available since the last call to checkNewVideoFrame(). Call getVideoImage() to retrieve it.
	bool		checkNewVideoFrame();
	//! Returns whether there is a new depth frame available since the last call to checkNewDepthFrame(). Call getDepthImage() to retrieve it.
	bool		checkNewDepthFrame();

	//! Sets the tilt of the motor measured in degrees. The device supports a range from -31 to +32 degrees.
	void		setTilt( float degrees );
	//! Returns the tilt measured in degrees
	float		getTilt() const;

	//! Returns the width of the captured image in pixels.
	int32_t		getWidth() const { return 640; }
	//! Returns the height of the captured image in pixels.
	int32_t		getHeight() const { return 480; }
	//! Returns the size of the captured image in pixels.
	Vec2i		getSize() const { return Vec2i( getWidth(), getHeight() ); }
	//! Returns the aspect ratio of the capture imagee, which is its width / height
	float		getAspectRatio() const { return getWidth() / (float)getHeight(); }
	//! Returns the bounding rectangle of the capture imagee, which is Area( 0, 0, width, height )
	Area		getBounds() const { return Area( 0, 0, getWidth(), getHeight() ); }
	
	typedef enum { LED_OFF = 0, LED_GREEN = 1, LED_RED = 2, LED_YELLOW = 3, LED_BLINK_YELLOW = 4, LED_BLINK_GREEN = 5, LED_BLINK_RED_YELLOW = 6 } LedColor;
	//! Sets the device's LED color/blink state
	void		setLedColor( LedColor ledColorCode );

	//! Returns the current accelerometer data, measured as meters/second<sup>2</sup>.
	Vec3f		getAccel() const;
	
	ImageSourceRef			getVideoImage();
	ImageSourceRef			getDepthImage();

	std::shared_ptr<uint8_t>	getVideoData();
	std::shared_ptr<uint16_t>	getDepthData();

	//! Sets the video image returned by getVideoImage() and getVideoData() to be infrared when \a infrared is true, color when it's false (the default)
	void		setVideoInfrared( bool infrared = true );
	//! Returns whether the video image returned by getVideoImage() and getVideoData() is infrared when \c true, or color when it's \c false (the default)
	bool		isVideoInfrared() const { return mObj->mVideoInfrared; }

	//! Sets whether the depth should be registered to the color image, by default it's true, however, if the setDepthRegistered() method is not called, then the depth is not registered to the RGB image
	//void		setDepthRegistered( bool registerDepth = true );
	//bool		isDepthRegistered() const { return mObj->mVideoInfrared; }


	//! Returns the number of Kinect devices attached to the system
	static int	getNumDevices();

	//! Parent class for all Kinect exceptions
	class Exc : cinder::Exception {
	};
	
	//! Exception thrown from a failure in freenect_init()
	class ExcFailedFreenectInit : public Exc {
	};

	//! Exception thrown from a failure to create a Freenect device
	class ExcFailedOpenDevice : public Exc {
	};
	
	static void		depthImageCB( freenect_device *dev, void *depth, uint32_t timestamp );
	static void		colorImageCB( freenect_device *dev, void *rgb, uint32_t timestamp );

	static freenect_context*	getContext();

	struct Obj {
		Obj( int deviceIndex, bool depthRegister );
		~Obj();
		
		template<typename T>
		struct BufferManager {
			BufferManager( size_t allocationSize, Obj *kinectObj )
				: mKinectObj( kinectObj ), mAllocationSize( allocationSize ), mActiveBuffer( 0 )
			{}
			~BufferManager();
			
			T*			getNewBuffer();
			void		setActiveBuffer( T *buffer );
			void		derefActiveBuffer();
			T*			refActiveBuffer();
			void		derefBuffer( T *buffer );

			Obj						*mKinectObj;
			size_t					mAllocationSize;
			// map from pointer to reference count
			std::map<T*,size_t>		mBuffers;
			T						*mActiveBuffer;
		};
				
		std::shared_ptr<std::thread>	mThread;
		std::recursive_mutex			mMutex;
		freenect_device					*mDevice;

		BufferManager<uint8_t>			mColorBuffers;
		BufferManager<uint16_t>			mDepthBuffers;
		
		volatile bool					mShouldDie;
		volatile bool					mVideoInfrared;
		volatile bool					mNewVideoFrame, mNewDepthFrame;
		volatile bool					mLastVideoFrameInfrared;
		float							mTilt;
	};

  protected:
	//! Creates a new Kinect based on Device # \a device. 0 is the typical value for \a deviceIndex.
	Kinect( Device device );

	friend class ImageSourceKinectColor;
	friend class ImageSourceKinectDepth;
	friend class ImageSourceKinectInfrared;

	static void			threadedFunc( struct Kinect::Obj *arg );
	
	static std::mutex				sContextMutex;
	static freenect_context			*sContext;	
	
	std::shared_ptr<Obj>			mObj;
};

} // namespace cinder
