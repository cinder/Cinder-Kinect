#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIo.h"
#include "CinderFreenect.h"
#include "Resources.h"

static const int VBO_X_RES  = 640;
static const int VBO_Y_RES  = 480;

using namespace ci;
using namespace ci::app;
using namespace std;

class PointCloudGl : public App {
public:
    
    PointCloudGl();
    void           update()override;
    void           draw()override;
    
    gl::VboMeshRef createVboMesh();
    
    // PARAMS
    params::InterfaceGlRef	mParams;
    
    // CAMERA
    CameraPersp		mCam;
    quat			mSceneRotation;
    vec3			mEye, mCenter, mUp;
    float			mCameraDistance;
    float			mKinectTilt;
    
    // KINECT AND TEXTURES
    KinectRef		mKinect;
    gl::TextureRef  mDepthTexture;
    float			mScale;
    float			mXOff, mYOff;
    
    // BATCH AND SHADER
    gl::BatchRef    mPointCloud;
    gl::GlslProgRef	mPointCloudShader;
};


PointCloudGl::PointCloudGl()
{
    // SETUP PARAMS
    mParams = params::InterfaceGl::create( "KinectPointCloud", vec2( 200, 180 ) );
    mParams->addParam( "Scene Rotation", &mSceneRotation, "opened=1" );
    mParams->addParam( "Cam Distance", &mCameraDistance, "step=.1 keyIncr=s keyDecr=w" );
    mParams->addParam( "Kinect Tilt", &mKinectTilt, "min=-31 max=31 keyIncr=T keyDecr=t" );
    
    // SETUP CAMERA
    mCameraDistance = 10.0f;
    mEye			= vec3( -10.0f, 10.0f, mCameraDistance );
    mCenter			= vec3(0,0,-5.);
    mUp				= vec3(0,1,0);
    mCam.setPerspective( 75.0f, getWindowAspectRatio(), 1.0f, 100000.0f );
    mCam.lookAt(mEye, mCenter, mUp);
    
    // SETUP KINECT AND TEXTURES
    mKinect			= Kinect::create(); // use the default Kinect
    mDepthTexture	= gl::Texture::create( mKinect->getWidth(), mKinect->getHeight(), gl::Texture::Format().internalFormat(GL_R16UI).dataType(GL_UNSIGNED_SHORT).minFilter(GL_NEAREST).magFilter(GL_NEAREST) );
    
    // SETUP VBO AND SHADER
    
    try {
        mPointCloudShader	= gl::GlslProg::create( loadAsset( "pointcloud.vert" ), loadAsset( "pointcloud.frag" ) );
    } catch (const gl::GlslProgCompileExc &e) {
        console() << e.what() << endl;
    }
    
    auto mesh = createVboMesh();
    mPointCloud = gl::Batch::create( mesh, mPointCloudShader );
    mPointCloudShader->uniform("uDepthTexture", 0);
    mPointCloudShader->uniform( "ref_pix_size", mKinect->getZeroPlanePixelSize() );
    mPointCloudShader->uniform( "ref_distance", mKinect->getZeroPlaneDistance() );
    mPointCloudShader->uniform( "const_shift", mKinect->getRegistrationConstShift() );
    mPointCloudShader->uniform( "dcmos_emitter_dist", mKinect->getDcmosEmitterDist() );

    // SETUP GL
    gl::enableDepthWrite();
    gl::enableDepthRead();
        
}

gl::VboMeshRef PointCloudGl::createVboMesh()
{
    
    vector<float> data;
    
    int numVertices = VBO_X_RES * VBO_Y_RES;
    
    for( int x=0; x<VBO_X_RES; ++x ){
        for( int y=0; y<VBO_Y_RES; ++y ){
            
            float xPer	= x / (float)(VBO_X_RES-1);
            float yPer	= y / (float)(VBO_Y_RES-1);
            
            auto position = vec3( x, y, 0. );//vec3( ( xPer * 2.0f - 1.0f ) * VBO_X_RES, ( yPer * 2.0f - 1.0f ) * VBO_Y_RES, 0.0f );
            auto tc = vec2( xPer, yPer );
            
            data.push_back(position.x);
            data.push_back(position.y);
            data.push_back(position.z);
            data.push_back(tc.x);
            data.push_back(tc.y);
            
        }
    }
    
    geom::BufferLayout data_layout;
    data_layout.append(geom::POSITION, 3, sizeof(float)*5, 0);
    data_layout.append(geom::TEX_COORD_0, 2, sizeof(float)*5, sizeof(float)*3);
    
    auto data_buffer = gl::Vbo::create(GL_ARRAY_BUFFER, sizeof(float)*data.size(), data.data(), GL_STATIC_DRAW);
    
    vector<pair<geom::BufferLayout, gl::VboRef>> layouts( 1, make_pair(data_layout, data_buffer) );
    
    return gl::VboMesh::create( numVertices, GL_POINTS, layouts );
    
}

void PointCloudGl::update()
{
    if( mKinect->checkNewDepthFrame() )
        mDepthTexture->update( Channel16u( mKinect->getDepthImage() ) );
    
    if( mKinectTilt != mKinect->getTilt() )
        mKinect->setTilt( mKinectTilt );
    
}

void PointCloudGl::draw()
{
    gl::clear( Color( 0.0f, 0.0f, 0.0f ), true );
    
    {
        gl::ScopedMatrices pushMatrix;
        gl::setMatrices( mCam );
        gl::ScopedViewport view( vec2(0), getWindowSize() );
        
        {
            gl::ScopedModelMatrix pushModel;
            gl::rotate( mSceneRotation );
            gl::ScopedTextureBind depthTex( mDepthTexture, 0 );
            mPointCloud->draw();
        }
    }
    
    mParams->draw();
}

void prepareSettings( App::Settings* settings )
{
    settings->setWindowSize( 1280, 720 );
}

CINDER_APP( PointCloudGl, RendererGl, prepareSettings )
