#version 150

in vec4 ciPosition;
in vec2 ciTexCoord0;

uniform usampler2D  uDepthTexture;
uniform float       ref_pix_size;
uniform float       ref_distance;
uniform float       const_shift;
uniform float       dcmos_emitter_dist;

uniform mat4        ciModelViewProjection;

out     float       vDepthColor;

const float parameter_coefficient = 4;
const float shift_scale = 10;
const float pixel_size_factor = 1;
#define S2D_CONST_OFFSET 0.375

/// convert raw shift value to metric depth (in mm)
float raw_to_mm(float raw)
{
    float fixed_ref_x = ( ( raw - (parameter_coefficient * const_shift / pixel_size_factor ) ) / parameter_coefficient ) - S2D_CONST_OFFSET;
    float metric = fixed_ref_x * ref_pix_size * pixel_size_factor;
    return shift_scale * ( ( metric * ref_distance / ( dcmos_emitter_dist - metric ) ) + ref_distance );
}

///depth to world conversion
vec3 depth_mm_to_world( float cam_x, float cam_y, float depth_mm ){
    
    vec3  world;
    float factor    = 2. * ref_pix_size * depth_mm / ref_distance;
    world.x         = ( cam_x - 320. ) * factor;
    world.y         = ( cam_y - 240. ) * factor;
    world.z         = depth_mm;
    return          world;
    
}

void main()
{
    vec4 pos		= ciPosition;
    uint rawdepth	= texture( uDepthTexture, ciTexCoord0 ).r;
    vDepthColor     = float(rawdepth) / 65535.; //divide by max short bytes for % grey
    pos.xyz         = depth_mm_to_world( ciTexCoord0.x*640., (1.-ciTexCoord0.y)*480., raw_to_mm( float(rawdepth) ) );
	gl_Position		= ciModelViewProjection * pos;
}
