#version 140
#extension GL_EXT_shader_framebuffer_fetch : require
#extension GL_ARM_shader_framebuffer_fetch_depth_stencil : require

// 3DFinalPassFogFS reading depth and the attribute buffer from the tile (framebuffer fetch)
// instead of sampling the attachments as textures, so the 3D pass is not split and depth /
// attributes never have to be written out (debug.litev.gl3dtile).

layout(std140) uniform uConfig
{
    vec2 uScreenSize;
    int uDispCnt;
    vec4 uToonColors[32];
    vec4 uEdgeColors[8];
    vec4 uFogColor;
    float uFogDensity[34];
    int uFogOffset;
    int uFogShift;
};

layout(location = 0) out vec4 oColor;
layout(location = 1) inout vec4 oAttr;

uniform float uWZ0;

vec4 CalculateFog(float depth)
{
    // W-buffer early-Z frames store 1 - z0/w (GPU3D_OpenGL WZ0): back to w
    int idepth = uWZ0 > 0.0 ? int(min(uWZ0 / max(1.0 - depth, 1.0 / 16777216.0), 16777215.0))
                            : int(depth * 16777216.0);
    int densityid, densityfrac;

    if (idepth < uFogOffset)
    {
        densityid = 0;
        densityfrac = 0;
    }
    else
    {
        uint udepth = uint(idepth);
        udepth -= uint(uFogOffset);
        udepth = (udepth >> 2) << uint(uFogShift);

        densityid = int(udepth >> 17);
        if (densityid >= 32)
        {
            densityid = 32;
            densityfrac = 0;
        }
        else
        densityfrac = int(udepth & uint(0x1FFFF));
    }

    float density = mix(uFogDensity[densityid], uFogDensity[densityid+1], float(densityfrac)/131072.0);

    return vec4(density, density, density, density);
}

void main()
{
    vec4 ret = vec4(0,0,0,0);
    if (oAttr.b != 0.0) ret = CalculateFog(gl_LastFragDepthARM);
    oColor = ret;
}
