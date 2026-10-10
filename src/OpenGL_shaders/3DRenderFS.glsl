#version 140
#ifdef GL_ES
#define FRAGLOC(loc) layout(location = loc)
#else
#define FRAGLOC(loc)
#endif

#ifdef TexUnorm
uniform sampler2DArray CurTexture;
#else
uniform usampler2DArray CurTexture;
#endif
uniform sampler2DArray Capture128Texture;
uniform sampler2DArray Capture256Texture;

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

// MEDIUMP_HERE (debug.litev.gl3dmp: colour maths in mediump; texcoords and depth stay highp)
uniform int uRenderMode; // 0=opaque 1=translucent 2=shadowmask

smooth in vec4 fColor;
smooth in highp vec2 fTexcoord;
flat in ivec3 fPolygonAttr;

#if defined(WBuffer) && !defined(WEarlyZ)
smooth in highp float fZ;
#endif

FRAGLOC(0) out vec4 oColor;
FRAGLOC(1) out vec4 oAttr;

vec4 FinalColor()
{
    vec4 col;
    vec4 vcol = fColor;
    int blendmode = (fPolygonAttr.x >> 4) & 0x3;

    if (blendmode == 2)
    {
        if ((uDispCnt & (1<<1)) == 0)
        {
            // toon
            vec3 tooncolor = uToonColors[int(vcol.r * 31.0)].rgb;
            vcol.rgb = tooncolor;
        }
        else
        {
            // highlight
            vcol.rgb = vcol.rrr;
        }
    }

    if (fPolygonAttr.y == 0xFFFF)
    {
        // no texture
        col = vcol;
    }
    else
    {
        highp vec3 texcoord = vec3(fTexcoord, fPolygonAttr.y);
        vec4 tcol;
        if (fPolygonAttr.z == 0)
#ifdef TexUnorm
            tcol = texture(CurTexture, texcoord);
#else
            tcol = vec4(texture(CurTexture, texcoord)) / vec4(63,63,63,31);
#endif
        else if (fPolygonAttr.z == 1)
            tcol = texture(Capture128Texture, texcoord);
        else
            tcol = texture(Capture256Texture, texcoord);

        if ((blendmode & 1) != 0)
        {
            // decal
            col.rgb = (tcol.rgb * tcol.a) + (vcol.rgb * (1.0-tcol.a));
            col.a = vcol.a;
        }
        else
        {
            // modulate
            col = vcol * tcol;
        }
    }

    if (blendmode == 2)
    {
        if ((uDispCnt & (1<<1)) != 0)
        {
            vec3 tooncolor = uToonColors[int(vcol.r * 31.0)].rgb;
            col.rgb = min(col.rgb + tooncolor, 1.0);
        }
    }

#ifdef GL_ES
    // Mali/Android GLES: the 3D layer reaches the display with R/B swapped
    // relative to the (correct) 2D compositor output, so the whole 3D layer must
    // be emitted in BGRA (matching the v1 GLES renderer). Desktop keeps RGBA.
    return col.bgra;
#else
    return col.rgba;
#endif
}

void main()
{
#ifdef AlphaTestOnly
    // LITEV_GL_ALPHATEST_2PASS pass A: FinalColor's alpha only (polygon alpha is 31 here)
    {
        float a = fColor.a;
        if (fPolygonAttr.y != 0xFFFF && ((fPolygonAttr.x >> 4) & 0x3) != 1)
        {
            highp vec3 texcoord = vec3(fTexcoord, fPolygonAttr.y);
            if (fPolygonAttr.z == 0)
#ifdef TexUnorm
                a *= texture(CurTexture, texcoord).a;
#else
                a *= float(texture(CurTexture, texcoord).a) / 31.0;
#endif
            else if (fPolygonAttr.z == 1)
                a *= texture(Capture128Texture, texcoord).a;
            else
                a *= texture(Capture256Texture, texcoord).a;
        }
        if (a < 30.5/31.0) discard;
        oColor = vec4(0.0);
        oAttr = vec4(0.0);
        return;
    }
#endif
#ifdef NoDiscard
    // opaque pass, polygon that can't produce a transparent pixel: no alpha test, so the Mali
    // keeps its early depth test and hidden-surface removal (any discard in the shader loses both)
    oColor = FinalColor();
    oAttr = vec4(float((fPolygonAttr.x >> 24) & 0x3F) / 63.0, max(gl_FragCoord.z, 1.0/255.0), float((fPolygonAttr.x >> 15) & 0x1), 1.0);
#else
    if (uRenderMode == 2)
    {
        oColor = vec4(0,0,0,1);
    }
    else
    {
        vec4 col = FinalColor();
        if (uRenderMode == 0 || uRenderMode == 3)
        {
            // opaque pixels (3: translucent texels too, blended by the caller)
            if (col.a < (uRenderMode == 0 ? 30.5/31.0 : 0.5/31.0)) discard;

            oAttr.r = float((fPolygonAttr.x >> 24) & 0x3F) / 63.0;
            oAttr.g = max(gl_FragCoord.z, 1.0/255.0);   // opaque pixel: 8-bit depth for edge marking (0 = none)
            oAttr.b = float((fPolygonAttr.x >> 15) & 0x1);
            // 3: a translucent texel keeps the attributes under it (the pass blends with SRC_ALPHA,
            // attachment 1 too): its polygon ID / edge flag outlined whole shadow decals
            oAttr.a = (uRenderMode == 3 && col.a < 30.5/31.0) ? 0.0 : 1.0;
        }
        else
        {
            // translucent pixels
            if (col.a < 0.5/31.0) discard;
            if (col.a >= 30.5/31.0) discard;

            oAttr.b = 0.0;
            oAttr.a = 1.0;
        }

        oColor = col;
    }
#endif

#if defined(WBuffer) && !defined(WEarlyZ)
    // depth-equal polygons: the DS's +-0xFF W-buffer margin (see 3DRenderVS)
    gl_FragDepth = ((fPolygonAttr.x & 0x4000) != 0) ? fZ - 255.0 / 16777216.0 : fZ;
#endif
}
