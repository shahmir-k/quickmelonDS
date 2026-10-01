#version 140
#ifdef GL_ES
#define FRAGLOC(loc) layout(location = loc)
#else
#define FRAGLOC(loc)
#endif

uniform usampler2DArray CurTexture;
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

uniform int uRenderMode; // 0=opaque 1=translucent 2=shadowmask

smooth in vec4 fColor;
smooth in vec2 fTexcoord;
flat in ivec3 fPolygonAttr;

#ifdef WBuffer
smooth in float fZ;
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
        vec3 texcoord = vec3(fTexcoord, fPolygonAttr.y);
        vec4 tcol;
        if (fPolygonAttr.z == 0)
            tcol = vec4(texture(CurTexture, texcoord)) / vec4(63,63,63,31);
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
    if (uRenderMode == 2)
    {
        oColor = vec4(0,0,0,1);
    }
    else
    {
        vec4 col = FinalColor();
        if (uRenderMode == 0)
        {
            // opaque pixels
            if (col.a < 30.5/31.0) discard;

            oAttr.r = float((fPolygonAttr.x >> 24) & 0x3F) / 63.0;
            oAttr.g = 0.0;
            oAttr.b = float((fPolygonAttr.x >> 15) & 0x1);
            oAttr.a = 1.0;
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

#ifdef WBuffer
    gl_FragDepth = fZ;
#endif
}
