#version 140

uniform sampler2D BGLayerTex[4];
uniform sampler2DArray OBJLayerTex;
uniform sampler2DArray Capture128Tex;
uniform sampler2DArray Capture256Tex;
uniform isampler2D MosaicTex;

struct sBGConfig
{
    ivec2 Size;
    int Type;
    int PalOffset;
    int TileOffset;
    int MapOffset;
    bool Clamp;
};

layout(std140) uniform ubBGConfig
{
    int uVRAMMask;
    sBGConfig uBGConfig[4];
};

struct sScanline
{
    ivec2 BGOffset[4];
    ivec4 BGRotscale[2];
    int BackColor;
    uint WinRegs;
    int WinMask;
    ivec4 WinPos;
    bvec4 BGMosaicEnable;
    ivec4 MosaicSize;
};

layout(std140) uniform ubScanlineConfig
{
    sScanline uScanline[192];
};

layout(std140) uniform ubCompositorConfig
{
    ivec4 uBGPrio;
    bool uEnableOBJ;
    bool uEnable3D;
    int uBlendCnt;
    int uBlendEffect;
    ivec3 uBlendCoef;
};

uniform int uScaleFactor;

smooth in vec4 fTexcoord;

out vec4 oColor;

int MosaicX = 0;

ivec3 ConvertColor(int col)
{
    ivec3 ret;
    ret.r = (col & 0x1F) << 1;
    ret.g = ((col & 0x3E0) >> 4) | (col >> 15);
    ret.b = (col & 0x7C00) >> 9;
    return ret;
}

vec4 BG0Fetch(vec2 coord)
{
    return texture(BGLayerTex[0], coord);
}

vec4 BG1Fetch(vec2 coord)
{
    return texture(BGLayerTex[1], coord);
}

vec4 BG2Fetch(vec2 coord)
{
    return texture(BGLayerTex[2], coord);
}

vec4 BG3Fetch(vec2 coord)
{
    return texture(BGLayerTex[3], coord);
}

vec4 BG0CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = uScanline[line].BGOffset[0];
    vec2 bgpos = vec2(bgoffset.xy) + coord;

    if (uScanline[line].BGMosaicEnable[0])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    return BG0Fetch(bgpos / vec2(uBGConfig[0].Size));
}

vec4 BG1CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = uScanline[line].BGOffset[1];
    vec2 bgpos = vec2(bgoffset.xy) + coord;

    if (uScanline[line].BGMosaicEnable[1])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    return BG1Fetch(bgpos / vec2(uBGConfig[1].Size));
}

vec4 BG2CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = uScanline[line].BGOffset[2];
    vec2 bgpos;
    if (uBGConfig[2].Type >= 2)
    {
        // rotscale BG
        bgpos = vec2(bgoffset.xy) / 256.0;
        vec4 rotscale = vec4(uScanline[line].BGRotscale[0]) / 256.0;
        mat2 rsmatrix = mat2(rotscale.xy, rotscale.zw);
        bgpos = bgpos + (coord * rsmatrix);
    }
    else
    {
        // text-mode BG
        bgpos = vec2(bgoffset.xy) + coord;
    }

    if (uScanline[line].BGMosaicEnable[2])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    if (uBGConfig[2].Type >= 7)
    {
        // hi-res capture
        bgpos.y += float(uBGConfig[2].MapOffset);
        vec3 capcoord = vec3(bgpos / vec2(uBGConfig[2].Size), uBGConfig[2].TileOffset);

        // due to the possible weirdness of display capture buffers,
        // we need to do custom wraparound handling
        if (uBGConfig[2].Clamp)
        {
            if (any(lessThan(capcoord.xy, vec2(0))) || any(greaterThanEqual(capcoord.xy, vec2(1))))
                return vec4(0);
        }

        if (uBGConfig[2].Type == 7)
            return texture(Capture128Tex, capcoord);
        else
            return texture(Capture256Tex, capcoord);
    }

    return BG2Fetch(bgpos / vec2(uBGConfig[2].Size));
}

vec4 BG3CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = uScanline[line].BGOffset[3];
    vec2 bgpos;
    if (uBGConfig[3].Type >= 2)
    {
        // rotscale BG
        bgpos = vec2(bgoffset.xy) / 256.0;
        vec4 rotscale = vec4(uScanline[line].BGRotscale[1]) / 256.0;
        mat2 rsmatrix = mat2(rotscale.xy, rotscale.zw);
        bgpos = bgpos + (coord * rsmatrix);
    }
    else
    {
        // text-mode BG
        bgpos = vec2(bgoffset.xy) + coord;
    }

    if (uScanline[line].BGMosaicEnable[3])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    if (uBGConfig[3].Type >= 7)
    {
        // hi-res capture
        bgpos.y += float(uBGConfig[3].MapOffset);
        vec3 capcoord = vec3(bgpos / vec2(uBGConfig[3].Size), uBGConfig[3].TileOffset);

        // due to the possible weirdness of display capture buffers,
        // we need to do custom wraparound handling
        if (uBGConfig[3].Clamp)
        {
            if (any(lessThan(capcoord.xy, vec2(0))) || any(greaterThanEqual(capcoord.xy, vec2(1))))
                return vec4(0);
        }

        if (uBGConfig[3].Type == 7)
            return texture(Capture128Tex, capcoord);
        else
            return texture(Capture256Tex, capcoord);
    }

    return BG3Fetch(bgpos / vec2(uBGConfig[3].Size));
}

void CalcSpriteMosaic(in ivec2 coord, out ivec4 objflags, out vec4 objcolor)
{
    for (int i = 0; i < 16; i++)
    {
        ivec2 curpos = ivec2(coord.x - 15 + i, coord.y);

        if (curpos.x < 0)
        {
            objflags = ivec4(0);
            objcolor = vec4(0);
        }
        else
        {
            int mosx = texelFetch(MosaicTex, ivec2(curpos.x, uScanline[curpos.y].MosaicSize.z), 0).r;
            vec4 color = texelFetch(OBJLayerTex, ivec3(curpos * uScaleFactor, 0), 0);
            ivec4 flags = ivec4(texelFetch(OBJLayerTex, ivec3(curpos * uScaleFactor, 1), 0) * 255.0);

            bool latch = false;
            if (mosx == 0)
                latch = true;
            else if (flags.g == 0)
                latch = true;
            else if (objflags.g == 0)
                latch = true;
            else if (flags.a < objflags.a)
                latch = true;

            if (latch)
            {
                objflags = flags;
                objcolor = color;
            }
        }
    }
}

// Fast variant (debug.litev.glcomp, default on; 0 = 2DCompositorFS.glsl). Same result:
// - disabled BGs are not fetched (uniform branches)
// - the 4x5 priority loop becomes a top-two selection over per-layer draw-order keys
//   (key = the iteration at which the original loop would have written the layer; the
//   last write wins, so the top layer is the max key and the second the next one)
// - blend maths in mediump ints (values stay below 2^12)
vec4 CompositeLayers()
{
    ivec2 coord = ivec2(fTexcoord.zw);
    vec2 bgcoord = vec2(fTexcoord.x, fract(fTexcoord.y));
    int xpos = int(fTexcoord.x);
    int line = int(fTexcoord.y);

    ivec4 mosaicsize = uScanline[line].MosaicSize;
    if (mosaicsize.x > 0)
        MosaicX = texelFetch(MosaicTex, ivec2(bgcoord.x, mosaicsize.x), 0).r;

    uint winregs = uScanline[line].WinRegs;
    int winmask = uScanline[line].WinMask;
    ivec4 winpos = uScanline[line].WinPos;

    vec4 lc0 = vec4(0), lc1 = vec4(0), lc2 = vec4(0), lc3 = vec4(0), lc4 = vec4(0);
    if (uBGPrio[0] >= 0) lc0 = BG0CalcAndFetch(bgcoord, line);
    if (uBGPrio[1] >= 0) lc1 = BG1CalcAndFetch(bgcoord, line);
    if (uBGPrio[2] >= 0) lc2 = BG2CalcAndFetch(bgcoord, line);
    if (uBGPrio[3] >= 0) lc3 = BG3CalcAndFetch(bgcoord, line);

    ivec4 objflags = ivec4(0);
    if (mosaicsize.z > 0)
    {
        vec4 oc;
        CalcSpriteMosaic(ivec2(fTexcoord.xy), objflags, oc);
        lc4 = oc;
    }
    else
    {
        lc4 = texelFetch(OBJLayerTex, ivec3(coord, 0), 0);
        objflags = ivec4(texelFetch(OBJLayerTex, ivec3(coord, 1), 0) * 255.0);
    }

    bool inside_win0, inside_win1;

    if (xpos < winpos[0])
        inside_win0 = ((winmask & (1<<0)) != 0);
    else if (xpos < winpos[1])
        inside_win0 = ((winmask & (1<<1)) != 0);
    else
        inside_win0 = ((winmask & (1<<2)) != 0);

    if (xpos < winpos[2])
        inside_win1 = ((winmask & (1<<3)) != 0);
    else if (xpos < winpos[3])
        inside_win1 = ((winmask & (1<<4)) != 0);
    else
        inside_win1 = ((winmask & (1<<5)) != 0);

    uint winsel = winregs;
    if (objflags.b > 0)
        winsel = winregs >> 8;
    if (inside_win1)
        winsel = winregs >> 16;
    if (inside_win0)
        winsel = winregs >> 24;

    // draw-order keys (-1: not drawn). Original order: prio 3..0, within a prio BG3..BG0 then OBJ.
    mediump int k0 = (uBGPrio[0] >= 0 && lc0.a > 0.0 && (winsel & 1u) != 0u) ? (3 - uBGPrio[0]) * 5 + 3 : -1;
    mediump int k1 = (uBGPrio[1] >= 0 && lc1.a > 0.0 && (winsel & 2u) != 0u) ? (3 - uBGPrio[1]) * 5 + 2 : -1;
    mediump int k2 = (uBGPrio[2] >= 0 && lc2.a > 0.0 && (winsel & 4u) != 0u) ? (3 - uBGPrio[2]) * 5 + 1 : -1;
    mediump int k3 = (uBGPrio[3] >= 0 && lc3.a > 0.0 && (winsel & 8u) != 0u) ? (3 - uBGPrio[3]) * 5 + 0 : -1;
    mediump int k4 = (uEnableOBJ && lc4.a > 0.0 && (winsel & 16u) != 0u) ? (3 - objflags.a) * 5 + 4 : -1;

    // top two by key; the backdrop (key -1, mask 0x20) sits under everything
    mediump int t1 = -1, t2 = -1;         // layer index of top / second (-1 backdrop, -2 none)
    mediump int kb1 = -1, kb2 = -2;
    vec4 c1 = vec4(0), c2 = vec4(0);
#define TOP2(k, i, c) \
    if (k > kb1) { kb2 = kb1; t2 = t1; c2 = c1; kb1 = k; t1 = i; c1 = c; } \
    else if (k > kb2) { kb2 = k; t2 = i; c2 = c; }
    if (k0 >= 0) { TOP2(k0, 0, lc0) }
    if (k1 >= 0) { TOP2(k1, 1, lc1) }
    if (k2 >= 0) { TOP2(k2, 2, lc2) }
    if (k3 >= 0) { TOP2(k3, 3, lc3) }
    if (k4 >= 0) { TOP2(k4, 4, lc4) }
#undef TOP2

    mediump ivec4 back = ivec4(ConvertColor(uScanline[line].BackColor), 0x20);
    mediump ivec4 col1, col2;
    mediump int mask1, mask2;
    if (t1 < 0)
    {
        col1 = back; mask1 = 0x20;
        col2 = ivec4(0); mask2 = 0;
    }
    else
    {
        col1 = ivec4(c1 * 255.0) >> ivec4(2,2,2,3);
        mask1 = 1 << t1;
        if (t2 < 0) { col2 = back; mask2 = 0x20 << 8; }
        else { col2 = ivec4(c2 * 255.0) >> ivec4(2,2,2,3); mask2 = (1 << t2) << 8; }
    }
    bool specialcase = (t1 == 0) ? uEnable3D : ((t1 == 4) ? (objflags.r != 0) : false);

    int effect = 0;
    mediump int eva, evb, evy = uBlendCoef[2];

    if (specialcase && (uBlendCnt & mask2) != 0)
    {
        if (mask1 == (1<<0))
        {
            // 3D layer blending
            effect = 4;
            eva = (col1.a & 0x1F) + 1;
            evb = 32 - eva;
        }
        else if (objflags.r == 1)
        {
            // semi-transparent sprite
            effect = 1;
            eva = uBlendCoef[0];
            evb = uBlendCoef[1];
        }
        else //if (objflags.r == 2)
        {
            // bitmap sprite
            effect = 1;
            eva = col1.a;
            evb = 16 - eva;
        }
    }
    else if (((uBlendCnt & mask1) != 0) && ((winsel & (1u << 5)) != 0u))
    {
        effect = uBlendEffect;
        if (effect == 1)
        {
            if ((uBlendCnt & mask2) != 0)
            {
                eva = uBlendCoef[0];
                evb = uBlendCoef[1];
            }
            else
                effect = 0;
        }
    }

    if (effect == 1)
    {
        col1 = ((col1 * eva) + (col2 * evb) + 0x8) >> 4;
        col1 = min(col1, 0x3F);
    }
    else if (effect == 2)
    {
        col1 = col1 + ((((0x3F - col1) * evy) + 0x8) >> 4);
    }
    else if (effect == 3)
    {
        col1 = col1 - (((col1 * evy) + 0x7) >> 4);
    }
    else if (effect == 4)
    {
        col1 = ((col1 * eva) + (col2 * evb) + 0x10) >> 5;
    }

    return vec4(vec3(col1.rgb << 2) / 255.0, 1);
}

void main()
{
    oColor = CompositeLayers();
}
