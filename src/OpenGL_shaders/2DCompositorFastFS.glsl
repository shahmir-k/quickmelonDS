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

// SCANLINE_RUN (debug.litev.glcomprun): the draw covers lines with identical config, which
// comes from a single-struct block - constant offsets instead of a per-pixel indexed load
// The BG offsets may advance linearly over the run (text BG Y + 1 per line, rotscale
// reference + (B, D) per line): offset(line) = base + (line - uRunStart) * uStep.
#ifdef SCANLINE_RUN
layout(std140) uniform ubLineConfig
{
    sScanline uLine;
    ivec4 uStep[4];
    int uRunStart;
};
#define SL(l) uLine
#define BGOFF(k, l) (uLine.BGOffset[k] + ((l) - uRunStart) * uStep[k].xy)
#else
#define SL(l) uScanline[l]
#define BGOFF(k, l) uScanline[l].BGOffset[k]
#endif

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
// false: no sprite in the OBJ layer has the mosaic attribute (debug.litev.glhimos), so the
// OBJ mosaic loop's last iteration always wins: its result is the OBJ pixel at the line's
// native point, fetched directly (same pixels, 2 fetches instead of 48)
uniform bool uObjMosaicAny;

smooth in vec4 fTexcoord;

#ifdef SPLIT3D_STAGE1
// LITEV_GLHI_SPLIT3D stage 1 (1x): the composite without the 3D layer, and how the 3D layer
// would enter it: oColor = (that composite, info), oOther = the other layer's colour
#ifdef GL_ES
layout(location = 0) out vec4 oColor;
layout(location = 1) out vec4 oOther;
#else
out vec4 oColor;   // locations bound by name (desktop GLSL 140)
out vec4 oOther;
#endif
#else
out vec4 oColor;
#endif
#ifdef SPLIT3D_STAGE2
uniform sampler2D SplitTex0;   // stage 1 oColor (native)
uniform sampler2D SplitTex1;   // stage 1 oOther (native)
#endif

int MosaicX = 0;

ivec3 ConvertColor(int col)
{
    ivec3 ret;
    ret.r = (col & 0x1F) << 1;
    ret.g = ((col & 0x3E0) >> 4) | (col >> 15);
    ret.b = (col & 0x7C00) >> 9;
#ifdef GL_ES
    // GLES: palettes are uploaded as 5_5_5_1 (GLES_Compat), which puts DS red in .b; the
    // rest of the GLES pipeline (and the app's .bgr present) relies on that order, so a
    // colour decoded here from a raw 15-bit value must use it too
    ret = ret.bgr;
#endif
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
    ivec2 bgoffset = BGOFF(0, line);
    vec2 bgpos = vec2(bgoffset.xy) + coord;

    if (SL(line).BGMosaicEnable[0])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    return BG0Fetch(bgpos / vec2(uBGConfig[0].Size));
}

vec4 BG1CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = BGOFF(1, line);
    vec2 bgpos = vec2(bgoffset.xy) + coord;

    if (SL(line).BGMosaicEnable[1])
    {
        bgpos = floor(bgpos) - vec2(MosaicX, 0);
    }

    return BG1Fetch(bgpos / vec2(uBGConfig[1].Size));
}

vec4 BG2CalcAndFetch(vec2 coord, int line)
{
    ivec2 bgoffset = BGOFF(2, line);
    vec2 bgpos;
    if (uBGConfig[2].Type >= 2)
    {
        // rotscale BG
        bgpos = vec2(bgoffset.xy) / 256.0;
        vec4 rotscale = vec4(SL(line).BGRotscale[0]) / 256.0;
        mat2 rsmatrix = mat2(rotscale.xy, rotscale.zw);
        bgpos = bgpos + (coord * rsmatrix);
    }
    else
    {
        // text-mode BG
        bgpos = vec2(bgoffset.xy) + coord;
    }

    if (SL(line).BGMosaicEnable[2])
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
    ivec2 bgoffset = BGOFF(3, line);
    vec2 bgpos;
    if (uBGConfig[3].Type >= 2)
    {
        // rotscale BG
        bgpos = vec2(bgoffset.xy) / 256.0;
        vec4 rotscale = vec4(SL(line).BGRotscale[1]) / 256.0;
        mat2 rsmatrix = mat2(rotscale.xy, rotscale.zw);
        bgpos = bgpos + (coord * rsmatrix);
    }
    else
    {
        // text-mode BG
        bgpos = vec2(bgoffset.xy) + coord;
    }

    if (SL(line).BGMosaicEnable[3])
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
            int mosx = texelFetch(MosaicTex, ivec2(curpos.x, SL(curpos.y).MosaicSize.z), 0).r;
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

// the final colour from the top two layers (t: layer index, -1 backdrop) and their colours
vec4 Resolve(mediump int t1, mediump int t2, vec4 c1, vec4 c2, ivec4 objflags, uint winsel, int line)
{
    mediump ivec4 back = ivec4(ConvertColor(SL(line).BackColor), 0x20);
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

#ifndef COMPDIAG
#define COMPDIAG 0   // diagnostic variants (debug.litev.glcomp=2..5), wrong pixels by design
#endif
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
#if COMPDIAG == 2
    int line = 0;
#else
    int line = int(fTexcoord.y);
#endif

    ivec4 mosaicsize = SL(line).MosaicSize;
    if (mosaicsize.x > 0)
        MosaicX = texelFetch(MosaicTex, ivec2(bgcoord.x, mosaicsize.x), 0).r;

    uint winregs = SL(line).WinRegs;
    int winmask = SL(line).WinMask;
    ivec4 winpos = SL(line).WinPos;

    vec4 lc0 = vec4(0), lc1 = vec4(0), lc2 = vec4(0), lc3 = vec4(0), lc4 = vec4(0);
#if COMPDIAG == 4
    lc0 = lc1 = lc2 = lc3 = vec4(0.5);
#elif COMPDIAG == 5
    if (uBGPrio[0] >= 0) lc0 = BG0CalcAndFetch(bgcoord, line);
#else
#ifndef SPLIT3D_STAGE1
    if (uBGPrio[0] >= 0) lc0 = BG0CalcAndFetch(bgcoord, line);
#endif
    if (uBGPrio[1] >= 0) lc1 = BG1CalcAndFetch(bgcoord, line);
    if (uBGPrio[2] >= 0) lc2 = BG2CalcAndFetch(bgcoord, line);
    if (uBGPrio[3] >= 0) lc3 = BG3CalcAndFetch(bgcoord, line);
#endif

    ivec4 objflags = ivec4(0);
#if COMPDIAG == 3
    if (false)
#else
    if (mosaicsize.z > 0 && uObjMosaicAny)
#endif
    {
        vec4 oc;
        CalcSpriteMosaic(ivec2(fTexcoord.xy), objflags, oc);
        lc4 = oc;
    }
#if COMPDIAG != 3
    else if (mosaicsize.z > 0)
    {
        ivec2 p1 = ivec2(fTexcoord.xy) * uScaleFactor;
        lc4 = texelFetch(OBJLayerTex, ivec3(p1, 0), 0);
        objflags = ivec4(texelFetch(OBJLayerTex, ivec3(p1, 1), 0) * 255.0);
    }
#endif
#if COMPDIAG != 3
    else
#else
    if (false)
#endif
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

#ifdef SPLIT3D_STAGE1
    // the others' top two (k0 is -1 here: BG0 was not fetched); where the 3D layer would go
    mediump int k0s = (uBGPrio[0] >= 0 && (winsel & 1u) != 0u) ? (3 - uBGPrio[0]) * 5 + 3 : -1;
    int mode = (k0s < 0) ? 0 : (k0s > kb1) ? 1 : (k0s > kb2) ? 2 : 0;
    int info = mode | ((t1 + 1) << 2) | (objflags.r << 5) | (((winsel >> 5) & 1u) != 0u ? 128 : 0);
    oOther = c1;
    return vec4(Resolve(t1, t2, c1, c2, objflags, winsel, line).rgb, float(info) / 255.0);
#else
    return Resolve(t1, t2, c1, c2, objflags, winsel, line);
#endif
}

#ifdef SPLIT3D_STAGE2
// stage 2 (Nx): the 3D texel of this pixel joins the stage 1 result exactly as the full
// compositor would have put it in (same Resolve, same inputs)
void main()
{
    ivec2 np = ivec2(fTexcoord.xy);
    vec4 s0 = texelFetch(SplitTex0, np, 0);
    int info = int(s0.a * 255.0 + 0.5);
    int mode = info & 3;
    if (mode != 0)
    {
        int line = int(fTexcoord.y);
        vec4 lc0 = BG0CalcAndFetch(vec2(fTexcoord.x, fract(fTexcoord.y)), line);
        if (lc0.a > 0.0)
        {
            mediump int t = ((info >> 2) & 7) - 1;
            ivec4 of = ivec4((info >> 5) & 3, 0, 0, 0);
            uint win = ((info & 128) != 0) ? 32u : 0u;
            vec4 co = texelFetch(SplitTex1, np, 0);
            oColor = (mode == 1) ? Resolve(0, t, lc0, co, of, win, line) : Resolve(t, 0, co, lc0, of, win, line);
            return;
        }
    }
    oColor = vec4(s0.rgb, 1.0);
}
#else
void main()
{
#ifdef SPLIT3D_STAGE1
    oColor = CompositeLayers();
#else
    oColor = CompositeLayers();
#endif
}
#endif
