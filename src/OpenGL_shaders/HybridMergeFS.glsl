#version 140
#ifdef GL_ES
#define FRAGLOC(loc) layout(location = loc)
#else
#define FRAGLOC(loc)
#endif

// Hybrid renderer merge + final pass: per Nx output pixel, fetch the native-resolution
// 2D descriptor written by the CPU (SoftRenderer2D::HybridDesc), the Nx 3D pixel, and
// apply the merge recipe, then master brightness and the 6->8 bit expansion. Must match
// SoftRenderer2D::HybridResolvePixel and SoftRenderer::ApplyMasterBrightness exactly.
// Port of classic melonDS kCompositorFS_Nearest (removed upstream in ba317e2e).

uniform usampler2DArray DescTex;   // 513x192x2 RGBA8UI: 2 planes x 256 + control column
uniform sampler2D Tex3D;           // Nx 3D colour buffer
uniform sampler2D EdgeTex;         // 256x192 edge-marking overlay of that 3D frame (3DFinalPassEdgeFS)
uniform int uEdge;                 // 1: the frame has one
uniform int uScale;
uniform int uSingle;     // -1: both screens (MRT); 0/1: only that screen, to output 0
uniform ivec2 uOrigin;   // viewport origin of this draw in the target
smooth in highp vec2 fNative;
// LITEV_HYB_MERGE_FASTLINES: the lines drawn now show the 3D straight through (no 2D blend or
// brightness, 3D x offset 0: checked by the CPU), so a pixel is just its 3D colour, or the 2D
// layer under it where the 3D is empty
uniform int uFast;       // 1 + bright mode (0 none, 1 up, 2 down) for these lines; 0 = full merge
uniform int uFastEvy;

FRAGLOC(0) out vec4 oTopColor;
FRAGLOC(1) out vec4 oBottomColor;

ivec4 Desc(int x, int y, int layer)
{
    return ivec4(texelFetch(DescTex, ivec3(x, y, layer), 0));
}

ivec4 Get3D(ivec2 pos)
{
    if (pos.x < 0 || pos.x >= 256 * uScale) return ivec4(0);
    vec4 c = texelFetch(Tex3D, pos, 0);
    if (uEdge != 0)
    {
        highp vec2 p = vec2(pos) + 0.5;
        vec4 e = texelFetch(EdgeTex, ivec2(p / float(uScale)), 0);
        c.rgb = mix(c.rgb, e.rgb, e.a);
    }
#ifdef GL_ES
    c = c.bgra;   // the GLES 3D pass emits BGRA (3DRenderFS)
#endif
    return ivec4(round(c * vec4(63.0, 63.0, 63.0, 31.0)));
}

vec4 Screen(int layer, ivec2 P)
{
#ifdef NATIVE_VARYING
    // debug.litev.hybdiv (default on): no per-pixel integer divide (emulated on Mali)
    ivec2 n = ivec2(fNative);
#else
    ivec2 n = P / uScale;
#endif
    if (uFast != 0)
    {
#ifdef FAST_COPY
        // LITEV_HYB_MERGE_FASTCOPY: no brightness, no edge overlay: the 3D texel as it is (no
        // round trip through the 6-bit DS colour: within 2/255 of the full path)
        if (uFast == 1 && uEdge == 0)
        {
            vec4 t = texelFetch(Tex3D, P, 0);
            if (t.a >= 0.5 / 31.0)
#ifdef GL_ES
                return vec4(t.rgb, 1.0);   // the 3D is BGRA and the output .bgr: they cancel
#else
                return vec4(t.bgr, 1.0);
#endif
            ivec4 d = Desc(n.x + 256, n.y, layer);
            ivec3 cd = (d.rgb << 2) | (d.rgb >> 4);
            return vec4(vec3(cd.bgr) / 255.0, 1.0);
        }
#endif
        ivec4 c3 = Get3D(P);
        ivec4 px = c3.a == 0 ? Desc(n.x + 256, n.y, layer) : c3;
        if (uFast == 2)      px += ((0x3F - px) * uFastEvy) >> 4;
        else if (uFast == 3) px -= ((px * uFastEvy) + 0xF) >> 4;
        ivec3 cf = (px.rgb << 2) | (px.rgb >> 4);
        return vec4(vec3(cf.bgr) / 255.0, 1.0);
    }
    ivec4 ctl = Desc(512, n.y, layer);
    int dispmode = ctl.b & 0x3;
    ivec4 pix = Desc(n.x, n.y, layer);

    if (dispmode == 1 && (ctl.b & 0x4) != 0)   // the line carries 3D descriptors
    {
        int mode = pix.a >> 5;
        int ev1 = pix.a & 0x1F;
        if (mode != 7)
        {
            int xpos = ctl.a | ((ctl.b & 0x80) << 1);
            if ((xpos & 0x100) != 0) xpos -= 512;
            ivec4 c3 = Get3D(ivec2(P.x + xpos * uScale, P.y));

            if (c3.a == 0)
                pix = Desc(n.x + 256, n.y, layer);
            else if (mode == 1)
                pix = min((pix * ev1 + c3 * (Desc(n.x + 256, n.y, layer).a & 0x1F) + 0x8) >> 4, ivec4(0x3F));
            else if (mode == 4)
            {
                int eva = c3.a + 1;
                if (eva < 32)
                    pix = min((c3 * eva + pix * (32 - eva) + 0x10) >> 5, ivec4(0x3F));
                else
                    pix = c3;
            }
            else
            {
                pix = c3;
                if (mode == 2)      pix += (((0x3F - pix) * ev1) + 0x8) >> 4;
                else if (mode == 3) pix -= ((pix * ev1) + 0x7) >> 4;
            }
        }
    }

    if (dispmode != 0)
    {
        int brightmode = ctl.g >> 6;
        int evy = min(ctl.r & 0x1F, 16);
        if (brightmode == 1)      pix += ((0x3F - pix) * evy) >> 4;
        else if (brightmode == 2) pix -= ((pix * evy) + 0xF) >> 4;
    }

    ivec3 c = (pix.rgb << 2) | (pix.rgb >> 4);
    // the app presents the frame texture with a .bgr swizzle (software RAM frames are BGRA)
    return vec4(vec3(c.bgr) / 255.0, 1.0);
}

void main()
{
    ivec2 P = ivec2(gl_FragCoord.xy) - uOrigin;
    if (uSingle < 0)
    {
        oTopColor = Screen(0, P);
        oBottomColor = Screen(1, P);
    }
    else
    {
        oTopColor = Screen(uSingle, P);
        oBottomColor = vec4(0.0);
    }
}
