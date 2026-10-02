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
uniform int uScale;
uniform int uSingle;     // -1: both screens (MRT); 0/1: only that screen, to output 0
uniform ivec2 uOrigin;   // viewport origin of this draw in the target

FRAGLOC(0) out vec4 oTopColor;
FRAGLOC(1) out vec4 oBottomColor;

ivec4 Desc(int x, int y, int layer)
{
    return ivec4(texelFetch(DescTex, ivec3(x, y, layer), 0));
}

ivec4 Get3D(ivec2 pos)
{
    if (pos.x < 0 || pos.x >= 256 * uScale) return ivec4(0);
#ifdef GL_ES
    vec4 c = texelFetch(Tex3D, pos, 0).bgra;   // the GLES 3D pass emits BGRA (3DRenderFS)
#else
    vec4 c = texelFetch(Tex3D, pos, 0);
#endif
    return ivec4(round(c * vec4(63.0, 63.0, 63.0, 31.0)));
}

vec4 Screen(int layer, ivec2 P)
{
    ivec2 n = P / uScale;
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
