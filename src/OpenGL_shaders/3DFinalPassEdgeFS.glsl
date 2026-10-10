#version 140

uniform sampler2D DepthBuffer;
uniform sampler2D AttrBuffer;

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

out vec4 oColor;

// DS edge marking: an opaque pixel (attr.g, set by 3DRenderFS) is marked when a neighbour one DS
// pixel away (scale internal pixels) has another polygon ID and is farther (the clear plane
// counts: it carries the clear polygon ID and depth). The DS also requires the pixel to lie on
// its polygon's rasterized edge; skipping that only differs where polygons interpenetrate.
// ponytail: neighbours past the screen border clamp to the pixel itself (no mark there); the DS
// compares them against the clear values.
bool farther(ivec2 c, int refPolyID, float refDepth)
{
    c = clamp(c, ivec2(0), ivec2(uScreenSize) - 1);
    int polyid = int(texelFetch(AttrBuffer, c, 0).r * 63.0 + 0.5);
    return polyid != refPolyID && refDepth < texelFetch(DepthBuffer, c, 0).r;
}

void main()
{
    ivec2 coord = ivec2(gl_FragCoord.xy);
    vec4 attr = texelFetch(AttrBuffer, coord, 0);
    if (attr.g == 0.0) discard;

    int scale = max(int(uScreenSize.x) / 256, 1);
    int polyid = int(attr.r * 63.0 + 0.5);
    float depth = texelFetch(DepthBuffer, coord, 0).r;

    if (!(farther(coord + ivec2(0,-scale), polyid, depth) ||
          farther(coord + ivec2(0, scale), polyid, depth) ||
          farther(coord + ivec2(-scale,0), polyid, depth) ||
          farther(coord + ivec2( scale,0), polyid, depth)))
        discard;

#ifdef GL_ES
    oColor.rgb = uEdgeColors[polyid >> 3].bgr;   // the GLES 3D layer is BGRA (3DRenderFS)
#else
    oColor.rgb = uEdgeColors[polyid >> 3].rgb;
#endif
    // this isn't quite accurate (antialiasing), but it will have to do
    oColor.a = ((uDispCnt & (1<<4)) != 0) ? 0.5 : 1.0;
}
