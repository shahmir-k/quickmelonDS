#version 140

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

// DS edge marking at 1x: one fragment per DS pixel of the 256x192 edge overlay (the hybrid merge
// lays it over the Nx 3D colour). A pixel is marked when its opaque polygon has a neighbour one DS
// pixel away with another polygon ID that is farther. Reads only the attribute buffer, at the
// centre of each DS pixel: R = opaque polygon ID, G = 8-bit window depth of an opaque pixel, 0
// where there is none (the clear plane: farthest). The DS also requires the pixel to lie on its
// polygon's rasterized edge; skipping that only differs where polygons interpenetrate.
// ponytail: 8-bit depth (equal-depth neighbours mark neither side), neighbours past the screen
// border clamp to the pixel itself; 16-bit depth / clear-value borders if it shows.
int scale;
bool farther(ivec2 c, int refPolyID, float refDepth)
{
    vec4 a = texelFetch(AttrBuffer, clamp(c, ivec2(0), ivec2(uScreenSize) - 1), 0);
    return int(a.r * 63.0 + 0.5) != refPolyID && (a.g == 0.0 || refDepth < a.g);
}

void main()
{
    scale = max(int(uScreenSize.x) / 256, 1);
    ivec2 coord = ivec2(gl_FragCoord.xy) * scale + scale / 2;
    vec4 attr = texelFetch(AttrBuffer, coord, 0);
    int polyid = int(attr.r * 63.0 + 0.5);

    if (attr.g == 0.0 ||
        !(farther(coord + ivec2(0,-scale), polyid, attr.g) ||
          farther(coord + ivec2(0, scale), polyid, attr.g) ||
          farther(coord + ivec2(-scale,0), polyid, attr.g) ||
          farther(coord + ivec2( scale,0), polyid, attr.g)))
    {
        oColor = vec4(0.0);
        return;
    }

#ifdef GL_ES
    oColor.rgb = uEdgeColors[polyid >> 3].bgr;   // in the 3D layer's layout: BGRA on GLES (3DRenderFS)
#else
    oColor.rgb = uEdgeColors[polyid >> 3].rgb;
#endif
    // coverage: this isn't quite accurate (antialiasing), but it will have to do
    oColor.a = ((uDispCnt & (1<<4)) != 0) ? 0.5 : 1.0;
}
