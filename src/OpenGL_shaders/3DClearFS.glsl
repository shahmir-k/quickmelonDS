#version 140
#ifdef GL_ES
#define FRAGLOC(loc) layout(location = loc)
#else
#define FRAGLOC(loc)
#endif

uniform uvec4 uColor;
uniform uint uOpaquePolyID;
uniform uint uFogFlag;

FRAGLOC(0) out vec4 oColor;
FRAGLOC(1) out vec4 oAttr;

void main()
{
#ifdef GL_ES
    oColor = vec4(uColor).bgra / 31.0; // Mali: 3D layer emitted BGRA (see 3DRenderFS)
#else
    oColor = vec4(uColor).rgba / 31.0;
#endif
    oAttr.r = float(uOpaquePolyID) / 63.0;
    oAttr.g = 0.0;
    oAttr.b = float(uFogFlag);
    oAttr.a = 1.0;
}
