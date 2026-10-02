#version 140

// full-screen triangle, no vertex attributes
smooth out highp vec2 fNative;   // native (1x) position: (P + 0.5) / scale at pixel centres

void main()
{
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
    fNative = p * vec2(256.0, 192.0);
}
