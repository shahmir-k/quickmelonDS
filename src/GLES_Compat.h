/*
    liteDS-v2-android GLES3 compatibility shim.

    The upstream melonDS OpenGL renderer targets desktop OpenGL (GLSL 140 /
    glad). On Android the frontend builds the core against GLES3 headers
    (MELONDS_GL_HEADER=<GLES3/gl32.h>). A handful of desktop-only entry points
    and enums are missing from GLES3; this header provides equivalent inline
    shims so the renderer source compiles unchanged.

    Included by PlatformOGL.h AFTER the real GL header, only when building for
    Android. Desktop builds never see this file, so their behaviour is intact.

    NOTE (colour order): GLES3 has no GL_UNSIGNED_SHORT_1_5_5_5_REV. We alias it
    to GL_UNSIGNED_SHORT_5_5_5_1 (fine for null-data allocations) and interpose
    the glTexImage/glTexSubImage entry points to convert REAL u16 texel uploads
    (DS palettes / VRAM captures) from the 1555_REV bit layout to 5_5_5_1 on the
    fly, preserving exact component values.
*/
#ifndef MELONDS_GLES_COMPAT_H
#define MELONDS_GLES_COMPAT_H

#if defined(__ANDROID__)

#include <cstdint>

// --- Enum aliases -----------------------------------------------------------
#ifndef GL_UNSIGNED_SHORT_1_5_5_5_REV
#define GL_UNSIGNED_SHORT_1_5_5_5_REV GL_UNSIGNED_SHORT_5_5_5_1
#endif

// --- 1555_REV upload conversion ----------------------------------------------
// Uploads of real u16 texel data in DS layout (A:1 B:5 G:5 R:5 from the MSB,
// R in the LOW bits) would be misread with 5_5_5_1 component boundaries (R in
// the HIGH bits). Convert such uploads to the 5_5_5_1 bit layout on the fly
// (palettes and occasional capture syncs; small buffers).
#include <vector>
#include <cstddef>

static inline unsigned short melonGLConv1555(unsigned short v)
{
    // 1555_REV -> 5551 keeping component order is a rotate-left-by-1: the
    // alpha bit moves from MSB to LSB and each 5-bit component shifts up one
    // bit into its 5551 position (verified on-device).
    return (unsigned short)(((v & 0x7FFFu) << 1) | (v >> 15));
}

static inline const void* melonGLConvertIf1555(GLenum type, size_t count, const void* data)
{
    if (type != GL_UNSIGNED_SHORT_5_5_5_1 || data == nullptr)
        return data;
    static thread_local std::vector<unsigned short> scratch;
    scratch.resize(count);
    const unsigned short* src = (const unsigned short*)data;
    for (size_t i = 0; i < count; i++)
        scratch[i] = melonGLConv1555(src[i]);
    return scratch.data();
}

static inline void melonGLTexImage2D(GLenum target, GLint level, GLint internalformat,
    GLsizei w, GLsizei h, GLint border, GLenum format, GLenum type, const void* data)
{
    glTexImage2D(target, level, internalformat, w, h, border, format, type,
                 melonGLConvertIf1555(type, (size_t)w * (size_t)h, data));
}
static inline void melonGLTexSubImage2D(GLenum target, GLint level, GLint xo, GLint yo,
    GLsizei w, GLsizei h, GLenum format, GLenum type, const void* data)
{
    glTexSubImage2D(target, level, xo, yo, w, h, format, type,
                    melonGLConvertIf1555(type, (size_t)w * (size_t)h, data));
}
static inline void melonGLTexImage3D(GLenum target, GLint level, GLint internalformat,
    GLsizei w, GLsizei h, GLsizei d, GLint border, GLenum format, GLenum type, const void* data)
{
    glTexImage3D(target, level, internalformat, w, h, d, border, format, type,
                 melonGLConvertIf1555(type, (size_t)w * (size_t)h * (size_t)d, data));
}
static inline void melonGLTexSubImage3D(GLenum target, GLint level, GLint xo, GLint yo, GLint zo,
    GLsizei w, GLsizei h, GLsizei d, GLenum format, GLenum type, const void* data)
{
    glTexSubImage3D(target, level, xo, yo, zo, w, h, d, format, type,
                    melonGLConvertIf1555(type, (size_t)w * (size_t)h * (size_t)d, data));
}
#define glTexImage2D    melonGLTexImage2D
#define glTexSubImage2D melonGLTexSubImage2D
#define glTexImage3D    melonGLTexImage3D
#define glTexSubImage3D melonGLTexSubImage3D

// GLES has the *f suffixed depth entry points only.
static inline void melonGLClearDepth(double d) { glClearDepthf((float)d); }
static inline void melonGLDepthRange(double n, double f) { glDepthRangef((float)n, (float)f); }
#define glClearDepth  melonGLClearDepth
#define glDepthRange  melonGLDepthRange

// glDrawBuffer(single) -> glDrawBuffers(1, &buf)
static inline void melonGLDrawBuffer(GLenum buf)
{
    GLenum bufs[1] = { buf };
    glDrawBuffers(1, bufs);
}
#define glDrawBuffer melonGLDrawBuffer

// Frag data locations are declared in-shader via layout(location=) on GLES
// (see the GL_ES guarded FRAGLOC macro in the MRT shaders). This is a no-op.
static inline void melonGLBindFragDataLocation(GLuint /*program*/, GLuint /*colorNumber*/, const char* /*name*/) {}
#define glBindFragDataLocation melonGLBindFragDataLocation

// glMapBuffer(target, access) -> glMapBufferRange over the whole buffer.
#ifndef GL_READ_ONLY
#define GL_READ_ONLY  0x88B8
#endif
#ifndef GL_WRITE_ONLY
#define GL_WRITE_ONLY 0x88B9
#endif
#ifndef GL_READ_WRITE
#define GL_READ_WRITE 0x88BA
#endif
static inline void* melonGLMapBuffer(GLenum target, GLenum access)
{
    GLint size = 0;
    glGetBufferParameteriv(target, GL_BUFFER_SIZE, &size);
    GLbitfield bits = 0;
    if (access == GL_READ_ONLY)  bits = GL_MAP_READ_BIT;
    else if (access == GL_WRITE_ONLY) bits = GL_MAP_WRITE_BIT;
    else bits = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT;
    return glMapBufferRange(target, 0, size, bits);
}
#define glMapBuffer melonGLMapBuffer

#endif // __ANDROID__
#endif // MELONDS_GLES_COMPAT_H
