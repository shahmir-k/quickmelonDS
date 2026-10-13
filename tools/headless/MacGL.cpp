// Offscreen OpenGL 4.1 core context for liteDS-headless on macOS (--renderer hybrid|glhires):
// the lib's GL renderers draw into their own FBOs, so no window or drawable is needed.
#include "frontend/glad/glad.h"
#include <OpenGL/OpenGL.h>
#include <dlfcn.h>

void* MacGLProc(const char* name)
{
    static void* lib = dlopen("/System/Library/Frameworks/OpenGL.framework/OpenGL", RTLD_LAZY);
    return lib ? dlsym(lib, name) : nullptr;
}

bool MacGLCreate()
{
    CGLPixelFormatAttribute attr[] = {
        kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_GL4_Core,
        kCGLPFAAccelerated, kCGLPFAColorSize, (CGLPixelFormatAttribute)24,
        kCGLPFAAlphaSize, (CGLPixelFormatAttribute)8, (CGLPixelFormatAttribute)0};
    CGLPixelFormatObj pf = nullptr;
    GLint n = 0;
    if (CGLChoosePixelFormat(attr, &pf, &n) != kCGLNoError || !pf) return false;
    CGLContextObj ctx = nullptr;
    CGLError e = CGLCreateContext(pf, nullptr, &ctx);
    CGLDestroyPixelFormat(pf);
    return e == kCGLNoError && CGLSetCurrentContext(ctx) == kCGLNoError;
}

// GL 4.1 lacks a few entry points the renderers use (4.2/4.3): stand-ins with the same result.
// Called after glad has loaded (the glad pointers are plain globals).
static void InvalidateNop(GLenum, GLsizei, const GLenum*) {}   // a hint: contents undefined anyway
static void FmtOf(GLenum ifmt, GLenum& fmt, GLenum& type)
{
    fmt = 0x1908; type = 0x1401;                     // GL_RGBA, GL_UNSIGNED_BYTE
    if (ifmt == 0x8D7C) fmt = 0x8D99;                // GL_RGBA8UI -> GL_RGBA_INTEGER
}
static void TexStorage2D(GLenum t, GLsizei levels, GLenum ifmt, GLsizei w, GLsizei h)
{
    GLenum fmt, type; FmtOf(ifmt, fmt, type);
    for (int l = 0; l < levels; l++)
        glTexImage2D(t, l, ifmt, w > 1 ? w >> l : 1, h > 1 ? h >> l : 1, 0, fmt, type, nullptr);
}
static void TexStorage3D(GLenum t, GLsizei levels, GLenum ifmt, GLsizei w, GLsizei h, GLsizei d)
{
    GLenum fmt, type; FmtOf(ifmt, fmt, type);
    for (int l = 0; l < levels; l++)
        glTexImage3D(t, l, ifmt, w > 1 ? w >> l : 1, h > 1 ? h >> l : 1, d, 0, fmt, type, nullptr);
}
void MacGLShims()
{
    if (!glad_glInvalidateFramebuffer) glad_glInvalidateFramebuffer = InvalidateNop;
    if (!glad_glTexStorage2D) glad_glTexStorage2D = TexStorage2D;
    if (!glad_glTexStorage3D) glad_glTexStorage3D = TexStorage3D;
}
