// Routes the GL calls of the including file through GLDefer (LITEV_GLHI_DEFER). Include it
// after every other header. Calls with plain arguments are recorded as they are; data
// pointers are copied at the call; calls that return something run on the worker at once.
#ifndef GLDEFERCALLS_H
#define GLDEFERCALLS_H

#include "GLDefer.h"

namespace melonDS
{
namespace GLDefer
{
// bytes per pixel of the upload formats these renderers use
inline size_t PixelBytes(GLenum format, GLenum type)
{
    size_t c = (format == GL_RGBA || format == GL_RGBA_INTEGER) ? 4 : (format == GL_RG || format == GL_RG_INTEGER) ? 2 : 1;
    switch (type)
    {
    case GL_UNSIGNED_SHORT_5_5_5_1:   // (GLES: also what GL_UNSIGNED_SHORT_1_5_5_5_REV maps to)
    case GL_UNSIGNED_SHORT_4_4_4_4: case GL_UNSIGNED_SHORT_5_6_5:
        return 2;
    case GL_UNSIGNED_SHORT: case GL_SHORT: case GL_HALF_FLOAT: return 2 * c;
    case GL_UNSIGNED_INT: case GL_INT: case GL_FLOAT: return 4 * c;
    default: return c;
    }
}
inline void TexImage2D(GLenum t, GLint l, GLint i, GLsizei w, GLsizei h, GLint b, GLenum f, GLenum ty, const void* d)
{ Call(&melonGLTexImage2D, t, l, i, w, h, b, f, ty, Data(d, (size_t)w * h * PixelBytes(f, ty))); }
inline void TexSubImage2D(GLenum t, GLint l, GLint x, GLint y, GLsizei w, GLsizei h, GLenum f, GLenum ty, const void* d)
{ Call(&melonGLTexSubImage2D, t, l, x, y, w, h, f, ty, Data(d, (size_t)w * h * PixelBytes(f, ty))); }
inline void TexImage3D(GLenum t, GLint l, GLint i, GLsizei w, GLsizei h, GLsizei dp, GLint b, GLenum f, GLenum ty, const void* d)
{ Call(&melonGLTexImage3D, t, l, i, w, h, dp, b, f, ty, Data(d, (size_t)w * h * dp * PixelBytes(f, ty))); }
inline void TexSubImage3D(GLenum t, GLint l, GLint x, GLint y, GLint z, GLsizei w, GLsizei h, GLsizei dp, GLenum f, GLenum ty, const void* d)
{ Call(&melonGLTexSubImage3D, t, l, x, y, z, w, h, dp, f, ty, Data(d, (size_t)w * h * dp * PixelBytes(f, ty))); }
}
}

#undef glTexImage2D
#undef glTexSubImage2D
#undef glTexImage3D
#undef glTexSubImage3D
#undef glDrawBuffer
#undef glClearDepth

#define GLD_ ::melonDS::GLDefer::
// recorded as is
#define glActiveTexture(...)            GLD_ Call(&glActiveTexture, __VA_ARGS__)
#define glBindBuffer(...)               GLD_ Call(&glBindBuffer, __VA_ARGS__)
#define glBindBufferBase(...)           GLD_ Call(&glBindBufferBase, __VA_ARGS__)
#define glBindBufferRange(...)          GLD_ Call(&glBindBufferRange, __VA_ARGS__)
#define glBindFramebuffer(...)          GLD_ Call(&glBindFramebuffer, __VA_ARGS__)
#define glBindTexture(...)              GLD_ Call(&glBindTexture, __VA_ARGS__)
#define glBindVertexArray(...)          GLD_ Call(&glBindVertexArray, __VA_ARGS__)
#define glBlitFramebuffer(...)          GLD_ Call(&glBlitFramebuffer, __VA_ARGS__)
#define glClear(...)                    GLD_ Call(&glClear, __VA_ARGS__)
#define glClearColor(...)               GLD_ Call(&glClearColor, __VA_ARGS__)
#define glClearDepth(...)               GLD_ Call(&melonGLClearDepth, __VA_ARGS__)
#define glColorMaski(...)               GLD_ Call(&glColorMaski, __VA_ARGS__)
#define glDeleteProgram(...)            GLD_ Call(&glDeleteProgram, __VA_ARGS__)
#define glDeleteSync(...)               GLD_ Call(&glDeleteSync, __VA_ARGS__)
#define glDepthFunc(...)                GLD_ Call(&glDepthFunc, __VA_ARGS__)
#define glDepthMask(...)                GLD_ Call(&glDepthMask, __VA_ARGS__)
#define glDisable(...)                  GLD_ Call(&glDisable, __VA_ARGS__)
#define glDrawArrays(...)               GLD_ Call(&glDrawArrays, __VA_ARGS__)
#define glDrawBuffer(...)               GLD_ Call(&melonGLDrawBuffer, __VA_ARGS__)
#define glEnable(...)                   GLD_ Call(&glEnable, __VA_ARGS__)
#define glEnableVertexAttribArray(...)  GLD_ Call(&glEnableVertexAttribArray, __VA_ARGS__)
#define glFlush(...)                    GLD_ Call(&glFlush)
#define glFramebufferTexture(...)       GLD_ Call(&glFramebufferTexture, __VA_ARGS__)
#define glFramebufferTexture2D(...)     GLD_ Call(&glFramebufferTexture2D, __VA_ARGS__)
#define glFramebufferTextureLayer(...)  GLD_ Call(&glFramebufferTextureLayer, __VA_ARGS__)
#define glPixelStorei(...)              GLD_ Call(&glPixelStorei, __VA_ARGS__)
#define glReadBuffer(...)               GLD_ Call(&glReadBuffer, __VA_ARGS__)
#define glScissor(...)                  GLD_ Call(&glScissor, __VA_ARGS__)
#define glStencilFunc(...)              GLD_ Call(&glStencilFunc, __VA_ARGS__)
#define glStencilMask(...)              GLD_ Call(&glStencilMask, __VA_ARGS__)
#define glStencilOp(...)                GLD_ Call(&glStencilOp, __VA_ARGS__)
#define glTexParameteri(...)            GLD_ Call(&glTexParameteri, __VA_ARGS__)
#define glUniform1f(...)                GLD_ Call(&glUniform1f, __VA_ARGS__)
#define glUniform1i(...)                GLD_ Call(&glUniform1i, __VA_ARGS__)
#define glUniform2i(...)                GLD_ Call(&glUniform2i, __VA_ARGS__)
#define glUniformBlockBinding(...)      GLD_ Call(&glUniformBlockBinding, __VA_ARGS__)
#define glUseProgram(...)               GLD_ Call(&glUseProgram, __VA_ARGS__)
#define glVertexAttribIPointer(...)     GLD_ Call(&glVertexAttribIPointer, __VA_ARGS__)
#define glVertexAttribPointer(...)      GLD_ Call(&glVertexAttribPointer, __VA_ARGS__)
#define glViewport(...)                 GLD_ Call(&glViewport, __VA_ARGS__)
#define glWaitSync(...)                 GLD_ Call(&glWaitSync, __VA_ARGS__)
// data copied at the call
#define glBufferData(t, n, d, u)        GLD_ Call(&glBufferData, t, n, GLD_ Data(d, n), u)
#define glBufferSubData(t, o, n, d)     GLD_ Call(&glBufferSubData, t, o, n, GLD_ Data(d, n))
#define glDrawBuffers(n, b)             GLD_ Call(&glDrawBuffers, n, GLD_ Data(b, (n) * sizeof(GLenum)))
#define glDeleteBuffers(n, p)           GLD_ Call(&glDeleteBuffers, n, GLD_ Data(p, (n) * sizeof(GLuint)))
#define glDeleteFramebuffers(n, p)      GLD_ Call(&glDeleteFramebuffers, n, GLD_ Data(p, (n) * sizeof(GLuint)))
#define glDeleteTextures(n, p)          GLD_ Call(&glDeleteTextures, n, GLD_ Data(p, (n) * sizeof(GLuint)))
#define glDeleteVertexArrays(n, p)      GLD_ Call(&glDeleteVertexArrays, n, GLD_ Data(p, (n) * sizeof(GLuint)))
#define glTexImage2D(...)               GLD_ TexImage2D(__VA_ARGS__)
#define glTexSubImage2D(...)            GLD_ TexSubImage2D(__VA_ARGS__)
#define glTexImage3D(...)               GLD_ TexImage3D(__VA_ARGS__)
#define glTexSubImage3D(...)            GLD_ TexSubImage3D(__VA_ARGS__)
// results needed now: run on the worker at once
#define glCheckFramebufferStatus(...)   GLD_ CallSync(&glCheckFramebufferStatus, __VA_ARGS__)
#define glFenceSync(...)                GLD_ CallSync(&glFenceSync, __VA_ARGS__)
#define glGetUniformBlockIndex(...)     GLD_ CallSync(&glGetUniformBlockIndex, __VA_ARGS__)
#define glGetUniformLocation(...)       GLD_ CallSync(&glGetUniformLocation, __VA_ARGS__)
#define glFinish(...)                   GLD_ CallSyncV(&glFinish)
#define glGenBuffers(...)               GLD_ CallSyncV(&glGenBuffers, __VA_ARGS__)
#define glGenFramebuffers(...)          GLD_ CallSyncV(&glGenFramebuffers, __VA_ARGS__)
#define glGenTextures(...)              GLD_ CallSyncV(&glGenTextures, __VA_ARGS__)
#define glGenVertexArrays(...)          GLD_ CallSyncV(&glGenVertexArrays, __VA_ARGS__)
#define glGetIntegerv(...)              GLD_ CallSyncV(&glGetIntegerv, __VA_ARGS__)
#define glGetBooleani_v(...)            GLD_ CallSyncV(&glGetBooleani_v, __VA_ARGS__)
#define glReadPixels(...)               GLD_ CallSyncV(&glReadPixels, __VA_ARGS__)

#endif
