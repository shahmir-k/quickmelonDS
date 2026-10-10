#include "GPU3D_TexcacheOpenGL.h"
#include "LitevSoftProf.h"
#include "Platform.h"

namespace melonDS
{

#ifdef LITEV_GL_TEX_UNORM
// The GL 3D renderer's textures as normalized RGBA8 instead of RGBA8UI: the shader samples a float
// texture directly instead of converting 6-bit integer texels (tcol = vec4(uvec4) / (63,63,63,31)),
// the expensive part of texturing on the Mali. Colours expand 6 -> 8 bits ((c << 2) | (c >> 4)),
// alpha 5 -> 8 ((a << 3) | (a >> 2)): within 1/510 of c/63 and a/31, and every alpha keeps its side
// of the shader's 0.5/31 and 30.5/31 tests. debug.litev.glunorm=0 turns it off.
bool TexUnorm()
{
    static const bool on = OpenGL::Prop("glunorm", 1) != 0;
    return on;
}
#endif

GLuint TexcacheOpenGLLoader::GenerateTexture(u32 width, u32 height, u32 layers)
{
    GLuint texarray;
    glGenTextures(1, &texarray);
    glBindTexture(GL_TEXTURE_2D_ARRAY, texarray);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    if (IsCompute)
        glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGBA8UI, width, height, layers);
    else
    {
        const double t0 = LSP_NOW();
#ifdef LITEV_GL_TEX_UNORM
        if (TexUnorm())
            LSP_GLT(TexNew, glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, width, height, layers, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr));
        else
#endif
        LSP_GLT(TexNew, glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8UI, width, height, layers, 0, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, nullptr));
#ifdef LITEV_SOFTPROF
        if (LitevSP::StallLogOn())
            Platform::Log(Platform::LogLevel::Info, "LITEV_STALL texnew %ux%u x%u (%u KB) %.2f ms\n", width, height, layers, width * height * 4 * layers / 1024, LSP_NOW() - t0);
#endif
    }

    return texarray;
}

void TexcacheOpenGLLoader::UploadTexture(GLuint handle, u32 width, u32 height, u32 layer, void* data)
{
    glBindTexture(GL_TEXTURE_2D_ARRAY, handle);
#ifdef LITEV_GL_TEX_UNORM
    if (!IsCompute && TexUnorm())
    {
        // ponytail: one static buffer (the texcache uploads from one thread at a time)
        static u32 buf[1024 * 1024];
        const u32* src = (const u32*)data;
        for (u32 i = 0; i < width * height; i++)
        {
            const u32 v = src[i];
            const u32 c = v & 0x3F3F3F;          // 6-bit r, g, b
            const u32 a = (v >> 24) & 0x1F;
            buf[i] = (c << 2) | ((c >> 4) & 0x030303) | (((a << 3) | (a >> 2)) << 24);
        }
        LSP_GLT(TexUp, glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, layer, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, buf));
        return;
    }
#endif
    LSP_GLT(TexUp, glTexSubImage3D(GL_TEXTURE_2D_ARRAY,
        0, 0, 0, layer,
        width, height, 1,
        GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, data));
}

void TexcacheOpenGLLoader::DeleteTexture(GLuint handle)
{
    glDeleteTextures(1, &handle);
}

}