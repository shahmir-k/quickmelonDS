#include "GPU3D_TexcacheOpenGL.h"
#include "LitevSoftProf.h"
#include "Platform.h"

namespace melonDS
{

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