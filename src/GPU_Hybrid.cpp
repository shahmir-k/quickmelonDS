/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#include <string.h>
#include <algorithm>
#include "NDS.h"
#include "GPU_Hybrid.h"

namespace melonDS
{

#include "OpenGL_shaders/HybridMergeVS.h"
#include "OpenGL_shaders/HybridMergeFS.h"

#ifndef LITEV_SOFT2D_THREADED
#error "HybridRenderer needs the threaded software 2D (LITEV_SOFT2D_THREADED)"
#endif

HybridRenderer::HybridRenderer(melonDS::NDS& nds)
    : SoftRenderer(nds)
{
    // 3D on the GPU. No GLRenderer parent: captures stay in emulated VRAM.
    Rend3D = std::make_unique<GLRenderer3D>(GPU.GPU3D, nullptr);
    Hybrid = true;
    HybridCheck = false;
    for (int i = 0; i < 3; i++) HybFB[i] = new u32[2 * 192 * HybStride]();
    for (int i = 0; i < 2; i++) Hyb3D[i] = new u32[256 * 192]();
}

HybridRenderer::~HybridRenderer()
{
    StopAsyncThread();   // the 2D thread reads HybFB; stop it before GL teardown below
    glDeleteProgram(MergeShader);
    glDeleteVertexArrays(1, &EmptyVAO);
    glDeleteTextures(1, &DescTex);
    glDeleteTextures(2, OutTex);
    glDeleteFramebuffers(2, OutFB);
    glDeleteFramebuffers(1, &ReadFB);
    glDeleteFramebuffers(1, &DownFB);
    glDeleteTextures(1, &DownTex);
}

bool HybridRenderer::Init()
{
    if (!Rend3D->Init()) return false;
    // 4 colour buffers: the 3D a presented frame pairs with is up to 3 renders old
    // (async 2D pipeline at depth 2), and one more is being rendered.
    GL3D()->SetColorRing(GLRenderer3D::MaxColorRing);

    if (!OpenGL::CompileVertexFragmentProgram(MergeShader,
            kHybridMergeVS, kHybridMergeFS, "HybridMergeShader",
            {}, {{"oTopColor", 0}, {"oBottomColor", 1}}))
        return false;

    glUseProgram(MergeShader);
    glUniform1i(glGetUniformLocation(MergeShader, "DescTex"), 0);
    glUniform1i(glGetUniformLocation(MergeShader, "Tex3D"), 1);
    ScaleULoc = glGetUniformLocation(MergeShader, "uScale");

    glGenVertexArrays(1, &EmptyVAO);

    auto texParams = [](GLenum target)
    {
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    };

    glGenTextures(1, &DescTex);
    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    texParams(GL_TEXTURE_2D_ARRAY);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8UI, HybStride, 192, 2, 0, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, nullptr);

    glGenTextures(2, OutTex);
    glGenFramebuffers(2, OutFB);
    for (int i = 0; i < 2; i++)
    {
        glBindTexture(GL_TEXTURE_2D_ARRAY, OutTex[i]);
        texParams(GL_TEXTURE_2D_ARRAY);
    }

    glGenTextures(1, &DownTex);
    glBindTexture(GL_TEXTURE_2D, DownTex);
    texParams(GL_TEXTURE_2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 256, 192, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glGenFramebuffers(1, &DownFB);
    glBindFramebuffer(GL_FRAMEBUFFER, DownFB);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, DownTex, 0);
    glGenFramebuffers(1, &ReadFB);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    SetScale(1);
    GL3D()->SetRenderSettings(1, false);
    return true;
}

void HybridRenderer::Reset()
{
    SoftRenderer::Reset();   // drains the 2D, resets the 2D renderers and the GL 3D
    for (int i = 0; i < 3; i++) memset(HybFB[i], 0, 2 * 192 * HybStride * sizeof(u32));
}

void HybridRenderer::PreSavestate()
{
    FlushAsyncRender();
}

void HybridRenderer::PostSavestate()
{
    Rend3D->Reset();   // texture cache
}

void HybridRenderer::SetRenderSettings(RendererSettings& settings)
{
    SetScale(std::max(1, settings.ScaleFactor));
    GL3D()->SetRenderSettings(Scale, settings.BetterPolygons);
}

void HybridRenderer::SetScale(int scale)
{
    if (scale == Scale) return;
    Scale = scale;

    const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    for (int i = 0; i < 2; i++)
    {
        glBindTexture(GL_TEXTURE_2D_ARRAY, OutTex[i]);
        glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, 256 * Scale, 192 * Scale, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindFramebuffer(GL_FRAMEBUFFER, OutFB[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, OutTex[i], 0, 0);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, OutTex[i], 0, 1);
        glDrawBuffers(2, bufs);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

int HybridRenderer::HybridCurrentTag()
{
    return GL3D()->GetCurColor();
}

// Emu thread, VBlank of a capture frame: the frame's 3D (latest ring entry) at 1x in the
// software 3D line format (6-bit RGB, 5-bit alpha in bits 24-28). Synchronous; the 3D was
// submitted at the previous VCount 215, so the wait is short.
void HybridRenderer::HybridReadback3D(u32* dst)
{
    GLuint tex = GL3D()->GetColorTex(GL3D()->GetCurColor());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, ReadFB);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    glDisable(GL_SCISSOR_TEST);
    if (Scale > 1)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, DownFB);
        glBlitFramebuffer(0, 0, 256 * Scale, 192 * Scale, 0, 0, 256, 192, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, DownFB);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, 256, 192, GL_RGBA, GL_UNSIGNED_BYTE, ReadBuf);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    for (int i = 0; i < 256 * 192; i++)
    {
        const u8* p = &ReadBuf[i * 4];
#ifdef __ANDROID__
        u32 r = p[2], g = p[1], b = p[0];   // the GLES 3D pass emits BGRA (3DRenderFS)
#else
        u32 r = p[0], g = p[1], b = p[2];
#endif
        u32 a = p[3];
        dst[i] = ((r * 63 + 127) / 255) | (((g * 63 + 127) / 255) << 8) |
                 (((b * 63 + 127) / 255) << 16) | (((a * 31 + 127) / 255) << 24);
    }
}

// Emu thread, after RunFrame: upload the last completed 2D frame's descriptors and
// merge them with the 3D that frame pairs with, at Nx, into a 2-layer array texture.
bool HybridRenderer::GetFramebuffers(void** top, void** bottom)
{
    const int fb = AsyncPresentBuf;

    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, HybStride, 192, 2,
                    GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, HybFB[fb]);

    OutIdx ^= 1;
    glBindFramebuffer(GL_FRAMEBUFFER, OutFB[OutIdx]);
    glViewport(0, 0, 256 * Scale, 192 * Scale);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glColorMaski(1, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);

    glUseProgram(MergeShader);
    glUniform1i(ScaleULoc, Scale);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, GL3D()->GetColorTex(HybTag[fb]));
    glBindVertexArray(EmptyVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    *top = &OutTex[OutIdx];
    *bottom = nullptr;
    return false;
}

}
