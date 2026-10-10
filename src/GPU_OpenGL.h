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

#ifndef GPU_OPENGL_H
#define GPU_OPENGL_H

#include <atomic>
#include <functional>
#include <memory>
#include "OpenGLSupport.h"
#include "GPU.h"
#include "GPU2D_OpenGL.h"
#include "GPU3D_OpenGL.h"
#include "GPU3D_Compute.h"

namespace melonDS
{

class GLThread3D;
class GLWorker;

class GLRenderer : public Renderer
{
public:
    GLRenderer(melonDS::NDS& nds, bool compute);
    ~GLRenderer() override;
    bool Init() override;
    void Reset() override;
    void Stop() override;

    void PreSavestate() override;
    void PostSavestate() override;

    void SetRenderSettings(RendererSettings& settings) override;

    void DrawScanline(u32 line) override;
    void DrawSprites(u32 line) override;

    void VBlank() override;
    void VBlankEnd() override;

    void AllocCapture(u32 bank, u32 start, u32 len) override;
    void SyncVRAMCapture(u32 bank, u32 start, u32 len, bool complete) override;

    bool GetFramebuffers(void** top, void** bottom) override;

    bool NeedsShaderCompile() override;
    void ShaderCompileStep(int& current, int& count) override;

    void Start3DRendering() override;

    // Copy the front buffer (top at row 0, bottom at row (192 + 2) * scale) into dstTex on a
    // present thread with its own shared context: pre() runs there first, post() after the
    // copy is issued (fence + hand-off). debug.litev.glhiasync=0: on the calling thread.
    void PresentIntoAsync(GLuint dstTex, int dstWidth, std::function<void()> pre, std::function<void()> post);
    void WaitPresent();

private:
    std::unique_ptr<GLWorker> Present;
    GLuint PresReadFB = 0, PresDrawFB = 0;          // present-thread objects
    u64 PresSeq = 0, BufPresSeq[2] {};
    std::atomic<u64> PresDone { 0 };
    GLsync BufFence[2] {};                          // the copy of FPOutputTex[i] (GPU-side)
    void BlitFront(int front, GLuint dstTex, int dstWidth, GLuint readFB, GLuint drawFB);

    // 3D on its own GL thread (debug.litev.glhithread=0: inline). Get3DTex() returns the
    // 3D output the current frame shows, waiting (GPU-side) for its render on first use.
    GLThread3D* Thread3D = nullptr;
    bool Synced3D = false;
    int Prev3DColor = 0;
    GLuint Get3DTex();

    friend class GLRenderer2D;
    friend class GLRenderer3D;
    friend class ComputeRenderer3D;

    bool IsCompute;

    int ScaleFactor;
    int ScreenW, ScreenH;

    GLuint RectVtxBuffer;
    GLuint RectVtxArray;

    GLuint OutputTex3D;
    GLuint OutputTex2D[2];

    struct sFinalPassConfig
    {
        u32 uScreenSwap[192];
        u32 uScaleFactor;
        u32 uAuxLayer;
        u32 uDispModeA;
        u32 uDispModeB;
        u32 uBrightModeA;
        u32 uBrightModeB;
        u32 uBrightFactorA;
        u32 uBrightFactorB;
        float uAuxColorFactor;
        u32 __pad0[3];
    } FinalPassConfig;

    GLuint FPShader;
    GLuint FPConfigUBO;

    GLuint FPVertexBufferID;
    GLuint FPVertexArrayID;

    GLuint AuxInputTex;                 // aux input (VRAM and mainmem FIFO)

    // texture/fb for display capture VRAM input
    GLuint CaptureVRAMTex;
    GLuint CaptureVRAMFB;

    GLuint FPOutputTex[2];               // final output
    GLuint FPOutputFB[2];

    struct sCaptureConfig
    {
        float uInvCaptureSize[2];
        u32 uSrcALayer;
        u32 uSrcBLayer;
        u32 uSrcBOffset;
        u32 uDstMode;
        u32 uBlendFactors[2];
        float uSrcAOffset[192];
        float uSrcBColorFactor;
        u32 __pad0[3];
    } CaptureConfig;

    GLuint CaptureShader;
    GLuint CaptureConfigUBO;

    GLuint CaptureVtxBuffer;
    GLuint CaptureVtxArray;

    GLuint CaptureOutput256FB[4];
    GLuint CaptureOutput256Tex;
    GLuint CaptureOutput128FB[16];
    GLuint CaptureOutput128Tex;

    GLuint CapDownShader;
    GLint CapDownInputLayerULoc;

    GLuint CaptureSyncFB;
    GLuint CaptureSyncTex;

    u16* AuxInputBuffer[2];
    u8 AuxUsageMask;

    u32 DispCntA, DispCntB;
    u16 MasterBrightnessA, MasterBrightnessB;
    u32 CaptureCnt;

    bool NeedPartialRender;
    int LastLine;
    int StatFrames = 0;
    int LastCapLine;
    int Aux0VRAMCap;

    void SetScaleFactor(int scale);

    void RenderScreen(int ystart, int yend);
    void DoCapture(int ystart, int yend);
    void DownscaleCapture(int width, int height, int layer);
};

}

#endif // GPU_OPENGL_H
