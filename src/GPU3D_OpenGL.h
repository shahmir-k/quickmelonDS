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

#pragma once

#ifdef OGLRENDERER_ENABLED
#include <string>
#include "GPU3D.h"
#include "OpenGLSupport.h"
#include "GPU3D_TexcacheOpenGL.h"
#include "NonStupidBitfield.h"
#include <functional>
#include <vector>

namespace melonDS
{
class GLRenderer;

class GLRenderer3D : public Renderer3D
{
public:
    // parent == nullptr: no GLRenderer around (the hybrid renderer). Display captures
    // then live in emulated VRAM only, never as hi-res GL textures.
    GLRenderer3D(melonDS::GPU3D& gpu3D, GLRenderer* parent) noexcept;
    ~GLRenderer3D() override;
    bool Init() override;
    void Reset() override;

    void SetRenderSettings(int scale, bool betterpolygons) noexcept;
    void SetBetterPolygons(bool betterpolygons) noexcept;
    void SetScaleFactor(int scale) noexcept;
    [[nodiscard]] bool GetBetterPolygons() const noexcept { return BetterPolygons; }
    [[nodiscard]] int GetScaleFactor() const noexcept { return ScaleFactor; }

    void RenderFrame() override;
    u32* GetLine(int line) override;

    // RenderFrame split for the hybrid's GL thread: PrepareFrame on the emu thread,
    // RenderPreparedFrame on the thread that owns this renderer's GL objects.
    // slot: up to two prepared frames may be outstanding (one rendering, one queued);
    // beforeVRAMWrite runs before PrepareFrame modifies the flat texture VRAM a running
    // frame may still be reading (the caller waits for it there).
    // copyPolys=false (LITEV_GL_NOSNAPCOPY): the job reads GPU3D's polygon/vertex bank in place;
    // returns that bank (0/1, -1: none read), which the caller must keep unwritten until the job ends.
    int PrepareFrame(int slot = 0, const std::function<void()>& beforeVRAMWrite = {}, bool copyPolys = true);
#ifdef LITEV_HYB_TEXSTAGE
    void EnableTexStaging() { Texcache.EnableStaging(); }
#endif
    void RenderPreparedFrame(int slot = 0);

    // Colour output ring (hybrid renderer): with n > 1 every rendered frame goes to the
    // next of n colour textures, so a frame's 3D stays readable while later frames
    // render. n = 1 (the default) is the single buffer GLRenderer uses.
    static constexpr int MaxColorRing = 6;
    mutable u32 StatDraws = 0, StatPolys = 0;   // diagnostic counters (LITEV_HYB log)
    void SetColorRing(int n) noexcept;
    [[nodiscard]] int GetCurColor() const noexcept { return CurColor; }
    [[nodiscard]] GLuint GetColorTex(int i) const noexcept { return ColorBufferTex[i]; }

private:
    GLRenderer* Parent;

    // GL version requirements
    // * texelFetch: 3.0 (GLSL 1.30)     (3.2/1.50 for MS)
    // * UBO: 3.1

    struct RendererPolygon
    {
        Polygon* PolyData;

        u32 NumIndices;
        u32 IndicesOffset;
        GLuint PrimType;

        u32 NumEdgeIndices;
        u32 EdgeIndicesOffset;

        u32 RenderKey;

        GLuint TexID;
        u32 TexRepeat;
    };

    //GLCompositor CurGLCompositor;
    RendererPolygon PolygonList[2048] {};

    bool TexEnable;
    TexcacheOpenGL Texcache;

    bool BuildRenderShader(int flags);   // bit0 W-buffer, bit1 no alpha test (LITEV_GL_OPAQUE_NODISCARD)
    void UseRenderShader(int flags);
    void SetupPolygon(RendererPolygon* rp, Polygon* polygon) const;
    u32* SetupVertex(const Polygon* poly, int vid, const Vertex* vtx, u32 vtxattr, u32 texlayer, u32* vptr) const;
    void BuildPolygons(RendererPolygon* polygons, int npolys, int captureinfo[16]);
    void SetupPolygonTexture(const RendererPolygon* poly) const;
    int RenderSinglePolygon(int i) const;
    int RenderPolygonBatch(int i) const;
    int RenderPolygonEdgeBatch(int i) const;
    void RenderSceneChunk(int y, int h);
#ifdef LITEV_GL_WARM_VARIANTS
    void WarmVariants();
    bool Warmed = false;
#endif


    enum
    {
        RenderMode_Opaque = 0,
        RenderMode_Translucent,
        RenderMode_ShadowMask,
        RenderMode_OpaqueBlended,   // LITEV_GL_BATCH_NEEDOPAQUE: opaque and translucent texels in one draw
    };
    static constexpr u32 RenderKey_NoDiscard = 0x40000000;   // LITEV_GL_OPAQUE_NODISCARD (bits 20-29 = texattr)


    GLuint ClearShaderPlain {};
    GLuint ClearShaderBitmap {};

    GLuint RenderShader[4] {};
    GLint RenderModeULoc = 0;
    GLuint CurShaderID = -1;
    // LITEV_GL_WBUF_EARLYZ: this frame's W-buffer depth mapping, window depth = 1 - WZ0/w
    // (0: the plain z/2^24 mapping)
    float WZ0 = 0;
    static bool WEarlyZ();
    static bool NoDiscard();
    static bool FogShaderBlend();
    static std::string FogFetchSource();

    GLuint FinalPassEdgeShader {};
    GLuint FinalPassFogShader {};
    GLuint FinalPassFogFetchShader {};   // debug.litev.gl3dtile: fog by framebuffer fetch
    bool TileMode = false;               // debug.litev.gl3dtile: invalidate around the 3D pass

    // std140 compliant structure
    struct
    {
        float uScreenSize[2];       // vec2       0 / 2
        u32 uDispCnt;               // int        2 / 1
        u32 __pad0;
        float uToonColors[32][4];   // vec4[32]   4 / 128
        float uEdgeColors[8][4];    // vec4[8]    132 / 32
        float uFogColor[4];         // vec4       164 / 4
        float uFogDensity[34][4];   // float[34]  168 / 136
        u32 uFogOffset;             // int        304 / 1
        u32 uFogShift;              // int        305 / 1
        u32 _pad1[2];               // int        306 / 2
    } ShaderConfig {};

    GLuint ShaderConfigUBO {};
    int NumFinalPolys {}, NumOpaqueFinalPolys {};

    GLuint ClearVertexBufferID = 0, ClearVertexArrayID {};
    GLint ClearUniformLoc[4] {};

    GLint ClearBitmapULoc[2] {};
    GLuint ClearBitmapTex[2];
    u32* ClearBitmap[2];
    u8 ClearBitmapDirty;

    // vertex buffer
    // * XYZW: 4x16bit
    // * RGBA: 4x8bit
    // * ST: 2x16bit
    // * polygon data: 3x32bit (polygon/texture attributes)
    //
    // polygon attributes:
    // * bit4-7, 11, 14-15, 24-29: POLYGON_ATTR
    // * bit16-20: Z shift
    // * bit8: front-facing (?)
    // * bit9: W-buffering (?)

    GLuint VertexBufferID {};
    u32 VertexBuffer[10240 * 7] {};
    u32 NumVertices {};

    GLuint VertexArrayID {};
    GLuint IndexBufferID {};
    u16 IndexBuffer[2048 * 40] {};
    u32 NumIndices {}, NumEdgeIndices {};

    const u32 EdgeIndicesOffset = 2048 * 30;

    int ScaleFactor {};
    bool LineQuads = true;   // line polygons as thin quads (debug.litev.linequads=0: GL_LINES)
    bool BetterPolygons {};
    int ScreenW {}, ScreenH {};

    GLuint ColorBufferTex[MaxColorRing] {};
    int ColorRing = 1, CurColor = 0;
    // render state snapshot taken in PrepareFrame (Next[slot]); RenderPreparedFrame copies
    // it to S, which everything that renders reads
    struct Snap
    {
        bool Skip;
        int Color;
        u32 RenderDispCnt, RenderClearAttr1, RenderClearAttr2;
        u32 RenderFogColor, RenderFogOffset, RenderFogShift;
        u16 RenderToonTable[32], RenderEdgeTable[8];
        u8 RenderFogDensityTable[34];
        u32 RenderNumPolygons;
        int CaptureInfo[16];   // GLRenderer's captures that textures read (-1: plain VRAM)
        Polygon* RenderPolygonRAM[2048];
    } S {}, Next[2] {};
    // the polygons and vertices of Next[slot] (RenderPolygonRAM points here): GPU3D reuses its
    // polygon/vertex bank at the VBlank after next, possibly while this job still draws
    std::vector<Polygon> PolyCopy[2];
    std::vector<Vertex> VtxCopy[2];
    GLuint DepthBufferTex {}, AttrBufferTex {};
    void AllocColorBuffers() noexcept;

    GLuint MainFramebuffer {};
    GLuint WrapSampler[9] {};   // [wrapS * 3 + wrapT], 0 clamp / 1 repeat / 2 mirror
};
}
#endif