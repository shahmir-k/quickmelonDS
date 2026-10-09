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

#include "GPU_OpenGL.h"

#include <algorithm>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "NDS.h"
#include "GPU.h"
#ifdef __ANDROID__
#include <sys/system_properties.h>
#include <stdlib.h>
#endif

namespace melonDS
{

#include "OpenGL_shaders/3DClearVS.h"
#include "OpenGL_shaders/3DClearFS.h"
#include "OpenGL_shaders/3DClearBitmapVS.h"
#include "OpenGL_shaders/3DClearBitmapFS.h"
#include "OpenGL_shaders/3DRenderVS.h"
#include "OpenGL_shaders/3DRenderFS.h"
#include "OpenGL_shaders/3DFinalPassVS.h"
#include "OpenGL_shaders/3DFinalPassEdgeFS.h"
#include "OpenGL_shaders/3DFinalPassFogFS.h"
#include "OpenGL_shaders/3DFinalPassFogFetchFS.h"

bool GLRenderer3D::BuildRenderShader(bool wbuffer)
{
    std::string wbufdef = "#define WBuffer\n";

    char shadername[32];
    snprintf(shadername, sizeof(shadername), "RenderShader%c", wbuffer?'W':'Z');

    std::string vsbuf = k3DRenderVS;
    if (wbuffer)
    {
        auto pos = vsbuf.find('\n') + 1;
        vsbuf = vsbuf.substr(0, pos) + wbufdef + vsbuf.substr(pos);
    }

    std::string fsbuf = k3DRenderFS;
    if (OpenGL::Prop("gl3dmp", 1))
        fsbuf.replace(fsbuf.find("// MEDIUMP_HERE"), 15, "precision mediump float; //");
    if (wbuffer)
    {
        auto pos = fsbuf.find('\n') + 1;
        fsbuf = fsbuf.substr(0, pos) + wbufdef + fsbuf.substr(pos);
    }

    GLuint prog;
    bool ret = OpenGL::CompileVertexFragmentProgram(prog,
        vsbuf, fsbuf,
        shadername,
        {{"vPosition", 0}, {"vColor", 1}, {"vTexcoord", 2}, {"vPolygonAttr", 3}},
        {{"oColor", 0}, {"oAttr", 1}});

    if (!ret) return false;

    GLint uni_id = glGetUniformBlockIndex(prog, "uConfig");
    glUniformBlockBinding(prog, uni_id, 0);

    glUseProgram(prog);

    uni_id = glGetUniformLocation(prog, "CurTexture");
    glUniform1i(uni_id, 0);
    uni_id = glGetUniformLocation(prog, "Capture128Texture");
    glUniform1i(uni_id, 1);
    uni_id = glGetUniformLocation(prog, "Capture256Texture");
    glUniform1i(uni_id, 2);

    RenderShader[(int)wbuffer] = prog;

    return true;
}

void GLRenderer3D::UseRenderShader(bool wbuffer)
{
    int flags = (int)wbuffer;
    if (CurShaderID == flags) return;
    glUseProgram(RenderShader[flags]);
    CurShaderID = flags;

    RenderModeULoc = glGetUniformLocation(RenderShader[flags], "uRenderMode");
}

void SetupDefaultTexParams(GLuint tex)
{
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

GLRenderer3D::GLRenderer3D(melonDS::GPU3D& gpu3D, GLRenderer* parent) noexcept :
    Renderer3D(gpu3D), Parent(parent), Texcache(gpu3D.GPU, TexcacheOpenGLLoader(false))
{
    ClearBitmap[0] = new u32[256*256];
    ClearBitmap[1] = new u32[256*256];

    ScaleFactor = 0;
    BetterPolygons = false;

    // GLRenderer3D::Init() will be used to actually initialize the renderer;
    // The various glDelete* functions silently ignore invalid IDs,
    // so we can just let the destructor clean up a half-initialized renderer.
}

bool GLRenderer3D::Init()
{
#ifdef __ANDROID__
    {
        // debug.litev.linequads=0: draw line polygons as GL_LINES (default: thin quads)
        char b[92] = {};
        LineQuads = !(__system_property_get("debug.litev.linequads", b) > 0 && atoi(b) == 0);
    }
#endif
    GLint uni_id;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);

    glDepthRange(0, 1);
    glClearDepth(1.0);

    if (!OpenGL::CompileVertexFragmentProgram(ClearShaderPlain,
            k3DClearVS, k3DClearFS,
            "ClearShaderPlain",
            {{"vPosition", 0}},
            {{"oColor", 0}, {"oAttr", 1}}))
        return false;

    ClearUniformLoc[0] = glGetUniformLocation(ClearShaderPlain, "uColor");
    ClearUniformLoc[1] = glGetUniformLocation(ClearShaderPlain, "uDepth");
    ClearUniformLoc[2] = glGetUniformLocation(ClearShaderPlain, "uOpaquePolyID");
    ClearUniformLoc[3] = glGetUniformLocation(ClearShaderPlain, "uFogFlag");

    if (!OpenGL::CompileVertexFragmentProgram(ClearShaderBitmap,
              k3DClearBitmapVS, k3DClearBitmapFS,
              "ClearShaderBitmap",
              {{"vPosition", 0}},
              {{"oColor", 0}, {"oAttr", 1}}))
        return false;

    ClearBitmapULoc[0] = glGetUniformLocation(ClearShaderBitmap, "uClearBitmapOffset");
    ClearBitmapULoc[1] = glGetUniformLocation(ClearShaderBitmap, "uOpaquePolyID");

    glUseProgram(ClearShaderBitmap);
    uni_id = glGetUniformLocation(ClearShaderBitmap, "ClearBitmapColor");
    glUniform1i(uni_id, 0);
    uni_id = glGetUniformLocation(ClearShaderBitmap, "ClearBitmapDepth");
    glUniform1i(uni_id, 1);

    memset(RenderShader, 0, sizeof(RenderShader));

    if (!BuildRenderShader(false))
        return false;

    if (!BuildRenderShader(true))
        return false;

    if (!OpenGL::CompileVertexFragmentProgram(FinalPassEdgeShader,
            k3DFinalPassVS, k3DFinalPassEdgeFS,
            "FinalPassEdgeShader",
            {{"vPosition", 0}},
            {{"oColor", 0}}))
        return false;
    if (!OpenGL::CompileVertexFragmentProgram(FinalPassFogShader,
            k3DFinalPassVS, k3DFinalPassFogFS,
            "FinalPassFogShader",
            {{"vPosition", 0}},
            {{"oColor", 0}}))
        return false;

    uni_id = glGetUniformBlockIndex(FinalPassEdgeShader, "uConfig");
    glUniformBlockBinding(FinalPassEdgeShader, uni_id, 0);

    glUseProgram(FinalPassEdgeShader);
    uni_id = glGetUniformLocation(FinalPassEdgeShader, "DepthBuffer");
    glUniform1i(uni_id, 0);
    uni_id = glGetUniformLocation(FinalPassEdgeShader, "AttrBuffer");
    glUniform1i(uni_id, 1);

#ifdef __ANDROID__
    // debug.litev.gl3dtile (default on): keep the 3D pass in tile memory - invalidate the
    // attachments before the clear and depth/stencil + attributes after the frame (never
    // loaded or written back), fog reads them by framebuffer fetch instead of as textures
    {
        char b[92] = {};
        TileMode = !(__system_property_get("debug.litev.gl3dtile", b) > 0 && atoi(b) == 0);
        bool fetch = false, fetchDS = false;
        GLint n = 0;
        glGetIntegerv(GL_NUM_EXTENSIONS, &n);
        for (GLint i = 0; i < n; i++)
        {
            const char* e = (const char*)glGetStringi(GL_EXTENSIONS, i);
            if (!e) continue;
            if (!strcmp(e, "GL_EXT_shader_framebuffer_fetch")) fetch = true;
            if (!strcmp(e, "GL_ARM_shader_framebuffer_fetch_depth_stencil")) fetchDS = true;
        }
        if (TileMode && fetch && fetchDS &&
            OpenGL::CompileVertexFragmentProgram(FinalPassFogFetchShader,
                k3DFinalPassVS, k3DFinalPassFogFetchFS, "FinalPassFogFetchShader",
                {{"vPosition", 0}}, {{"oColor", 0}, {"oAttr", 1}}))
        {
            uni_id = glGetUniformBlockIndex(FinalPassFogFetchShader, "uConfig");
            glUniformBlockBinding(FinalPassFogFetchShader, uni_id, 0);
        }
        else
            FinalPassFogFetchShader = 0;
        Platform::Log(Platform::Info, "LITEV_GL3D tile=%d fbfetch=%d/%d fogfetch=%d\n", TileMode, fetch, fetchDS, FinalPassFogFetchShader != 0);
    }
#endif

    uni_id = glGetUniformBlockIndex(FinalPassFogShader, "uConfig");
    glUniformBlockBinding(FinalPassFogShader, uni_id, 0);

    glUseProgram(FinalPassFogShader);
    uni_id = glGetUniformLocation(FinalPassFogShader, "DepthBuffer");
    glUniform1i(uni_id, 0);
    uni_id = glGetUniformLocation(FinalPassFogShader, "AttrBuffer");
    glUniform1i(uni_id, 1);


    memset(&ShaderConfig, 0, sizeof(ShaderConfig));

    glGenBuffers(1, &ShaderConfigUBO);
    glBindBuffer(GL_UNIFORM_BUFFER, ShaderConfigUBO);
    static_assert((sizeof(ShaderConfig) & 15) == 0);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(ShaderConfig), &ShaderConfig, GL_STATIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, ShaderConfigUBO);


    float clearvtx[6*2] =
    {
        -1.0, -1.0,
        1.0, 1.0,
        -1.0, 1.0,

        -1.0, -1.0,
        1.0, -1.0,
        1.0, 1.0
    };

    glGenBuffers(1, &ClearVertexBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, ClearVertexBufferID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(clearvtx), clearvtx, GL_STATIC_DRAW);

    glGenVertexArrays(1, &ClearVertexArrayID);
    glBindVertexArray(ClearVertexArrayID);
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)(0));

    // init textures for the clear bitmap
    glGenTextures(2, ClearBitmapTex);

    glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8UI, 256, 256, 0, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, nullptr);

    glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, 256, 256, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);


    glGenBuffers(1, &VertexBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, VertexBufferID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexBuffer), nullptr, GL_DYNAMIC_DRAW);

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    glEnableVertexAttribArray(0); // position
    glVertexAttribIPointer(0, 4, GL_UNSIGNED_SHORT, 7*4, (void*)(0));
    glEnableVertexAttribArray(1); // color
    glVertexAttribIPointer(1, 4, GL_UNSIGNED_BYTE, 7*4, (void*)(2*4));
    glEnableVertexAttribArray(2); // texcoords
    glVertexAttribIPointer(2, 2, GL_SHORT, 7*4, (void*)(3*4));
    glEnableVertexAttribArray(3); // attrib
    // NB: the shader declares vPolygonAttr as a *signed* ivec3, and the texcache
    // encodes the "normal texture" sentinel as 0xFFFF0000 in vPolygonAttr.y
    // (bit 31 set). Feeding that via GL_UNSIGNED_INT into a signed attribute is a
    // signedness mismatch: desktop GL delivers the raw bits, but the Mali-G52
    // GLES driver mangles values >= 2^31, collapsing the low 16 bits to 0xFFFF so
    // every polygon is misrouted as untextured (white/untextured 3D models).
    // Match the attribute type to the signed shader declaration.
    glVertexAttribIPointer(3, 3, GL_INT, 7*4, (void*)(4*4));

    glGenBuffers(1, &IndexBufferID);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferID);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(IndexBuffer), nullptr, GL_DYNAMIC_DRAW);

    glGenFramebuffers(1, &MainFramebuffer);

    // color buffers
    glGenTextures(MaxColorRing, ColorBufferTex);
    for (int i = 0; i < MaxColorRing; i++)
        SetupDefaultTexParams(ColorBufferTex[i]);

    // depth/stencil buffer
    glGenTextures(1, &DepthBufferTex);
    SetupDefaultTexParams(DepthBufferTex);

    // attribute buffer
    // R: opaque polyID (for edgemarking)
    // G: edge flag
    // B: fog flag
    glGenTextures(1, &AttrBufferTex);
    SetupDefaultTexParams(AttrBufferTex);

    if (Parent) Parent->OutputTex3D = ColorBufferTex[0];

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_MAX);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    static const GLint modes[3] = {GL_CLAMP_TO_EDGE, GL_REPEAT, GL_MIRRORED_REPEAT};
    glGenSamplers(9, WrapSampler);
    for (int i = 0; i < 9; i++)
    {
        glSamplerParameteri(WrapSampler[i], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glSamplerParameteri(WrapSampler[i], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glSamplerParameteri(WrapSampler[i], GL_TEXTURE_WRAP_S, modes[i / 3]);
        glSamplerParameteri(WrapSampler[i], GL_TEXTURE_WRAP_T, modes[i % 3]);
    }

    return true;
}

GLRenderer3D::~GLRenderer3D()
{
    assert(glDeleteTextures != nullptr);

    Texcache.Reset();

    if (FinalPassFogFetchShader) glDeleteProgram(FinalPassFogFetchShader);
    glDeleteFramebuffers(1, &MainFramebuffer);
    glDeleteSamplers(9, WrapSampler);
    glDeleteTextures(MaxColorRing, ColorBufferTex);
    glDeleteTextures(1, &DepthBufferTex);
    glDeleteTextures(1, &AttrBufferTex);

    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteBuffers(1, &VertexBufferID);
    glDeleteVertexArrays(1, &ClearVertexArrayID);
    glDeleteBuffers(1, &ClearVertexBufferID);
    glDeleteTextures(2, ClearBitmapTex);
    delete[] ClearBitmap[0];
    delete[] ClearBitmap[1];

    glDeleteBuffers(1, &ShaderConfigUBO);

    for (int i = 0; i < 2; i++)
    {
        if (!RenderShader[i]) continue;
        glDeleteProgram(RenderShader[i]);
    }
}

void GLRenderer3D::Reset()
{
    Texcache.Reset();
    ClearBitmapDirty = 0x3;
}

void GLRenderer3D::SetBetterPolygons(bool betterpolygons) noexcept
{
    SetRenderSettings(ScaleFactor, betterpolygons);
}

void GLRenderer3D::SetScaleFactor(int scale) noexcept
{
    SetRenderSettings(scale, BetterPolygons);
}


void GLRenderer3D::SetRenderSettings(int scale, bool betterpolygons) noexcept
{
    if (betterpolygons == BetterPolygons && scale == ScaleFactor)
        return;

    // TODO set it for 2D renderer
    //CurGLCompositor.SetScaleFactor(scale);
    ScaleFactor = scale;
    BetterPolygons = betterpolygons;

    ScreenW = 256 * scale;
    ScreenH = 192 * scale;

    AllocColorBuffers();

    glBindTexture(GL_TEXTURE_2D, DepthBufferTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, ScreenW, ScreenH, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, NULL);
    glBindTexture(GL_TEXTURE_2D, AttrBufferTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ScreenW, ScreenH, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    GLenum fbassign[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};

    glBindFramebuffer(GL_FRAMEBUFFER, MainFramebuffer);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ColorBufferTex[CurColor], 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, DepthBufferTex, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, AttrBufferTex, 0);
    glDrawBuffers(2, fbassign);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    //glLineWidth(scale);
    //glLineWidth(1.5);
}


void GLRenderer3D::AllocColorBuffers() noexcept
{
    for (int i = 0; i < ColorRing; i++)
    {
        glBindTexture(GL_TEXTURE_2D, ColorBufferTex[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ScreenW, ScreenH, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    }
}

void GLRenderer3D::SetColorRing(int n) noexcept
{
    ColorRing = std::clamp(n, 1, MaxColorRing);
    CurColor = 0;
    if (ScaleFactor) AllocColorBuffers();
}

void GLRenderer3D::SetupPolygon(GLRenderer3D::RendererPolygon* rp, Polygon* polygon) const
{
    rp->PolyData = polygon;

    // render key: depending on what we're drawing
    // opaque polygons:
    // - depthfunc
    // -- alpha=0
    // regular translucent polygons:
    // - depthfunc
    // -- depthwrite
    // --- polyID
    // ---- need opaque
    // shadow mask polygons:
    // - depthfunc?????
    // shadow polygons:
    // - depthfunc
    // -- depthwrite
    // --- polyID

    rp->RenderKey = (polygon->Attr >> 14) & 0x1; // bit14 - depth func
    if (!polygon->IsShadowMask)
    {
        if (polygon->Translucent)
        {
            if (polygon->IsShadow) rp->RenderKey |= 0x20000;
            else                   rp->RenderKey |= 0x10000;
            rp->RenderKey |= (polygon->Attr >> 10) & 0x2; // bit11 - depth write
            rp->RenderKey |= (polygon->Attr >> 13) & 0x4; // bit15 - fog
            rp->RenderKey |= (polygon->Attr & 0x3F000000) >> 16; // polygon ID
            if ((polygon->Attr & 0x001F0000) == 0x001F0000) // need opaque
                rp->RenderKey |= 0x4000;
        }
        else
        {
            if ((polygon->Attr & 0x001F0000) == 0)
                rp->RenderKey |= 0x2;
            rp->RenderKey |= (polygon->Attr & 0x3F000000) >> 16; // polygon ID
        }
    }
    else
    {
        rp->RenderKey |= 0x30000;
    }

    u32 textype = (polygon->TexParam >> 26) & 0x7;
    u32 texattr = (polygon->TexParam >> 16) & 0x3FF;
    if (TexEnable && (textype != 0))
        rp->RenderKey |= (0x80000 | (texattr << 20));
}

u32* GLRenderer3D::SetupVertex(const Polygon* poly, int vid, const Vertex* vtx, u32 vtxattr, u32 texlayer, u32* vptr) const
{
    u32 z = poly->FinalZ[vid];
    u32 w = poly->FinalW[vid];

    u32 alpha = (poly->Attr >> 16) & 0x1F;

    // Z should always fit within 16 bits, so it's okay to do this
    u32 zshift = 0;
    while (z > 0xFFFF) { z >>= 1; zshift++; }

    u32 x, y;
    if (ScaleFactor > 1)
    {
        x = (vtx->HiresPosition[0] * ScaleFactor) >> 4;
        y = (vtx->HiresPosition[1] * ScaleFactor) >> 4;
    }
    else
    {
        x = vtx->FinalPosition[0];
        y = vtx->FinalPosition[1];
    }

    // correct nearly-vertical edges that would look vertical on the DS
    /*{
        int vtopid = vid - 1;
        if (vtopid < 0) vtopid = poly->NumVertices-1;
        Vertex* vtop = poly->Vertices[vtopid];
        if (vtop->FinalPosition[1] >= vtx->FinalPosition[1])
        {
            vtopid = vid + 1;
            if (vtopid >= poly->NumVertices) vtopid = 0;
            vtop = poly->Vertices[vtopid];
        }
        if ((vtop->FinalPosition[1] < vtx->FinalPosition[1]) &&
            (vtx->FinalPosition[0] == vtop->FinalPosition[0]-1))
        {
            if (ScaleFactor > 1)
                x = (vtop->HiresPosition[0] * ScaleFactor) >> 4;
            else
                x = vtop->FinalPosition[0];
        }
    }*/

    *vptr++ = x | (y << 16);
    *vptr++ = z | (w << 16);

    *vptr++ =  (vtx->FinalColor[0] >> 1) |
              ((vtx->FinalColor[1] >> 1) << 8) |
              ((vtx->FinalColor[2] >> 1) << 16) |
              (alpha << 24);

    *vptr++ = (u16)vtx->TexCoords[0] | ((u16)vtx->TexCoords[1] << 16);

    *vptr++ = vtxattr | (zshift << 16);
    *vptr++ = texlayer;
    *vptr++ = TextureWidth(poly->TexParam) | (TextureHeight(poly->TexParam) << 16);

    return vptr;
}

void GLRenderer3D::BuildPolygons(GLRenderer3D::RendererPolygon* polygons, int npolys, int captureinfo[16])
{
    u32* vptr = &VertexBuffer[0];
    u32 vidx = 0;

    u32 iidx = 0;
    u32 eidx = EdgeIndicesOffset;

    u32 curtexparam = 0;
    u32 curtexpal = 0;
    GLuint curtexid = 0;
    u32 curtexlayer = (u32)-1;

    for (int i = 0; i < npolys; i++)
    {
        RendererPolygon* rp = &polygons[i];
        Polygon* poly = rp->PolyData;

        rp->IndicesOffset = iidx;
        rp->NumIndices = 0;

        u32 vidx_first = vidx;

        u32 polyattr = poly->Attr;
        u32 texparam = poly->TexParam & ~0xC00F0000;
        u32 texpal = poly->TexPalette;

        u32 alpha = (polyattr >> 16) & 0x1F;

        u32 vtxattr = polyattr & 0x1F00C8F0;
        if (poly->FacingView) vtxattr |= (1<<8);
        if (poly->WBuffer)    vtxattr |= (1<<9);

        if ((texparam != curtexparam) || (texpal != curtexpal))
        {
            u32 textype = (texparam >> 26) & 0x7;
            if (TexEnable && (textype != 0))
            {
                // figure out which texture this polygon is going to use

                u32 texaddr = texparam & 0xFFFF;
                u32 texwidth = TextureWidth(texparam);
                u32 texheight = TextureHeight(texparam);
                int capblock = -1;
                if ((textype == 7) && ((texwidth == 128) || (texwidth == 256)))
                {
                    // if this is a direct color texture, and the width is 128 or 256
                    // then it might be a display capture
                    u32 startaddr = texaddr << 3;
                    u32 endaddr = startaddr + (texheight * texwidth * 2);

                    startaddr >>= 15;
                    endaddr = (endaddr + 0x7FFF) >> 15;

                    for (u32 b = startaddr; b < endaddr; b++)
                    {
                        int blk = captureinfo[b];
                        if (blk == -1) continue;

                        capblock = blk;
                    }
                }

                if (capblock != -1)
                {
                    if (texwidth == 128)
                    {
                        curtexid = -1;
                        curtexlayer = capblock | (((texaddr >> 5) & 0x7F) << 20);
                    }
                    else
                    {
                        curtexid = -2;
                        curtexlayer = (capblock >> 2) | (((texaddr >> 6) & 0xFF) << 20);
                    }
                }
                else
                {
                    u32* halp;
                    Texcache.GetTexture(texparam, texpal, curtexid, curtexlayer, halp);
                    curtexlayer |= 0xFFFF0000;
                }
            }
            else
            {
                // no texture
                curtexid = 0;
                curtexlayer = (u32)-1;
            }

            curtexparam = texparam;
            curtexpal = texpal;
        }

        rp->TexID = curtexid;
        rp->TexRepeat = (poly->TexParam >> 16) & 0xF;

        // assemble vertices
        if (poly->Type == 1) // line
        {
            rp->PrimType = GL_LINES;

            u32 lastx, lasty;
            int nout = 0;
            for (u32 j = 0; j < poly->NumVertices; j++)
            {
                Vertex* vtx = poly->Vertices[j];

                if (j > 0)
                {
                    if (lastx == vtx->FinalPosition[0] &&
                        lasty == vtx->FinalPosition[1]) continue;
                }

                lastx = vtx->FinalPosition[0];
                lasty = vtx->FinalPosition[1];

                vptr = SetupVertex(poly, j, vtx, vtxattr, curtexlayer, vptr);

                IndexBuffer[iidx++] = vidx;
                rp->NumIndices++;

                vidx++;
                nout++;
                if (nout >= 2) break;
            }

            if (LineQuads)
            {
                // Draw the line as a thin quad, ScaleFactor pixels wide (the DS line width at
                // this resolution), so it batches with the triangle draws around it: offset
                // a copy of both ends along the minor axis. A 1-vertex line draws nothing,
                // as with GL_LINES.
                rp->PrimType = GL_TRIANGLES;
                iidx -= rp->NumIndices;
                rp->NumIndices = 0;
                if (nout == 2)
                {
                    u32* v0 = vptr - 14;
                    u32* v1 = vptr - 7;
                    int dx = (int)(v1[0] & 0xFFFF) - (int)(v0[0] & 0xFFFF);
                    int dy = (int)(v1[0] >> 16) - (int)(v0[0] >> 16);
                    const u32 w = ScaleFactor > 1 ? ScaleFactor : 1;
                    const u32 off = (std::abs(dx) >= std::abs(dy)) ? (w << 16) : w;
                    memcpy(vptr, v0, 7 * 4); vptr[0] += off; vptr += 7;
                    memcpy(vptr, v1, 7 * 4); vptr[0] += off; vptr += 7;
                    const u32 a = vidx - 2, b = vidx - 1, a2 = vidx, b2 = vidx + 1;
                    vidx += 2;
                    IndexBuffer[iidx++] = a; IndexBuffer[iidx++] = b; IndexBuffer[iidx++] = b2;
                    IndexBuffer[iidx++] = a; IndexBuffer[iidx++] = b2; IndexBuffer[iidx++] = a2;
                    rp->NumIndices = 6;
                }
            }
        }
        else if (poly->NumVertices == 3) // regular triangle
        {
            rp->PrimType = GL_TRIANGLES;

            for (int j = 0; j < 3; j++)
            {
                Vertex* vtx = poly->Vertices[j];

                vptr = SetupVertex(poly, j, vtx, vtxattr, curtexlayer, vptr);
                vidx++;
            }

            // build a triangle
            IndexBuffer[iidx++] = vidx_first;
            IndexBuffer[iidx++] = vidx - 2;
            IndexBuffer[iidx++] = vidx - 1;
            rp->NumIndices += 3;
        }
        else // quad, pentagon, etc
        {
            rp->PrimType = GL_TRIANGLES;

            if (!BetterPolygons)
            {
                // regular triangle-splitting

                for (u32 j = 0; j < poly->NumVertices; j++)
                {
                    Vertex* vtx = poly->Vertices[j];

                    vptr = SetupVertex(poly, j, vtx, vtxattr, curtexlayer, vptr);

                    if (j >= 2)
                    {
                        // build a triangle
                        IndexBuffer[iidx++] = vidx_first;
                        IndexBuffer[iidx++] = vidx - 1;
                        IndexBuffer[iidx++] = vidx;
                        rp->NumIndices += 3;
                    }

                    vidx++;
                }
            }
            else
            {
                // attempt at 'better' splitting
                // this doesn't get rid of the error while splitting a bigger polygon into triangles
                // but we can attempt to reduce it

                u32 cX = 0, cY = 0;
                float cZ = 0;
                float cW = 0;

                float cR = 0, cG = 0, cB = 0;
                float cS = 0, cT = 0;

                for (u32 j = 0; j < poly->NumVertices; j++)
                {
                    Vertex* vtx = poly->Vertices[j];

                    cX += vtx->HiresPosition[0];
                    cY += vtx->HiresPosition[1];

                    float fw = (float)poly->FinalW[j] * poly->NumVertices;
                    cW += 1.0f / fw;

                    if (poly->WBuffer) cZ += poly->FinalZ[j] / fw;
                    else               cZ += poly->FinalZ[j];

                    cR += (vtx->FinalColor[0] >> 1) / fw;
                    cG += (vtx->FinalColor[1] >> 1) / fw;
                    cB += (vtx->FinalColor[2] >> 1) / fw;

                    cS += vtx->TexCoords[0] / fw;
                    cT += vtx->TexCoords[1] / fw;
                }

                cX /= poly->NumVertices;
                cY /= poly->NumVertices;

                cW = 1.0f / cW;

                if (poly->WBuffer) cZ *= cW;
                else               cZ /= poly->NumVertices;

                cR *= cW;
                cG *= cW;
                cB *= cW;

                cS *= cW;
                cT *= cW;

                cX = (cX * ScaleFactor) >> 4;
                cY = (cY * ScaleFactor) >> 4;

                u32 w = (u32)cW;

                u32 z = (u32)cZ;
                u32 zshift = 0;
                while (z > 0xFFFF) { z >>= 1; zshift++; }

                // build center vertex
                *vptr++ = cX | (cY << 16);
                *vptr++ = z | (w << 16);

                *vptr++ =  (u32)cR |
                          ((u32)cG << 8) |
                          ((u32)cB << 16) |
                          (alpha << 24);

                *vptr++ = (u16)cS | ((u16)cT << 16);

                *vptr++ = vtxattr | (zshift << 16);
                *vptr++ = curtexlayer;
                *vptr++ = TextureWidth(texparam) | (TextureHeight(texparam) << 16);

                vidx++;

                // build the final polygon
                for (u32 j = 0; j < poly->NumVertices; j++)
                {
                    Vertex* vtx = poly->Vertices[j];

                    vptr = SetupVertex(poly, j, vtx, vtxattr, curtexlayer, vptr);

                    if (j >= 1)
                    {
                        // build a triangle
                        IndexBuffer[iidx++] = vidx_first;
                        IndexBuffer[iidx++] = vidx - 1;
                        IndexBuffer[iidx++] = vidx;
                        rp->NumIndices += 3;
                    }

                    vidx++;
                }

                IndexBuffer[iidx++] = vidx_first;
                IndexBuffer[iidx++] = vidx - 1;
                IndexBuffer[iidx++] = vidx_first + 1;
                rp->NumIndices += 3;
            }
        }

        rp->EdgeIndicesOffset = eidx;
        rp->NumEdgeIndices = 0;

        if (poly->Type == 1)
        {
            // a line writes only its two distinct ends (plus their offset copies as a quad), not
            // NumVertices vertices: edges over NumVertices reached into the next polygon's
            // vertices and drew a long stray edge (Pokemon White's town intro)
            if (vidx - vidx_first >= 2)
            {
                IndexBuffer[eidx++] = vidx_first;
                IndexBuffer[eidx++] = vidx_first + 1;
                rp->NumEdgeIndices = 2;
            }
            continue;
        }

        u32 vidx_cur = vidx_first;
        for (u32 j = 1; j < poly->NumVertices; j++)
        {
            IndexBuffer[eidx++] = vidx_cur;
            IndexBuffer[eidx++] = vidx_cur + 1;
            vidx_cur++;
            rp->NumEdgeIndices += 2;
        }
        IndexBuffer[eidx++] = vidx_cur;
        IndexBuffer[eidx++] = vidx_first;
        rp->NumEdgeIndices += 2;
    }

    NumVertices = vidx;
    NumIndices = iidx;
    NumEdgeIndices = eidx - EdgeIndicesOffset;
}

// ---- redundant GL state filter (SetupPolygonTexture .. RenderSceneChunk) ----
// The scene issues ~300 draws per frame, each preceded by depth/stencil/mask/texture state
// calls that are mostly unchanged; on Mali most of the GL thread's CPU time is driver work.
// Skip calls that would not change the state. Reset at the start of RenderSceneChunk
// (other code on the same context may have changed it).
namespace
{
struct GLStateCache
{
    GLenum DepthFunc; GLenum StencilFunc; GLint StencilRef; GLuint StencilFuncMask;
    GLenum SFail, DpFail, DpPass; GLuint StencilMask; GLboolean DepthMask;
    GLboolean ColorMask[2][4]; GLenum ActiveTex; GLuint Tex0; GLuint Sampler0;
    bool Valid;
};
GLStateCache SC {};
inline void SC_DepthFunc(GLenum f) { if (!SC.Valid || SC.DepthFunc != f) { SC.DepthFunc = f; glDepthFunc(f); } }
inline void SC_StencilFunc(GLenum f, GLint r, GLuint m)
{
    if (!SC.Valid || SC.StencilFunc != f || SC.StencilRef != r || SC.StencilFuncMask != m)
    { SC.StencilFunc = f; SC.StencilRef = r; SC.StencilFuncMask = m; glStencilFunc(f, r, m); }
}
inline void SC_StencilOp(GLenum a, GLenum b, GLenum c)
{
    if (!SC.Valid || SC.SFail != a || SC.DpFail != b || SC.DpPass != c)
    { SC.SFail = a; SC.DpFail = b; SC.DpPass = c; glStencilOp(a, b, c); }
}
inline void SC_StencilMask(GLuint m) { if (!SC.Valid || SC.StencilMask != m) { SC.StencilMask = m; glStencilMask(m); } }
inline void SC_DepthMask(GLboolean m) { if (!SC.Valid || SC.DepthMask != m) { SC.DepthMask = m; glDepthMask(m); } }
inline void SC_ColorMaski(GLuint i, GLboolean r, GLboolean g, GLboolean b, GLboolean a)
{
    GLboolean* c = SC.ColorMask[i & 1];
    if (!SC.Valid || i > 1 || c[0] != r || c[1] != g || c[2] != b || c[3] != a)
    { c[0] = r; c[1] = g; c[2] = b; c[3] = a; glColorMaski(i, r, g, b, a); }
}
inline void SC_ActiveTexture(GLenum t) { if (!SC.Valid || SC.ActiveTex != t) { SC.ActiveTex = t; glActiveTexture(t); } }
inline void SC_BindTexture(GLenum target, GLuint t)
{
    if (SC.Valid && SC.ActiveTex == GL_TEXTURE0 && target == GL_TEXTURE_2D_ARRAY)
    { if (SC.Tex0 == t) return; SC.Tex0 = t; }
    else if (SC.ActiveTex == GL_TEXTURE0) SC.Tex0 = ~0u;
    glBindTexture(target, t);
}
inline void SC_BindSampler(GLuint unit, GLuint s)
{
    if (unit == 0) { if (SC.Valid && SC.Sampler0 == s) return; SC.Sampler0 = s; }
    glBindSampler(unit, s);
}
inline void SC_Reset()
{
    // impossible values: the first call of each kind always goes through
    SC.DepthFunc = SC.StencilFunc = SC.SFail = SC.DpFail = SC.DpPass = SC.ActiveTex = ~0u;
    SC.StencilRef = -1; SC.StencilFuncMask = SC.StencilMask = ~0u - 1;
    SC.DepthMask = 2;
    for (auto& m : SC.ColorMask) for (auto& c : m) c = 2;
    SC.Tex0 = SC.Sampler0 = ~0u;
    SC.Valid = true;
}
}
#pragma push_macro("glDepthFunc")
#undef glDepthFunc
#define glDepthFunc SC_DepthFunc
#pragma push_macro("glStencilFunc")
#undef glStencilFunc
#define glStencilFunc SC_StencilFunc
#pragma push_macro("glStencilOp")
#undef glStencilOp
#define glStencilOp SC_StencilOp
#pragma push_macro("glStencilMask")
#undef glStencilMask
#define glStencilMask SC_StencilMask
#pragma push_macro("glDepthMask")
#undef glDepthMask
#define glDepthMask SC_DepthMask
#pragma push_macro("glColorMaski")
#undef glColorMaski
#define glColorMaski SC_ColorMaski
#pragma push_macro("glActiveTexture")
#undef glActiveTexture
#define glActiveTexture SC_ActiveTexture
#pragma push_macro("glBindTexture")
#undef glBindTexture
#define glBindTexture SC_BindTexture
#pragma push_macro("glBindSampler")
#undef glBindSampler
#define glBindSampler SC_BindSampler

void GLRenderer3D::SetupPolygonTexture(const RendererPolygon* poly) const
{
    bool iscap = (poly->TexID == (GLuint)-1 || poly->TexID == (GLuint)-2);

    // wrap mode per axis: 0 clamp, 1 repeat, 2 mirrored repeat
    const int ws = (poly->TexRepeat & (1<<0)) ? ((poly->TexRepeat & (1<<2)) ? 2 : 1) : 0;
    const int wt = (poly->TexRepeat & (1<<1)) ? ((poly->TexRepeat & (1<<3)) ? 2 : 1) : 0;

    if (!iscap)
    {
        // Sampler objects instead of glTexParameteri on the (shared) texture array: a
        // texture-state change per draw is a costly revalidation in the Mali driver.
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D_ARRAY, poly->TexID);
        glBindSampler(0, WrapSampler[ws * 3 + wt]);
        return;
    }

    if (poly->TexID == (GLuint)-1)
        glActiveTexture(GL_TEXTURE1);
    else
        glActiveTexture(GL_TEXTURE2);

    static const GLint modes[3] = {GL_CLAMP_TO_EDGE, GL_REPEAT, GL_MIRRORED_REPEAT};
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, modes[ws]);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, modes[wt]);
}

int GLRenderer3D::RenderSinglePolygon(int i) const
{
    const RendererPolygon* rp = &PolygonList[i];

    SetupPolygonTexture(rp);
    glDrawElements(rp->PrimType, rp->NumIndices, GL_UNSIGNED_SHORT, (void*)(uintptr_t)(rp->IndicesOffset * 2));
    StatDraws++;

    return 1;
}

int GLRenderer3D::RenderPolygonBatch(int i) const
{
    const RendererPolygon* rp = &PolygonList[i];
    GLuint primtype = rp->PrimType;
    u32 renderkey = rp->RenderKey;
    GLuint texid = rp->TexID;
    u32 texrepeat = rp->TexRepeat;
    int numpolys = 0;
    u32 numindices = 0;

    for (int iend = i; iend < NumFinalPolys; iend++)
    {
        const RendererPolygon* cur_rp = &PolygonList[iend];
        if (cur_rp->PrimType != primtype) break;
        if (cur_rp->RenderKey != renderkey) break;
        if (cur_rp->TexID != texid) break;
        if (cur_rp->TexRepeat != texrepeat) break;

        numpolys++;
        numindices += cur_rp->NumIndices;
    }

    SetupPolygonTexture(rp);
    glDrawElements(primtype, numindices, GL_UNSIGNED_SHORT, (void*)(uintptr_t)(rp->IndicesOffset * 2));
    StatDraws++;
    return numpolys;
}

int GLRenderer3D::RenderPolygonEdgeBatch(int i) const
{
    const RendererPolygon* rp = &PolygonList[i];
    u32 renderkey = rp->RenderKey;
    GLuint texid = rp->TexID;
    u32 texrepeat = rp->TexRepeat;
    int numpolys = 0;
    u32 numindices = 0;

    for (int iend = i; iend < NumFinalPolys; iend++)
    {
        const RendererPolygon* cur_rp = &PolygonList[iend];
        if (cur_rp->RenderKey != renderkey) break;
        if (cur_rp->TexID != texid) break;
        if (cur_rp->TexRepeat != texrepeat) break;

        numpolys++;
        numindices += cur_rp->NumEdgeIndices;
    }

    SetupPolygonTexture(rp);
    glDrawElements(GL_LINES, numindices, GL_UNSIGNED_SHORT, (void*)(uintptr_t)(rp->EdgeIndicesOffset * 2));
    return numpolys;
}

void GLRenderer3D::RenderSceneChunk(int y, int h)
{
    SC_Reset();
    bool flags = S.RenderPolygonRAM[0]->WBuffer;
    UseRenderShader(flags);

    //if (h != 192) glScissor(0, y<<ScaleFactor, 256<<ScaleFactor, h<<ScaleFactor);

    GLboolean fogenable = (S.RenderDispCnt & (1<<7)) ? GL_TRUE : GL_FALSE;

    // TODO: proper 'equal' depth test!
    // (has margin of +-0x200 in Z-buffer mode, +-0xFF in W-buffer mode)
    // for now we're using GL_LEQUAL to make it work to some extent

    // STENCIL BUFFER VALUES
    // 1111'1111 : background (clear plane)
    // 1111'1110 : shadow mask against background
    // 00pp'pppp : opaque polygon ID p
    // 01pp'pppp : translucent polygon ID p
    // 1___'____ : shadow mask

    // POLYGON ID BASED RENDERING RULES
    // opaque polygons: polygon ID ignored
    // translucent polygons: if dst is opaque
    //                       OR if dst is translucent and polygon ID is different
    // shadow polygons: if (dst is opaque AND dst opaque polygon ID is different)
    //                  OR (dst is translucent and dst translucent polygon ID is different)

    // pass 1: opaque pixels

    glUniform1i(RenderModeULoc, RenderMode_Opaque);
    glLineWidth(1.0);

    glColorMaski(1, GL_TRUE, GL_TRUE, fogenable, GL_FALSE);

    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    glBindVertexArray(VertexArrayID);

    OpenGL::GLStatAdd(OpenGL::GLStat3D);
    if (flags) OpenGL::GLStatAdd(OpenGL::GLStatWBuf);
    if (OpenGL::GLSkip() & 1) goto polygons_done;

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D_ARRAY, Parent ? Parent->CaptureOutput128Tex : 0);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D_ARRAY, Parent ? Parent->CaptureOutput256Tex : 0);

    glActiveTexture(GL_TEXTURE0);

    for (int i = 0; i < NumFinalPolys; )
    {
        RendererPolygon* rp = &PolygonList[i];

        if (rp->PolyData->IsShadowMask) { i++; continue; }
        if (rp->PolyData->Translucent) { i++; continue; }

        if (rp->PolyData->Attr & (1<<14))
            glDepthFunc(GL_LEQUAL);
        else
            glDepthFunc(GL_LESS);

        u32 polyattr = rp->PolyData->Attr;
        u32 polyid = (polyattr >> 24) & 0x3F;

        glStencilFunc(GL_ALWAYS, polyid, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);

        i += RenderPolygonBatch(i);
    }

    // if edge marking is enabled, mark all opaque edges
    // TODO BETTER EDGE MARKING!!! THIS SUCKS
    /*if (RenderDispCnt & (1<<5))
    {
        UseRenderShader(flags | RenderFlag_Edge);
        glLineWidth(1.5);

        glColorMaski(0, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glColorMaski(1, GL_FALSE, GL_TRUE, GL_FALSE, GL_FALSE);

        glDepthFunc(GL_ALWAYS);
        glDepthMask(GL_FALSE);

        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        glStencilMask(0);

        for (int i = 0; i < NumFinalPolys; )
        {
            RendererPolygon* rp = &PolygonList[i];

            if (rp->PolyData->IsShadowMask) { i++; continue; }

            i += RenderPolygonEdgeBatch(i);
        }

        glDepthMask(GL_TRUE);
    }*/

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_MAX);

    if (S.RenderDispCnt & (1<<3))
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
    else
        glBlendFuncSeparate(GL_ONE, GL_ZERO, GL_ONE, GL_ONE);

    glLineWidth(1.0);

    if (NumOpaqueFinalPolys > -1)
    {
        // pass 2: if needed, render translucent pixels that are against background pixels
        // when background alpha is zero, those need to be rendered with blending disabled

        if ((S.RenderClearAttr1 & 0x001F0000) == 0)
        {
            glDisable(GL_BLEND);

            for (int i = 0; i < NumFinalPolys; )
            {
                RendererPolygon* rp = &PolygonList[i];

                if (rp->PolyData->IsShadowMask)
                {
                    // draw actual shadow mask

                    glUniform1i(RenderModeULoc, RenderMode_ShadowMask);

                    glDisable(GL_BLEND);
                    glColorMaski(0, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glColorMaski(1, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glDepthMask(GL_FALSE);

                    // render where stencil is 0xFF
                    // set to 0xFE where this polygon z-fails
                    glDepthFunc(GL_LESS);
                    glStencilFunc(GL_EQUAL, 0xFF, 0xFF);
                    glStencilOp(GL_KEEP, GL_INVERT, GL_KEEP);
                    glStencilMask(0x01);

                    i += RenderPolygonBatch(i);
                }
                else if (rp->PolyData->Translucent)
                {
                    bool needopaque = ((rp->PolyData->Attr & 0x001F0000) == 0x001F0000);

                    u32 polyattr = rp->PolyData->Attr;
                    u32 polyid = (polyattr >> 24) & 0x3F;

                    if (polyattr & (1<<14))
                        glDepthFunc(GL_LEQUAL);
                    else
                        glDepthFunc(GL_LESS);

                    if (needopaque)
                    {
                        glUniform1i(RenderModeULoc, RenderMode_Opaque);

                        glDisable(GL_BLEND);
                        glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                        glColorMaski(1, GL_TRUE, GL_TRUE, fogenable, GL_FALSE);

                        // set stencil to the polygon's ID
                        glStencilFunc(GL_ALWAYS, polyid, 0xFF);
                        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
                        glStencilMask(0xFF);

                        glDepthMask(GL_TRUE);

                        RenderSinglePolygon(i);
                    }

                    glUniform1i(RenderModeULoc, RenderMode_Translucent);

                    GLboolean transfog;
                    if (!(polyattr & (1<<15))) transfog = fogenable;
                    else                       transfog = GL_FALSE;

                    if (rp->PolyData->IsShadow)
                    {
                        // shadow against clear-plane will only pass if its polyID matches that of the clear plane
                        u32 clrpolyid = (S.RenderClearAttr1 >> 24) & 0x3F;
                        if (polyid != clrpolyid) { i++; continue; }

                        glEnable(GL_BLEND);
                        glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                        glColorMaski(1, GL_FALSE, GL_FALSE, transfog, GL_FALSE);

                        // draw where shadow mask has previously been rendered (stencil=0xFE)
                        // when passing, set it to (polyID | 0x40)
                        // TODO might break bit0 of polyID
                        glStencilFunc(GL_EQUAL, 0xFE, 0xFF);
                        glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);
                        glStencilMask(~(0x40|polyid)); // heheh

                        if (polyattr & (1<<11)) glDepthMask(GL_TRUE);
                        else                    glDepthMask(GL_FALSE);

                        i += needopaque ? RenderSinglePolygon(i) : RenderPolygonBatch(i);
                    }
                    else
                    {
                        glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                        glColorMaski(1, GL_FALSE, GL_FALSE, transfog, GL_FALSE);

                        // draw on either background (0xFF) or shadowmask (0xFE)
                        // when passing, set it to (polyID | 0x40)
                        glStencilFunc(GL_EQUAL, 0xFF, 0xFE);
                        glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);
                        glStencilMask(~(0x40|polyid)); // heheh

                        if (polyattr & (1<<11)) glDepthMask(GL_TRUE);
                        else                    glDepthMask(GL_FALSE);

                        i += needopaque ? RenderSinglePolygon(i) : RenderPolygonBatch(i);
                    }
                }
                else
                    i++;
            }

            glEnable(GL_BLEND);
            glStencilMask(0xFF);
        }

        // pass 3: translucent pixels

        for (int i = 0; i < NumFinalPolys; )
        {
            RendererPolygon* rp = &PolygonList[i];

            if (rp->PolyData->IsShadowMask)
            {
                // clear shadow bits in stencil buffer
                OpenGL::GLStatAdd(OpenGL::GLStatShadow);

                glStencilMask(0x80);
                glClear(GL_STENCIL_BUFFER_BIT);

                // draw actual shadow mask

                glUniform1i(RenderModeULoc, RenderMode_ShadowMask);

                glDisable(GL_BLEND);
                glColorMaski(0, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                glColorMaski(1, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                glDepthMask(GL_FALSE);

                // set stencil bit7 where the shadowmask z-fails
                glDepthFunc(GL_LESS);
                glStencilFunc(GL_ALWAYS, 0x80, 0x80);
                glStencilOp(GL_KEEP, GL_REPLACE, GL_KEEP);

                i += RenderPolygonBatch(i);
            }
            else if (rp->PolyData->Translucent)
            {
                bool needopaque = ((rp->PolyData->Attr & 0x001F0000) == 0x001F0000);

                u32 polyattr = rp->PolyData->Attr;
                u32 polyid = (polyattr >> 24) & 0x3F;

                if (polyattr & (1<<14))
                    glDepthFunc(GL_LEQUAL);
                else
                    glDepthFunc(GL_LESS);

                if (needopaque)
                {
                    glUniform1i(RenderModeULoc, RenderMode_Opaque);

                    glDisable(GL_BLEND);
                    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glColorMaski(1, GL_TRUE, GL_TRUE, fogenable, GL_FALSE);

                    // set stencil to polyID
                    glStencilFunc(GL_ALWAYS, polyid, 0xFF);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
                    glStencilMask(0xFF);

                    glDepthMask(GL_TRUE);

                    RenderSinglePolygon(i);
                }

                glUniform1i(RenderModeULoc, RenderMode_Translucent);

                GLboolean transfog;
                if (!(polyattr & (1<<15))) transfog = fogenable;
                else                       transfog = GL_FALSE;

                if (rp->PolyData->IsShadow)
                {
                    glDisable(GL_BLEND);
                    glColorMaski(0, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glColorMaski(1, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glDepthMask(GL_FALSE);

                    // render where polyID matches (ignoring other bits)
                    // clear bit7 where it passes
                    glStencilFunc(GL_EQUAL, polyid, 0x3F);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_ZERO);
                    glStencilMask(0x80);

                    RenderSinglePolygon(i);

                    glEnable(GL_BLEND);
                    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glColorMaski(1, GL_FALSE, GL_FALSE, transfog, GL_FALSE);

                    // render where bit7 is set (ie. shadow mask)
                    // set bit6 and replace polyID
                    glStencilFunc(GL_EQUAL, 0xC0|polyid, 0x80);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
                    glStencilMask(0x7F);

                    if (polyattr & (1<<11)) glDepthMask(GL_TRUE);
                    else                    glDepthMask(GL_FALSE);

                    i += RenderSinglePolygon(i);
                }
                else
                {
                    glEnable(GL_BLEND);
                    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glColorMaski(1, GL_FALSE, GL_FALSE, transfog, GL_FALSE);

                    // render where polyID and bit6 do not match
                    // set bit6 and set polyID
                    glStencilFunc(GL_NOTEQUAL, 0x40|polyid, 0x7F);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
                    glStencilMask(0x7F);

                    if (polyattr & (1<<11)) glDepthMask(GL_TRUE);
                    else                    glDepthMask(GL_FALSE);

                    i += needopaque ? RenderSinglePolygon(i) : RenderPolygonBatch(i);
                }
            }
            else
                i++;
        }
    }

polygons_done:
    glBindSampler(0, 0);   // polygon passes done (SetupPolygonTexture)

    if (S.RenderDispCnt & (1<<5)) OpenGL::GLStatAdd(OpenGL::GLStatEdge);
    if (S.RenderDispCnt & (1<<7)) OpenGL::GLStatAdd(OpenGL::GLStatFog);
    if ((S.RenderDispCnt & 0x00A0) && !(OpenGL::GLSkip() & 2)) // fog/edge enabled
    {
        glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glColorMaski(1, GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        glEnable(GL_BLEND);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);

        glDepthFunc(GL_ALWAYS);
        glDepthMask(GL_FALSE);
        glStencilFunc(GL_ALWAYS, 0, 0);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        glStencilMask(0);

        // fog alone reads only this pixel: framebuffer fetch, no attachment sampling
        const bool fogFetch = FinalPassFogFetchShader && !(S.RenderDispCnt & (1<<5));
        if (!fogFetch)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, DepthBufferTex);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, AttrBufferTex);
        }

        glBindBuffer(GL_ARRAY_BUFFER, ClearVertexBufferID);
        glBindVertexArray(ClearVertexArrayID);

        if (S.RenderDispCnt & (1<<5))
        {
            // edge marking
            // TODO: depth/polyid values at screen edges

            glUseProgram(FinalPassEdgeShader);

            glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);

            glDrawArrays(GL_TRIANGLES, 0, 2*3);
        }

        if (S.RenderDispCnt & (1<<7))
        {
            // fog

            glUseProgram(fogFetch ? FinalPassFogFetchShader : FinalPassFogShader);

            if (S.RenderDispCnt & (1<<6))
                glBlendFuncSeparate(GL_ZERO, GL_ONE, GL_CONSTANT_COLOR, GL_ONE_MINUS_SRC_ALPHA);
            else
                glBlendFuncSeparate(GL_CONSTANT_COLOR, GL_ONE_MINUS_SRC_ALPHA, GL_CONSTANT_COLOR, GL_ONE_MINUS_SRC_ALPHA);

            {
                u32 c = S.RenderFogColor;
                u32 r = c & 0x1F;
                u32 g = (c >> 5) & 0x1F;
                u32 b = (c >> 10) & 0x1F;
                u32 a = (c >> 16) & 0x1F;

                glBlendColor((float)r/31.0, (float)g/31.0, (float)b/31.0, (float)a/31.0);
            }

            glDrawArrays(GL_TRIANGLES, 0, 2*3);
        }
    }
}
#undef glDepthFunc
#pragma pop_macro("glDepthFunc")
#undef glStencilFunc
#pragma pop_macro("glStencilFunc")
#undef glStencilOp
#pragma pop_macro("glStencilOp")
#undef glStencilMask
#pragma pop_macro("glStencilMask")
#undef glDepthMask
#pragma pop_macro("glDepthMask")
#undef glColorMaski
#pragma pop_macro("glColorMaski")
#undef glActiveTexture
#pragma pop_macro("glActiveTexture")
#undef glBindTexture
#pragma pop_macro("glBindTexture")
#undef glBindSampler
#pragma pop_macro("glBindSampler")
// ---- end of the state filter ----


void GLRenderer3D::RenderFrame()
{
    PrepareFrame();
    RenderPreparedFrame();
}

// Emu-thread half of RenderFrame: VRAM coherence for the texture cache (reads emu-side
// dirty tracking, writes the flat texture VRAM). Everything else RenderPreparedFrame reads
// (Render* registers, polygon RAM, flat VRAM) is stable until the next VBlank barrier.
void GLRenderer3D::PrepareFrame(int slot, const std::function<void()>& beforeVRAMWrite)
{
    Texcache.Prepare(slot, [&] { if (beforeVRAMWrite) beforeVRAMWrite(); });

    // Snapshot the render state, so GPU3D::VBlank may re-sort/rewrite it while the frame
    // is still rendering on another thread. The polygons themselves live in GPU3D's
    // double-buffered polygon/vertex RAM, which is not written again until the geometry
    // of the frame after next.
    Snap& n = Next[slot];
    n.RenderDispCnt = GPU3D.RenderDispCnt;
    n.RenderClearAttr1 = GPU3D.RenderClearAttr1;
    n.RenderClearAttr2 = GPU3D.RenderClearAttr2;
    n.RenderFogColor = GPU3D.RenderFogColor;
    n.RenderFogOffset = GPU3D.RenderFogOffset;
    n.RenderFogShift = GPU3D.RenderFogShift;
    memcpy(n.RenderToonTable, GPU3D.RenderToonTable, sizeof(n.RenderToonTable));
    memcpy(n.RenderEdgeTable, GPU3D.RenderEdgeTable, sizeof(n.RenderEdgeTable));
    memcpy(n.RenderFogDensityTable, GPU3D.RenderFogDensityTable, sizeof(n.RenderFogDensityTable));
    n.RenderNumPolygons = GPU3D.RenderNumPolygons;
    // (only GLRenderer keeps captures as GL textures; without it they are plain VRAM)
    if (Parent)
        GPU.GetCaptureInfo_Texture(n.CaptureInfo);
    else
        for (int i = 0; i < 16; i++) n.CaptureInfo[i] = -1;
    // the polygons and their vertices themselves (the job reads them through all its passes,
    // and GPU3D rewrites this bank once its VBlank after next swaps it back: a job running that
    // late, as in Pokemon White's town intro, drew half-rewritten polygons as wedges)
    {
        std::vector<Polygon>& pc = PolyCopy[slot];
        std::vector<Vertex>& vc = VtxCopy[slot];
        const u32 np = n.RenderNumPolygons;
        Vertex* vmin = nullptr; Vertex* vmax = nullptr;
        for (u32 i = 0; i < np; i++)
        {
            const Polygon* p = GPU3D.RenderPolygonRAM[i];
            for (u32 j = 0; j < p->NumVertices; j++)
            {
                Vertex* v = p->Vertices[j];
                if (!vmin || v < vmin) vmin = v;
                if (!vmax || v > vmax) vmax = v;
            }
        }
        if (vmin) vc.assign(vmin, vmax + 1); else vc.clear();
        pc.resize(np);
        for (u32 i = 0; i < np; i++)
        {
            const Polygon* p = GPU3D.RenderPolygonRAM[i];
            pc[i] = *p;
            for (u32 j = 0; j < p->NumVertices; j++) pc[i].Vertices[j] = &vc[p->Vertices[j] - vmin];
            n.RenderPolygonRAM[i] = &pc[i];
        }
    }

    // decided here (not on the render thread) so the caller knows at once which colour
    // buffer this frame's 3D lands in
    n.Skip = !Texcache.PendingChanged(slot) && GPU3D.RenderFrameIdentical;
    if (!n.Skip && ColorRing > 1)
        CurColor = (CurColor + 1) % ColorRing;
    n.Color = CurColor;
}

void GLRenderer3D::RenderPreparedFrame(int slot)
{
    u8 clrBitmapDirty;
    Texcache.Apply(slot, clrBitmapDirty);
    if (Next[slot].Skip)
        return;
    const int nPolys = Next[slot].RenderNumPolygons;
    memcpy(&S, &Next[slot], offsetof(Snap, RenderPolygonRAM) + nPolys * sizeof(Polygon*));

    // figure out which chunks of texture memory contain display captures
    // (only GLRenderer keeps captures as GL textures; without it they are plain VRAM)
    int* captureinfo = S.CaptureInfo;   // snapshotted in PrepareFrame

    if (ColorRing > 1)
    {
        // this frame's ring entry (advanced in PrepareFrame): the previous frames' 3D
        // stays intact for the hybrid merge
        glBindFramebuffer(GL_FRAMEBUFFER, MainFramebuffer);
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ColorBufferTex[S.Color], 0);
    }

    // if we're using a clear bitmap, set that up
    ClearBitmapDirty |= clrBitmapDirty;
    if (S.RenderDispCnt & (1<<14))
    {
        if (ClearBitmapDirty & (1<<0))
        {
            u16* vram = (u16*)&GPU.VRAMFlat_Texture[0x40000];
            for (int i = 0; i < 256*256; i++)
            {
                u16 color = vram[i];
                u32 r = (color << 1) & 0x3E; if (r) r++;
                u32 g = (color >> 4) & 0x3E; if (g) g++;
                u32 b = (color >> 9) & 0x3E; if (b) b++;
                u32 a = (color & 0x8000) ? 31 : 0;

                ClearBitmap[0][i] = r | (g << 8) | (b << 16) | (a << 24);
            }

            glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[0]);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 256, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, ClearBitmap[0]);
        }

        if (ClearBitmapDirty & (1<<1))
        {
            u16* vram = (u16*)&GPU.VRAMFlat_Texture[0x60000];
            for (int i = 0; i < 256*256; i++)
            {
                u16 val = vram[i];
                u32 depth = ((val & 0x7FFF) * 0x200) + 0x1FF;
                u32 fog = (val & 0x8000) << 9;

                ClearBitmap[1][i] = depth | fog;
            }

            glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[1]);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 256, GL_RED_INTEGER, GL_UNSIGNED_INT, ClearBitmap[1]);
        }

        ClearBitmapDirty = 0;
    }

    TexEnable = !!(S.RenderDispCnt & (1<<0));

    CurShaderID = -1;

    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, MainFramebuffer);
    if (TileMode)
    {
        // the clear below rewrites every pixel of every attachment: nothing to load
        static const GLenum all[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_DEPTH_STENCIL_ATTACHMENT};
        glInvalidateFramebuffer(GL_DRAW_FRAMEBUFFER, 3, all);
    }

    ShaderConfig.uScreenSize[0] = ScreenW;
    ShaderConfig.uScreenSize[1] = ScreenH;
    ShaderConfig.uDispCnt = S.RenderDispCnt;

    for (int i = 0; i < 32; i++)
    {
        u16 c = S.RenderToonTable[i];
        u32 r = c & 0x1F;
        u32 g = (c >> 5) & 0x1F;
        u32 b = (c >> 10) & 0x1F;

        ShaderConfig.uToonColors[i][0] = (float)r / 31.0;
        ShaderConfig.uToonColors[i][1] = (float)g / 31.0;
        ShaderConfig.uToonColors[i][2] = (float)b / 31.0;
    }

    for (int i = 0; i < 8; i++)
    {
        u16 c = S.RenderEdgeTable[i];
        u32 r = c & 0x1F;
        u32 g = (c >> 5) & 0x1F;
        u32 b = (c >> 10) & 0x1F;

        ShaderConfig.uEdgeColors[i][0] = (float)r / 31.0;
        ShaderConfig.uEdgeColors[i][1] = (float)g / 31.0;
        ShaderConfig.uEdgeColors[i][2] = (float)b / 31.0;
    }

    {
        u32 c = S.RenderFogColor;
        u32 r = c & 0x1F;
        u32 g = (c >> 5) & 0x1F;
        u32 b = (c >> 10) & 0x1F;
        u32 a = (c >> 16) & 0x1F;

        ShaderConfig.uFogColor[0] = (float)r / 31.0;
        ShaderConfig.uFogColor[1] = (float)g / 31.0;
        ShaderConfig.uFogColor[2] = (float)b / 31.0;
        ShaderConfig.uFogColor[3] = (float)a / 31.0;
    }

    for (int i = 0; i < 34; i++)
    {
        u8 d = S.RenderFogDensityTable[i];
        ShaderConfig.uFogDensity[i][0] = (float)d / 127.0;
    }

    ShaderConfig.uFogOffset = S.RenderFogOffset;
    ShaderConfig.uFogShift = S.RenderFogShift;

    // Buffer updates below re-specify the storage (orphaning): updating a buffer the GPU may
    // still be reading from the previous frame makes the Mali driver wait or copy it.
    glBindBuffer(GL_UNIFORM_BUFFER, ShaderConfigUBO);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(ShaderConfig), &ShaderConfig, GL_DYNAMIC_DRAW);

    glDisable(GL_SCISSOR_TEST);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);

    glViewport(0, 0, ScreenW, ScreenH);

    glDisable(GL_BLEND);
    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glColorMaski(1, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glStencilMask(0xFF);

    glDepthFunc(GL_ALWAYS);
    glStencilFunc(GL_ALWAYS, 0xFF, 0xFF);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

    // clear buffers
    // TODO: check whether 'clear polygon ID' affects translucent polyID
    // (for example when alpha is 1..30)
    if (S.RenderDispCnt & (1<<14))
    {
        // clear bitmap
        glUseProgram(ClearShaderBitmap);

        u32 polyid = (S.RenderClearAttr1 >> 24) & 0x3F;

        float bitmapoffset[2];
        u8 xoff = (S.RenderClearAttr2 >> 16) & 0xFF;
        u8 yoff = (S.RenderClearAttr2 >> 24) & 0xFF;
        bitmapoffset[0] = (float)xoff / 256.0;
        bitmapoffset[1] = (float)yoff / 256.0;

        glUniform2f(ClearBitmapULoc[0], bitmapoffset[0], bitmapoffset[1]);
        glUniform1ui(ClearBitmapULoc[1], polyid);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[0]);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, ClearBitmapTex[1]);
    }
    else
    {
        // plain clear plane
        glUseProgram(ClearShaderPlain);

        u32 r = S.RenderClearAttr1 & 0x1F;
        u32 g = (S.RenderClearAttr1 >> 5) & 0x1F;
        u32 b = (S.RenderClearAttr1 >> 10) & 0x1F;
        u32 fog = (S.RenderClearAttr1 >> 15) & 0x1;
        u32 a = (S.RenderClearAttr1 >> 16) & 0x1F;
        u32 polyid = (S.RenderClearAttr1 >> 24) & 0x3F;
        u32 z = ((S.RenderClearAttr2 & 0x7FFF) * 0x200) + 0x1FF;

        /*if (r) r = r*2 + 1;
        if (g) g = g*2 + 1;
        if (b) b = b*2 + 1;*/

        glUniform4ui(ClearUniformLoc[0], r, g, b, a);
        glUniform1ui(ClearUniformLoc[1], z);
        glUniform1ui(ClearUniformLoc[2], polyid);
        glUniform1ui(ClearUniformLoc[3], fog);

        // debug.litev.gl3dclr (default on): the same constant values as glClearBuffer, which
        // Mali applies at tile load for free, instead of a full-screen shaded draw
        static const bool clr = OpenGL::Prop("gl3dclr", 1) != 0;
        if (clr)
        {
#ifdef __ANDROID__
            const GLfloat col[4] = {b / 31.f, g / 31.f, r / 31.f, a / 31.f};   // 3D layer is BGRA on GLES
#else
            const GLfloat col[4] = {r / 31.f, g / 31.f, b / 31.f, a / 31.f};
#endif
            const GLfloat attr[4] = {polyid / 63.f, 0.f, (GLfloat)fog, 1.f};
            glClearBufferfv(GL_COLOR, 0, col);
            glClearBufferfv(GL_COLOR, 1, attr);
            glClearBufferfi(GL_DEPTH_STENCIL, 0, z / 16777216.f, 0xFF);
            goto cleared;
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, ClearVertexBufferID);
    glBindVertexArray(ClearVertexArrayID);
    glDrawArrays(GL_TRIANGLES, 0, 2*3);
cleared:

    if (S.RenderNumPolygons)
    {
        int npolys = 0;
        int firsttrans = -1;
        for (u32 i = 0; i < S.RenderNumPolygons; i++)
        {
            if (S.RenderPolygonRAM[i]->Degenerate) continue;

            SetupPolygon(&PolygonList[npolys], S.RenderPolygonRAM[i]);
            if (firsttrans < 0 && S.RenderPolygonRAM[i]->Translucent)
                firsttrans = npolys;

            npolys++;
        }
        NumFinalPolys = npolys;
        NumOpaqueFinalPolys = firsttrans;
        StatPolys += npolys;
#ifdef LITEV_GL_SORT_OPAQUE
        // Draw calls only merge polygons that are adjacent with the same state and texture; in
        // submission order a scene with many textures (Pokemon White's town: ~500 draws for ~700
        // polygons, ~16 ms of Mali driver CPU) merges almost nothing. Opaque polygons with the
        // LESS depth test draw the same picture in any order (except exact-depth ties and which
        // polygon ID the stencil keeps: display only), so group them by state and texture.
        // Depth-equal ones (decals) go after the rest, onto finished ground; translucent polygons
        // keep their order (blending depends on it). debug.litev.glsortopaque=0 turns it off.
        static const bool sortOpaque = OpenGL::Prop("glsortopaque", 1) != 0;
        if (sortOpaque)
        {
            const int nopaque = firsttrans < 0 ? npolys : firsttrans;
            auto key = [](const RendererPolygon& rp) {
                const Polygon* p = rp.PolyData;
                return std::make_tuple((p->Attr >> 14) & 1, p->Type == 1, rp.RenderKey, p->TexParam, p->TexPalette);
            };
            std::stable_sort(&PolygonList[0], &PolygonList[nopaque],
                             [&](const RendererPolygon& a, const RendererPolygon& b) { return key(a) < key(b); });
        }
#endif

        BuildPolygons(&PolygonList[0], npolys, captureinfo);
        glBindBuffer(GL_ARRAY_BUFFER, VertexBufferID);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexBuffer), nullptr, GL_DYNAMIC_DRAW);
        glBufferSubData(GL_ARRAY_BUFFER, 0, NumVertices*7*4, VertexBuffer);

        // bind to access the index buffer
        glBindVertexArray(VertexArrayID);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(IndexBuffer), nullptr, GL_DYNAMIC_DRAW);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, NumIndices * 2, IndexBuffer);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, EdgeIndicesOffset * 2, NumEdgeIndices * 2, IndexBuffer + EdgeIndicesOffset);

        RenderSceneChunk(0, 192);
    }

    if (TileMode)
    {
        // only the colour buffer leaves the 3D pass
        static const GLenum aux[2] = {GL_COLOR_ATTACHMENT1, GL_DEPTH_STENCIL_ATTACHMENT};
        glInvalidateFramebuffer(GL_DRAW_FRAMEBUFFER, 2, aux);
    }
}

u32* GLRenderer3D::GetLine(int line)
{
    return nullptr;
}

}
