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

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include "NDS.h"
#include "GPU.h"
#include "FIFO.h"
#include "GPU3D_Soft.h"
#include "Platform.h"
#include "GPU3D.h"
#include "LiteProfile.h"
// LITEV_GXFIFO_DMA_INLINE: single-source inline bodies for the GXFIFO producer path. Included here
// so the public WriteToGXFIFO/CmdFIFOWrite delegate to the SAME code DMA.cpp inlines (no duplication).
// The extern decl inside also gives CmdNumParams (defined below) external linkage for other TUs.
#include "GPU3D_GXFIFO_inl.h"
#ifdef LITEV_GX_CPUSEND
#include "ARMInterpreter.h"
#endif

#if defined(__ANDROID__)
#include <sys/system_properties.h>
#include <cstdlib>
// Read an integer debug.litev.* prop; returns `def` when it is unset (for default-ON levers).
static int litevGxPropDefault(const char* name, int def) {
    char b[8] = {0};
    if (__system_property_get(name, b) > 0) return (atoi(b) != 0) ? 1 : 0;
    return def;
}
#endif

#if (defined(LITEV_NEON_GEOMETRY) || defined(LITEV_GEOM_NEON2) || defined(LITEV_GEOM_NEON3)) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif


namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

#if (defined(LITEV_NEON_GEOMETRY) || defined(LITEV_GEOM_NEON2) || defined(LITEV_GEOM_NEON3)) && defined(__ARM_NEON)
// M6.11 step 3 — integer-NEON kernels for the GPU3D geometry engine's hot
// fixed-point math. These reproduce the scalar results BIT-EXACTLY, not
// approximately:
//   * each element product is a widening 32x32->64 multiply (vmull/vmlal_n_s32),
//     which equals the scalar `(s64)a * b` because both operands fit in s32
//     (vertex coords are s16, matrix entries s32);
//   * accumulation is in 64-bit lanes. 64-bit two's-complement addition is
//     associative and commutative, so the fold order does not matter — the sum
//     equals the scalar left-to-right sum modulo 2^64 even in the (content-
//     unreachable) overflow case, and the DS math is modular anyway;
//   * `>> Shift` is an arithmetic 64-bit shift (vshrq_n_s64), matching the
//     scalar `>>` on a signed s64; Shift is a template arg because the intrinsic
//     needs a compile-time count;
//   * the final store to an s32/s16 field truncates the low bits identically to
//     the scalar narrowing assignment.

// out[0..3] = ( v0*M[0..3] + v1*M[4..7] + v2*M[8..11] + v3*M[12..15] ) >> Shift
// M is four contiguous 4-wide s32 rows. When out aliases M's storage the caller
// must snapshot the rows first (MatrixMult4x4 copies into tmp).
template <int Shift>
static inline void NeonMat4Vec4_s64(s32* out, const s32* M,
                                    s32 v0, s32 v1, s32 v2, s32 v3)
{
    int32x4_t r0 = vld1q_s32(M);
    int32x4_t r1 = vld1q_s32(M + 4);
    int32x4_t r2 = vld1q_s32(M + 8);
    int32x4_t r3 = vld1q_s32(M + 12);

    int64x2_t lo = vmull_n_s32(vget_low_s32(r0),  v0);
    int64x2_t hi = vmull_n_s32(vget_high_s32(r0), v0);
    lo = vmlal_n_s32(lo, vget_low_s32(r1),  v1);
    hi = vmlal_n_s32(hi, vget_high_s32(r1), v1);
    lo = vmlal_n_s32(lo, vget_low_s32(r2),  v2);
    hi = vmlal_n_s32(hi, vget_high_s32(r2), v2);
    lo = vmlal_n_s32(lo, vget_low_s32(r3),  v3);
    hi = vmlal_n_s32(hi, vget_high_s32(r3), v3);

    lo = vshrq_n_s64(lo, Shift);
    hi = vshrq_n_s64(hi, Shift);

    out[0] = (s32)vgetq_lane_s64(lo, 0);
    out[1] = (s32)vgetq_lane_s64(lo, 1);
    out[2] = (s32)vgetq_lane_s64(hi, 0);
    out[3] = (s32)vgetq_lane_s64(hi, 1);
}

// returns { (v0*M[0]+v1*M[4]+v2*M[8])>>Shift , (v0*M[1]+v1*M[5]+v2*M[9])>>Shift }
// i.e. the 3-term, 2-output texcoord transform (the two low lanes of each row).
template <int Shift>
static inline int64x2_t NeonTex2_s64(const s32* M, s32 v0, s32 v1, s32 v2)
{
    int64x2_t acc = vmull_n_s32(vld1_s32(M + 0), v0);
    acc = vmlal_n_s32(acc, vld1_s32(M + 4), v1);
    acc = vmlal_n_s32(acc, vld1_s32(M + 8), v2);
    return vshrq_n_s64(acc, Shift);
}

#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
// LITEV_GEOM_NEON2 — the per-polygon backface-cull cross product in SubmitPolygon.
// The scalar reference (using x=Position[0], y=Position[1], w=Position[3]) is:
//   a = (v0 - v1), b = (v2 - v1)             (component subtraction in 32-bit `int`)
//   normalX = (s64)a.y*b.w - (s64)a.w*b.y
//   normalY = (s64)a.w*b.x - (s64)a.x*b.w
//   normalZ = (s64)a.x*b.y - (s64)a.y*b.x    (a 3D cross product of (x,y,w) vectors)
// BIT-EXACT because:
//   * the two deltas are computed with vsubq_s32 — 32-bit two's-complement wrap,
//     identical to the scalar `int` subtraction (even on the content-unreachable
//     overflow, DS math is modular);
//   * each product is a widening 32x32->64 vmull_s32, equal to the scalar
//     `(s64)delta * delta` since both deltas are s32;
//   * the per-lane 64-bit subtract equals the scalar `-`.
// The two byte-index vectors permute a/b lanes so the 4-lane cross falls out of two
// vmull_s32 halves; lane2 (normalZ's two products) is done with a scalar widening
// multiply of the same permuted lanes to avoid a second, mostly-wasted vmull pair.
// Fills n[0]=normalX, n[1]=normalY, n[2]=normalZ.
static inline void NeonCullNormal(const s32* p0, const s32* p1, const s32* p2, s64 n[3])
{
    int32x4_t a = vsubq_s32(vld1q_s32(p0), vld1q_s32(p1));   // {ax, ay, az(unused), aw}
    int32x4_t b = vsubq_s32(vld1q_s32(p2), vld1q_s32(p1));   // {bx, by, bz(unused), bw}

    // lane maps: perm_p -> {1,3,0,2}, perm_q -> {3,0,1,2} (each lane = 4 source bytes).
    const uint8x16_t perm_p = { 4,5,6,7,  12,13,14,15,  0,1,2,3,  8,9,10,11 };
    const uint8x16_t perm_q = { 12,13,14,15,  0,1,2,3,  4,5,6,7,  8,9,10,11 };
    int32x4_t A1 = vreinterpretq_s32_s8(vqtbl1q_s8(vreinterpretq_s8_s32(a), perm_p)); // {ay, aw, ax, az}
    int32x4_t B1 = vreinterpretq_s32_s8(vqtbl1q_s8(vreinterpretq_s8_s32(b), perm_q)); // {bw, bx, by, bz}
    int32x4_t A2 = vreinterpretq_s32_s8(vqtbl1q_s8(vreinterpretq_s8_s32(a), perm_q)); // {aw, ax, ay, az}
    int32x4_t B2 = vreinterpretq_s32_s8(vqtbl1q_s8(vreinterpretq_s8_s32(b), perm_p)); // {by, bw, bx, bz}

    int64x2_t p1lo = vmull_s32(vget_low_s32(A1), vget_low_s32(B1)); // {ay*bw, aw*bx}
    int64x2_t p2lo = vmull_s32(vget_low_s32(A2), vget_low_s32(B2)); // {aw*by, ax*bw}
    int64x2_t nlo  = vsubq_s64(p1lo, p2lo);                         // {normalX, normalY}

    s64 zpos = (s64)vgetq_lane_s32(A1, 2) * vgetq_lane_s32(B1, 2);  // ax*by
    s64 zneg = (s64)vgetq_lane_s32(A2, 2) * vgetq_lane_s32(B2, 2);  // ay*bx

    n[0] = vgetq_lane_s64(nlo, 0);
    n[1] = vgetq_lane_s64(nlo, 1);
    n[2] = zpos - zneg;
}
#endif
#endif

// 3D engine notes
//
// vertex/polygon RAM is filled when a complete polygon is defined, after it's been culled and clipped
// 04000604 reads from bank used by renderer
// bank used by renderer is emptied at scanline ~192
// banks are swapped at scanline ~194
// TODO: needs more investigation. it's weird.
//
// clipping rules:
// * if a shared vertex in a strip is clipped, affected polygons are converted into single polygons
//   strip is resumed at the first eligible polygon
//
// clipping exhibits oddities on the real thing. bad precision? fancy algorithm? TODO: investigate.
//
// vertex color precision:
// * vertex colors are kept at 5-bit during clipping. makes for shitty results.
// * vertex colors are converted to 9-bit before drawing, as such:
//   if (x > 0) x = (x << 4) + 0xF
//   the added bias affects interpolation.
//
// depth buffer:
// Z-buffering mode: val = ((Z * 0x800 * 0x1000) / W) + 0x7FFEFF (nope, wrong. TODO update)
// W-buffering mode: val = W
//
// formula for clear depth: (GBAtek is wrong there)
// clearZ = (val * 0x200) + 0x1FF;
//
// alpha is 5-bit
//
// matrix push/pop on the position matrix are always applied to the vector matrix too, even in position-only mode
// store/restore too, probably (TODO: confirm)
// (the idea is that each position matrix has an associated vector matrix)
//
// TODO: check if translate works on the vector matrix? seems pointless
//
// viewport Y coordinates are upside-down
//
// several registers are latched upon VBlank, the renderer uses the latched registers
// latched registers include:
// DISP3DCNT
// alpha test ref value
// fog color, offset, density table
// toon table
// edge table
// clear attributes
//
// TODO: check how DISP_1DOT_DEPTH works and whether it's latched
//
// TODO: emulate GPU hanging
// * when calling BEGIN with an incomplete polygon defined
// * probably same with BOXTEST
// * when sending vertices immediately after a BOXTEST
//
// TODO: test results should probably not be presented immediately, even if we set the busy flag


// command execution notes
//
// timings given by GBAtek are for individual commands
// actual display lists have different timing characteristics
// * vertex pipeline: individual vertex commands are able to execute in parallel
//   with certain other commands
// * similarly, the normal command can execute in parallel with a subsequent vertex
// * polygon pipeline: each vertex which completes a polygon takes longer to run
//   and imposes rules on when further vertex commands can run
//   (one every 9-cycle time slot during polygon setup)
//   polygon setup time is 27 cycles for a triangle and 36 for a quad
//   except: only one time slot is taken if the polygon is rejected by culling/clipping
// * additionally, some commands (BEGIN, LIGHT_VECTOR, BOXTEST) stall the polygon pipeline


const u8 CmdNumParams[256] =
{
    // 0x00
    0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x10
    1, 0, 1, 1, 1, 0, 16, 12, 16, 12, 9, 3, 3,
    0, 0, 0,
    // 0x20
    1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0,
    // 0x30
    1, 1, 1, 1, 32,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x40
    1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x50
    1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x60
    1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x70
    3, 2, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 0x80+
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

void MatrixLoadIdentity(s32* m);

GPU3D::GPU3D(melonDS::GPU& gpu) noexcept :
    NDS(gpu.NDS),
    GPU(gpu)
{
}

void Vertex::DoSavestate(Savestate* file) noexcept
{
    file->VarArray(Position, sizeof(Position));
    file->VarArray(Color, sizeof(Color));
    file->VarArray(TexCoords, sizeof(TexCoords));

    file->Bool32(&Clipped);

    file->VarArray(FinalPosition, sizeof(FinalPosition));
    file->VarArray(FinalColor, sizeof(FinalColor));
    file->VarArray(HiresPosition, sizeof(HiresPosition));
}

void GPU3D::ResetRenderingState() noexcept
{
    RenderNumPolygons = 0;

    RenderDispCnt = 0;
    RenderAlphaRef = 0;

    memset(RenderEdgeTable, 0, 8*2);
    memset(RenderToonTable, 0, 32*2);

    RenderFogColor = 0;
    RenderFogOffset = 0;
    RenderFogShift = 0;
    memset(RenderFogDensityTable, 0, 34);

    RenderClearAttr1 = 0x3F000000;
    RenderClearAttr2 = 0x00007FFF;
}

void GPU3D::Reset() noexcept
{
#ifdef LITEV_GXFIFO_UNIFIED
    CmdQHead = PipeN = FifoN = 0;
#else
    CmdFIFO.Clear();
    CmdPIPE.Clear();
#endif

    CmdStallQueue.Clear();

    ZeroDotWLimit = 0xFFFFFF;

    GXStat = 0;

    memset(ExecParams, 0, 32*4);
    ExecParamCount = 0;

    CycleCount = 0;
    VertexPipeline = 0;
    NormalPipeline = 0;
    PolygonPipeline = 0;
    VertexSlotCounter = 0;
    VertexSlotsFree = 1;

    NumPushPopCommands = 0;
    NumTestCommands = 0;

    MatrixMode = 0;

    MatrixLoadIdentity(ProjMatrix);
    MatrixLoadIdentity(PosMatrix);
    MatrixLoadIdentity(VecMatrix);
    MatrixLoadIdentity(TexMatrix);

    ClipMatrixDirty = true;
    UpdateClipMatrix();

    memset(Viewport, 0, sizeof(Viewport));

    memset(ProjMatrixStack, 0, 16*4);
    memset(PosMatrixStack, 0, 31 * 16*4);
    memset(VecMatrixStack, 0, 31 * 16*4);
    memset(TexMatrixStack, 0, 16*4);

    ProjMatrixStackPointer = 0;
    PosMatrixStackPointer = 0;
    TexMatrixStackPointer = 0;

    NumCommands = 0;
    CurCommand = 0;
    ParamCount = 0;
    TotalParams = 0;

    GeometryEnabled = false;
    RenderingEnabled = false;

    DispCnt = 0;
    AlphaRefVal = 0;
    AlphaRef = 0;

    memset(ToonTable, 0, sizeof(ToonTable));
    memset(EdgeTable, 0, sizeof(EdgeTable));

    // TODO: confirm initial polyid/color/fog values
    FogOffset = 0;
    FogColor = 0;
    memset(FogDensityTable, 0, sizeof(FogDensityTable));

    ClearAttr1 = 0x3F000000;
    ClearAttr2 = 0x00007FFF;

    ResetRenderingState();

    AbortFrame = false;

    Timestamp = 0;

    PolygonMode = 0;
    memset(CurVertex, 0, sizeof(CurVertex));
    memset(VertexColor, 0, sizeof(VertexColor));
    memset(TexCoords, 0, sizeof(TexCoords));
    memset(RawTexCoords, 0, sizeof(RawTexCoords));
    memset(Normal, 0, sizeof(Normal));

    memset(LightDirection, 0, sizeof(LightDirection));
    memset(LightColor, 0, sizeof(LightColor));
    memset(MatDiffuse, 0, sizeof(MatDiffuse));
    memset(MatAmbient, 0, sizeof(MatAmbient));
    memset(MatSpecular, 0, sizeof(MatSpecular));
    memset(MatEmission, 0, sizeof(MatSpecular));

    UseShininessTable = false;
    // Shininess table seems to be uninitialized garbage, at least on n3dsxl hw?
    // Also doesn't seem to be cleared properly unless the system is fully powered off?
    memset(ShininessTable, 0, sizeof(ShininessTable));

    PolygonAttr = 0;
    CurPolygonAttr = 0;

    TexParam = 0;
    TexPalette = 0;

    memset(PosTestResult, 0, 4*4);
    memset(VecTestResult, 0, 2*3);

    memset(TempVertexBuffer, 0, sizeof(TempVertexBuffer));
    VertexNum = 0;
    VertexNumInPoly = 0;
    NumConsecutivePolygons = 0;
    LastStripPolygon = nullptr;
    NumOpaquePolygons = 0;

    CurVertexRAM = &VertexRAM[0];
    CurPolygonRAM = &PolygonRAM[0];
    NumVertices = 0;
    NumPolygons = 0;
    CurRAMBank = 0;

    FlushRequest = 0;
    FlushAttributes = 0;

    RenderXPos = 0;
}

void GPU3D::DoSavestate(Savestate* file) noexcept
{
    file->Section("GP3D");

#ifdef LITEV_GXFIFO_UNIFIED
    {
        // same savestate layout as the two separate FIFOs
        FIFO<CmdFIFOEntry, 256> fifo {};
        FIFO<CmdFIFOEntry, 4> pipe {};
        if (file->Saving)
        {
            for (u32 i = 0; i < PipeN; i++) pipe.Write(CmdQ[(CmdQHead + i) & 511]);
            for (u32 i = 0; i < FifoN; i++) fifo.Write(CmdQ[(CmdQHead + PipeN + i) & 511]);
        }
        fifo.DoSavestate(file);
        pipe.DoSavestate(file);
        if (!file->Saving)
        {
            CmdQHead = 0;
            PipeN = pipe.Level(); FifoN = fifo.Level();
            for (u32 i = 0; i < PipeN; i++) CmdQ[i] = pipe.Read();
            for (u32 i = 0; i < FifoN; i++) CmdQ[PipeN + i] = fifo.Read();
        }
    }
#else
    CmdFIFO.DoSavestate(file);
    CmdPIPE.DoSavestate(file);
#endif

    file->Var32(&NumCommands);
    file->Var32(&CurCommand);
    file->Var32(&ParamCount);
    file->Var32(&TotalParams);

    file->Var32(&NumPushPopCommands);
    file->Var32(&NumTestCommands);

    file->Var32(&DispCnt);
    file->Var8(&AlphaRefVal);
    file->Var8(&AlphaRef);

    file->VarArray(ToonTable, 32*2);
    file->VarArray(EdgeTable, 8*2);

    file->Var32(&FogColor);
    file->Var32(&FogOffset);
    file->VarArray(FogDensityTable, 32);

    file->Var32(&ClearAttr1);
    file->Var32(&ClearAttr2);

    file->Var32(&RenderDispCnt);
    file->Var8(&RenderAlphaRef);

    file->VarArray(RenderToonTable, 32*2);
    file->VarArray(RenderEdgeTable, 8*2);

    file->Var32(&RenderFogColor);
    file->Var32(&RenderFogOffset);
    file->Var32(&RenderFogShift);
    file->VarArray(RenderFogDensityTable, 34);

    file->Var32(&RenderClearAttr1);
    file->Var32(&RenderClearAttr2);

    file->Var16(&RenderXPos);

    file->Var32(&ZeroDotWLimit);

    file->Var32(&GXStat);

    file->VarArray(ExecParams, 32*4);
    file->Var32(&ExecParamCount);
    file->Var32((u32*)&CycleCount);
    file->Var64(&Timestamp);

    file->Var32(&MatrixMode);

    file->VarArray(ProjMatrix, 16*4);
    file->VarArray(PosMatrix, 16*4);
    file->VarArray(VecMatrix, 16*4);
    file->VarArray(TexMatrix, 16*4);

    file->VarArray(ProjMatrixStack, 16*4);
    file->VarArray(PosMatrixStack, 32*16*4);
    file->VarArray(VecMatrixStack, 32*16*4);
    file->VarArray(TexMatrixStack, 16*4);

    file->Var32((u32*)&ProjMatrixStackPointer);
    file->Var32((u32*)&PosMatrixStackPointer);
    file->Var32((u32*)&TexMatrixStackPointer);

    file->VarArray(Viewport, sizeof(Viewport));

    file->VarArray(PosTestResult, 4*4);
    file->VarArray(VecTestResult, 2*3);

    file->Var32(&VertexNum);
    file->Var32(&VertexNumInPoly);
    file->Var32(&NumConsecutivePolygons);

    for (Vertex& vtx : TempVertexBuffer)
    {
        vtx.DoSavestate(file);
    }

    if (file->Saving)
    {
        u32 index = LastStripPolygon ? (u32)(LastStripPolygon - &PolygonRAM[0]) : UINT32_MAX;
        file->Var32(&index);
    }
    else
    {
        u32 index = UINT32_MAX;
        file->Var32(&index);
        LastStripPolygon = (index == UINT32_MAX) ? nullptr : &PolygonRAM[index];
    }

    file->Var32(&CurRAMBank);
    file->Var32(&NumVertices);
    file->Var32(&NumPolygons);
    file->Var32(&NumOpaquePolygons);

    file->Var32(&FlushRequest);
    file->Var32(&FlushAttributes);

    for (Vertex& vtx : VertexRAM)
    {
        vtx.DoSavestate(file);
    }

    for(int i = 0; i < 2048*2; i++)
    {
        Polygon* poly = &PolygonRAM[i];

        // this is a bit ugly, but eh
        // we can't save the pointers as-is, that's a bad idea
        if (file->Saving)
        {
            for (int j = 0; j < 10; j++)
            {
                Vertex* ptr = poly->Vertices[j];
                u32 index = ptr ? (u32)(ptr - &VertexRAM[0]) : UINT32_MAX;
                file->Var32(&index);
            }
        }
        else
        {
            for (int j = 0; j < 10; j++)
            {
                u32 index = UINT32_MAX;
                file->Var32(&index);
                poly->Vertices[j] = index == UINT32_MAX ? nullptr : &VertexRAM[index];
            }
        }

        file->Var32(&poly->NumVertices);

        file->VarArray(poly->FinalZ, sizeof(s32)*10);
        file->VarArray(poly->FinalW, sizeof(s32)*10);
        file->Bool32(&poly->WBuffer);

        file->Var32(&poly->Attr);
        file->Var32(&poly->TexParam);
        file->Var32(&poly->TexPalette);

        file->Bool32(&poly->FacingView);
        file->Bool32(&poly->Translucent);

        file->Bool32(&poly->IsShadowMask);
        file->Bool32(&poly->IsShadow);

        if (file->IsAtLeastVersion(4, 1))
            file->Var32((u32*)&poly->Type);
        else
            poly->Type = 0;

        file->Var32(&poly->VTop);
        file->Var32(&poly->VBottom);
        file->Var32((u32*)&poly->YTop);
        file->Var32((u32*)&poly->YBottom);
        file->Var32((u32*)&poly->XTop);
        file->Var32((u32*)&poly->XBottom);

        file->Var32(&poly->SortKey);

        if (!file->Saving)
        {
            poly->Degenerate = false;

            for (u32 j = 0; j < poly->NumVertices; j++)
            {
                if (poly->Vertices[j]->Position[3] == 0)
                    poly->Degenerate = true;
            }

            if (poly->YBottom > 192) poly->Degenerate = true;
        }
    }

    CmdStallQueue.DoSavestate(file);

    file->Var32((u32*)&VertexPipeline);
    file->Var32((u32*)&NormalPipeline);
    file->Var32((u32*)&PolygonPipeline);
    file->Var32((u32*)&VertexSlotCounter);
    file->Var32(&VertexSlotsFree);

    if (!file->Saving)
    {
        ClipMatrixDirty = true;
        UpdateClipMatrix();

        CurVertexRAM = &VertexRAM[CurRAMBank ? 6144 : 0];
        CurPolygonRAM = &PolygonRAM[CurRAMBank ? 2048 : 0];
    }

    file->Var32(&RenderNumPolygons);
    if (file->Saving)
    {
        for (const Polygon* p : RenderPolygonRAM)
        {
            u32 index = p ? (p - &PolygonRAM[0]) : UINT32_MAX;

            file->Var32(&index);
        }
    }
    else
    {
        for (int i = 0; i < RenderPolygonRAM.size(); ++i)
        {
            u32 index = UINT32_MAX;
            file->Var32(&index);

            RenderPolygonRAM[i] = index == UINT32_MAX ? nullptr : &PolygonRAM[index];
        }
    }

    file->VarArray(CurVertex, sizeof(s16)*3);
    file->VarArray(VertexColor, sizeof(u8)*3);
    file->VarArray(TexCoords, sizeof(s16)*2);
    file->VarArray(RawTexCoords, sizeof(s16)*2);
    file->VarArray(Normal, sizeof(s16)*3);

    file->VarArray(LightDirection, sizeof(s16)*4*3);
    file->VarArray(LightColor, sizeof(u8)*4*3);
    file->VarArray(MatDiffuse, sizeof(u8)*3);
    file->VarArray(MatAmbient, sizeof(u8)*3);
    file->VarArray(MatSpecular, sizeof(u8)*3);
    file->VarArray(MatEmission, sizeof(u8)*3);

    file->Bool32(&UseShininessTable);
    file->VarArray(ShininessTable, 128*sizeof(u8));

    file->Bool32(&AbortFrame);
    file->Bool32(&GeometryEnabled);
    file->Bool32(&RenderingEnabled);
    file->Var32(&PolygonMode);
    file->Var32(&PolygonAttr);
    file->Var32(&CurPolygonAttr);
    file->Var32(&TexParam);
    file->Var32(&TexPalette);

    RenderFrameIdentical = false;
}



void GPU3D::SetEnabled(bool geometry, bool rendering) noexcept
{
    GeometryEnabled = geometry;
    RenderingEnabled = rendering;

    if (!rendering) ResetRenderingState();
}



void MatrixLoadIdentity(s32* m)
{
    m[0] = 0x1000; m[1] = 0;      m[2] = 0;       m[3] = 0;
    m[4] = 0;      m[5] = 0x1000; m[6] = 0;       m[7] = 0;
    m[8] = 0;      m[9] = 0;      m[10] = 0x1000; m[11] = 0;
    m[12] = 0;     m[13] = 0;     m[14] = 0;      m[15] = 0x1000;
}

void MatrixLoad4x4(s32* m, s32* s)
{
    memcpy(m, s, 16*4);
}

void MatrixLoad4x3(s32* m, s32* s)
{
    m[0] = s[0];  m[1] = s[1];  m[2] = s[2];    m[3] = 0;
    m[4] = s[3];  m[5] = s[4];  m[6] = s[5];    m[7] = 0;
    m[8] = s[6];  m[9] = s[7];  m[10] = s[8];   m[11] = 0;
    m[12] = s[9]; m[13] = s[10]; m[14] = s[11]; m[15] = 0x1000;
}

void MatrixMult4x4(s32* m, s32* s)
{
    s32 tmp[16];
    memcpy(tmp, m, 16*4);

#if defined(LITEV_NEON_GEOMETRY) && defined(__ARM_NEON)
    // m = s*m, one output row per call. Row i of s is the vector; tmp holds the
    // (snapshotted) matrix rows the transform consumes column-wise.
    NeonMat4Vec4_s64<12>(m + 0,  tmp, s[0],  s[1],  s[2],  s[3]);
    NeonMat4Vec4_s64<12>(m + 4,  tmp, s[4],  s[5],  s[6],  s[7]);
    NeonMat4Vec4_s64<12>(m + 8,  tmp, s[8],  s[9],  s[10], s[11]);
    NeonMat4Vec4_s64<12>(m + 12, tmp, s[12], s[13], s[14], s[15]);
    return;
#endif

    // m = s*m
    m[0] = ((s64)s[0]*tmp[0] + (s64)s[1]*tmp[4] + (s64)s[2]*tmp[8] + (s64)s[3]*tmp[12]) >> 12;
    m[1] = ((s64)s[0]*tmp[1] + (s64)s[1]*tmp[5] + (s64)s[2]*tmp[9] + (s64)s[3]*tmp[13]) >> 12;
    m[2] = ((s64)s[0]*tmp[2] + (s64)s[1]*tmp[6] + (s64)s[2]*tmp[10] + (s64)s[3]*tmp[14]) >> 12;
    m[3] = ((s64)s[0]*tmp[3] + (s64)s[1]*tmp[7] + (s64)s[2]*tmp[11] + (s64)s[3]*tmp[15]) >> 12;

    m[4] = ((s64)s[4]*tmp[0] + (s64)s[5]*tmp[4] + (s64)s[6]*tmp[8] + (s64)s[7]*tmp[12]) >> 12;
    m[5] = ((s64)s[4]*tmp[1] + (s64)s[5]*tmp[5] + (s64)s[6]*tmp[9] + (s64)s[7]*tmp[13]) >> 12;
    m[6] = ((s64)s[4]*tmp[2] + (s64)s[5]*tmp[6] + (s64)s[6]*tmp[10] + (s64)s[7]*tmp[14]) >> 12;
    m[7] = ((s64)s[4]*tmp[3] + (s64)s[5]*tmp[7] + (s64)s[6]*tmp[11] + (s64)s[7]*tmp[15]) >> 12;

    m[8] = ((s64)s[8]*tmp[0] + (s64)s[9]*tmp[4] + (s64)s[10]*tmp[8] + (s64)s[11]*tmp[12]) >> 12;
    m[9] = ((s64)s[8]*tmp[1] + (s64)s[9]*tmp[5] + (s64)s[10]*tmp[9] + (s64)s[11]*tmp[13]) >> 12;
    m[10] = ((s64)s[8]*tmp[2] + (s64)s[9]*tmp[6] + (s64)s[10]*tmp[10] + (s64)s[11]*tmp[14]) >> 12;
    m[11] = ((s64)s[8]*tmp[3] + (s64)s[9]*tmp[7] + (s64)s[10]*tmp[11] + (s64)s[11]*tmp[15]) >> 12;

    m[12] = ((s64)s[12]*tmp[0] + (s64)s[13]*tmp[4] + (s64)s[14]*tmp[8] + (s64)s[15]*tmp[12]) >> 12;
    m[13] = ((s64)s[12]*tmp[1] + (s64)s[13]*tmp[5] + (s64)s[14]*tmp[9] + (s64)s[15]*tmp[13]) >> 12;
    m[14] = ((s64)s[12]*tmp[2] + (s64)s[13]*tmp[6] + (s64)s[14]*tmp[10] + (s64)s[15]*tmp[14]) >> 12;
    m[15] = ((s64)s[12]*tmp[3] + (s64)s[13]*tmp[7] + (s64)s[14]*tmp[11] + (s64)s[15]*tmp[15]) >> 12;
}

void MatrixMult4x3(s32* m, s32* s)
{
    s32 tmp[16];
    memcpy(tmp, m, 16*4);

#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
    // m = s*m, one output row per NeonMat4Vec4 call. Rows 0-2 have an implicit
    // 4th coefficient of 0 (v3=0 -> +0*tmp[12..15], harmless); row 3 uses the
    // constant 0x1000, matching the scalar (s64)0x1000*tmp[12..15] term. tmp is
    // the snapshotted matrix (all 16 rows valid). Bit-exact vs the scalar below
    // (identical widen/accumulate/>>12/truncate as MatrixMult4x4).
    NeonMat4Vec4_s64<12>(m + 0,  tmp, s[0], s[1], s[2],  0);
    NeonMat4Vec4_s64<12>(m + 4,  tmp, s[3], s[4], s[5],  0);
    NeonMat4Vec4_s64<12>(m + 8,  tmp, s[6], s[7], s[8],  0);
    NeonMat4Vec4_s64<12>(m + 12, tmp, s[9], s[10], s[11], 0x1000);
    return;
#endif

    // m = s*m
    m[0] = ((s64)s[0]*tmp[0] + (s64)s[1]*tmp[4] + (s64)s[2]*tmp[8]) >> 12;
    m[1] = ((s64)s[0]*tmp[1] + (s64)s[1]*tmp[5] + (s64)s[2]*tmp[9]) >> 12;
    m[2] = ((s64)s[0]*tmp[2] + (s64)s[1]*tmp[6] + (s64)s[2]*tmp[10]) >> 12;
    m[3] = ((s64)s[0]*tmp[3] + (s64)s[1]*tmp[7] + (s64)s[2]*tmp[11]) >> 12;

    m[4] = ((s64)s[3]*tmp[0] + (s64)s[4]*tmp[4] + (s64)s[5]*tmp[8]) >> 12;
    m[5] = ((s64)s[3]*tmp[1] + (s64)s[4]*tmp[5] + (s64)s[5]*tmp[9]) >> 12;
    m[6] = ((s64)s[3]*tmp[2] + (s64)s[4]*tmp[6] + (s64)s[5]*tmp[10]) >> 12;
    m[7] = ((s64)s[3]*tmp[3] + (s64)s[4]*tmp[7] + (s64)s[5]*tmp[11]) >> 12;

    m[8] = ((s64)s[6]*tmp[0] + (s64)s[7]*tmp[4] + (s64)s[8]*tmp[8]) >> 12;
    m[9] = ((s64)s[6]*tmp[1] + (s64)s[7]*tmp[5] + (s64)s[8]*tmp[9]) >> 12;
    m[10] = ((s64)s[6]*tmp[2] + (s64)s[7]*tmp[6] + (s64)s[8]*tmp[10]) >> 12;
    m[11] = ((s64)s[6]*tmp[3] + (s64)s[7]*tmp[7] + (s64)s[8]*tmp[11]) >> 12;

    m[12] = ((s64)s[9]*tmp[0] + (s64)s[10]*tmp[4] + (s64)s[11]*tmp[8] + (s64)0x1000*tmp[12]) >> 12;
    m[13] = ((s64)s[9]*tmp[1] + (s64)s[10]*tmp[5] + (s64)s[11]*tmp[9] + (s64)0x1000*tmp[13]) >> 12;
    m[14] = ((s64)s[9]*tmp[2] + (s64)s[10]*tmp[6] + (s64)s[11]*tmp[10] + (s64)0x1000*tmp[14]) >> 12;
    m[15] = ((s64)s[9]*tmp[3] + (s64)s[10]*tmp[7] + (s64)s[11]*tmp[11] + (s64)0x1000*tmp[15]) >> 12;
}

void MatrixMult3x3(s32* m, s32* s)
{
    // Single tmp[16] shared by both paths (matches MatrixMult4x3): the scalar path
    // reads only tmp[0..11]; the NEON path also needs tmp[12..15] in-bounds for the
    // kernel's vld1q_s32(tmp+12), zeroed below so they contribute *0.
    s32 tmp[16];
    memcpy(tmp, m, 12*4);

#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
    // 3x3 writes only m[0..11] (3 rows), each a 3-term row with implicit v3=0.
    // tmp[12..15] are zeroed then multiplied by v3=0 -> contribute exactly 0, so
    // the result is bit-exact vs the scalar path below (which never reads them).
    tmp[12] = tmp[13] = tmp[14] = tmp[15] = 0;
    NeonMat4Vec4_s64<12>(m + 0, tmp, s[0], s[1], s[2], 0);
    NeonMat4Vec4_s64<12>(m + 4, tmp, s[3], s[4], s[5], 0);
    NeonMat4Vec4_s64<12>(m + 8, tmp, s[6], s[7], s[8], 0);
    return;
#endif


    // m = s*m
    m[0] = ((s64)s[0]*tmp[0] + (s64)s[1]*tmp[4] + (s64)s[2]*tmp[8]) >> 12;
    m[1] = ((s64)s[0]*tmp[1] + (s64)s[1]*tmp[5] + (s64)s[2]*tmp[9]) >> 12;
    m[2] = ((s64)s[0]*tmp[2] + (s64)s[1]*tmp[6] + (s64)s[2]*tmp[10]) >> 12;
    m[3] = ((s64)s[0]*tmp[3] + (s64)s[1]*tmp[7] + (s64)s[2]*tmp[11]) >> 12;

    m[4] = ((s64)s[3]*tmp[0] + (s64)s[4]*tmp[4] + (s64)s[5]*tmp[8]) >> 12;
    m[5] = ((s64)s[3]*tmp[1] + (s64)s[4]*tmp[5] + (s64)s[5]*tmp[9]) >> 12;
    m[6] = ((s64)s[3]*tmp[2] + (s64)s[4]*tmp[6] + (s64)s[5]*tmp[10]) >> 12;
    m[7] = ((s64)s[3]*tmp[3] + (s64)s[4]*tmp[7] + (s64)s[5]*tmp[11]) >> 12;

    m[8] = ((s64)s[6]*tmp[0] + (s64)s[7]*tmp[4] + (s64)s[8]*tmp[8]) >> 12;
    m[9] = ((s64)s[6]*tmp[1] + (s64)s[7]*tmp[5] + (s64)s[8]*tmp[9]) >> 12;
    m[10] = ((s64)s[6]*tmp[2] + (s64)s[7]*tmp[6] + (s64)s[8]*tmp[10]) >> 12;
    m[11] = ((s64)s[6]*tmp[3] + (s64)s[7]*tmp[7] + (s64)s[8]*tmp[11]) >> 12;
}

void MatrixScale(s32* m, s32* s)
{
#if defined(LITEV_GEOM_NEON3) && defined(__ARM_NEON)
    // m[4r+j] = ((s64)s[r] * m[4r+j]) >> 12, rows r=0..2 (row 3 / m[12..15] is
    // left untouched, exactly as the scalar below). Each lane is a widening
    // 32x32->64 multiply (== scalar (s64)s[r]*m[4r+j], both operands s32), an
    // arithmetic 64-bit >>12, then a truncating store to s32 — bit-identical to
    // the scalar narrowing assignment. The vld1q of a row happens before its
    // store, so aliasing m in/out is safe.
    for (int r = 0; r < 3; r++)
    {
        int32x4_t row = vld1q_s32(m + r*4);
        int64x2_t lo = vshrq_n_s64(vmull_n_s32(vget_low_s32(row),  s[r]), 12);
        int64x2_t hi = vshrq_n_s64(vmull_n_s32(vget_high_s32(row), s[r]), 12);
        m[r*4+0] = (s32)vgetq_lane_s64(lo, 0);
        m[r*4+1] = (s32)vgetq_lane_s64(lo, 1);
        m[r*4+2] = (s32)vgetq_lane_s64(hi, 0);
        m[r*4+3] = (s32)vgetq_lane_s64(hi, 1);
    }
    return;
#endif
    m[0] = ((s64)s[0]*m[0]) >> 12;
    m[1] = ((s64)s[0]*m[1]) >> 12;
    m[2] = ((s64)s[0]*m[2]) >> 12;
    m[3] = ((s64)s[0]*m[3]) >> 12;

    m[4] = ((s64)s[1]*m[4]) >> 12;
    m[5] = ((s64)s[1]*m[5]) >> 12;
    m[6] = ((s64)s[1]*m[6]) >> 12;
    m[7] = ((s64)s[1]*m[7]) >> 12;

    m[8] = ((s64)s[2]*m[8]) >> 12;
    m[9] = ((s64)s[2]*m[9]) >> 12;
    m[10] = ((s64)s[2]*m[10]) >> 12;
    m[11] = ((s64)s[2]*m[11]) >> 12;
}

void MatrixTranslate(s32* m, s32* s)
{
#if defined(LITEV_GEOM_NEON3) && defined(__ARM_NEON)
    // The added delta is a 3-term Mat*Vec: lane j = (s[0]*m[j] + s[1]*m[4+j] +
    // s[2]*m[8+j]) >> 12. Reuses the proven NeonMat4Vec4_s64<12> kernel with v3=0
    // (so the +v3*m[12..15] term is exactly 0) writing to a SEPARATE temp t[] that
    // does not alias m. Then m[12+j] += t[j]: because m[12+j] is s32 and t[j] is
    // the s32-truncated shifted sum, m[12+j] + t[j] (mod 2^32) equals the scalar
    // (s32)(m[12+j] + (s64)sum>>12) (mod 2^32) — truncation distributes over add.
    s32 t[4];
    NeonMat4Vec4_s64<12>(t, m, s[0], s[1], s[2], 0);
    m[12] += t[0];
    m[13] += t[1];
    m[14] += t[2];
    m[15] += t[3];
    return;
#endif
    m[12] += ((s64)s[0]*m[0] + (s64)s[1]*m[4] + (s64)s[2]*m[8]) >> 12;
    m[13] += ((s64)s[0]*m[1] + (s64)s[1]*m[5] + (s64)s[2]*m[9]) >> 12;
    m[14] += ((s64)s[0]*m[2] + (s64)s[1]*m[6] + (s64)s[2]*m[10]) >> 12;
    m[15] += ((s64)s[0]*m[3] + (s64)s[1]*m[7] + (s64)s[2]*m[11]) >> 12;
}

void GPU3D::UpdateClipMatrix() noexcept
{
    if (!ClipMatrixDirty) return;
    ClipMatrixDirty = false;

    memcpy(ClipMatrix, ProjMatrix, 16*4);
    MatrixMult4x4(ClipMatrix, PosMatrix);
}



void GPU3D::AddCycles(s32 num) noexcept
{
    CycleCount += num;

    if (VertexPipeline > 0)
    {
        if (VertexPipeline > num) VertexPipeline -= num;
        else                      VertexPipeline = 0;
    }

    if (PolygonPipeline > 0)
    {
        if (PolygonPipeline > num)
        {
            PolygonPipeline -= num;
            VertexSlotCounter += num;
            while (VertexSlotCounter > 9)
            {
                VertexSlotCounter -= 9;
                VertexSlotsFree >>= 1;
            }
        }
        else
        {
            PolygonPipeline = 0;
            VertexSlotCounter = 0;
            VertexSlotsFree = 0x1;
        }
    }
}

void GPU3D::NextVertexSlot() noexcept
{
    s32 num = (9 - VertexSlotCounter) + 1;

    for (;;)
    {
        CycleCount += num;

        if (VertexPipeline > 0)
        {
            if (VertexPipeline > num) VertexPipeline -= num;
            else                      VertexPipeline = 0;
        }

        if (PolygonPipeline > 0)
        {
            if (PolygonPipeline > num)
            {
                PolygonPipeline -= num;
                VertexSlotCounter = 1;
                VertexSlotsFree >>= 1;
                if (VertexSlotsFree & 0x1)
                {
                    VertexSlotsFree &= ~0x1;
                    break;
                }
                else
                {
                    num = 9;
                    continue;
                }
            }
            else
            {
                PolygonPipeline = 0;
                VertexSlotCounter = 0;
                VertexSlotsFree = 1;
                break;
            }
        }
    }
}

void GPU3D::StallPolygonPipeline(s32 delay, s32 nonstalldelay) noexcept
{
    if (PolygonPipeline > 0)
    {
        CycleCount += PolygonPipeline + delay;

        // can be safely assumed those two will go to zero
        VertexPipeline = 0;
        NormalPipeline = 0;

        PolygonPipeline = 0;
        VertexSlotCounter = 0;
        VertexSlotsFree = 1;
    }
    else
    {
        if (VertexPipeline > nonstalldelay)
            AddCycles((VertexPipeline - nonstalldelay) + 1);
        else
            AddCycles(NormalPipeline + 1);
    }
}



template<int comp, s32 plane, bool attribs>
void ClipSegment(Vertex* outbuf, const Vertex* vin, const Vertex* vout)
{
    s64 factor_num = vin->Position[3] - (plane*vin->Position[comp]);
    s32 factor_den = factor_num - (vout->Position[3] - (plane*vout->Position[comp]));

#define INTERPOLATE(var)  { outbuf->var = (vin->var + ((vout->var - vin->var) * factor_num) / factor_den); }

    if (comp != 0) INTERPOLATE(Position[0]);
    if (comp != 1) INTERPOLATE(Position[1]);
    if (comp != 2) INTERPOLATE(Position[2]);
    INTERPOLATE(Position[3]);
    outbuf->Position[comp] = plane*outbuf->Position[3];

    if (attribs)
    {
        INTERPOLATE(Color[0]);
        INTERPOLATE(Color[1]);
        INTERPOLATE(Color[2]);

        INTERPOLATE(TexCoords[0]);
        INTERPOLATE(TexCoords[1]);
    }

    outbuf->Clipped = true;

#undef INTERPOLATE
}

template<int comp, bool attribs>
int ClipAgainstPlane(const GPU3D& gpu, Vertex* vertices, int nverts, int clipstart)
{
#ifdef LITEV_GEOM_CLIP_PLANESKIP
    // No vertex beyond either side of this plane: both passes below would copy every
    // vertex through `temp` and back unchanged, leaving only the colour fix-up.
    {
        bool any = false;
        for (int i = clipstart; i < nverts; i++)
        {
            s32 p = vertices[i].Position[comp], w = vertices[i].Position[3];
            any |= (p > w) | (p < -w);
        }
        if (!any)
        {
            for (int i = 0; i < nverts; i++)
            {
                Vertex* vtx = &vertices[i];
                vtx->Color[0] &= ~0xFFF; vtx->Color[0] += 0xFFF;
                vtx->Color[1] &= ~0xFFF; vtx->Color[1] += 0xFFF;
                vtx->Color[2] &= ~0xFFF; vtx->Color[2] += 0xFFF;
            }
            return nverts;
        }
    }
#endif
#ifdef LITEV_GX_CLIP_LEAN
    // Same result with fewer 64-byte vertex copies: vertices are read in place (no local copy per
    // vertex), and a side with no vertex past it skips its pass (the pass would copy them through
    // unchanged); the result lands in temp or vertices, copied back only when it ends in temp.
    {
        Vertex temp[10];
        const Vertex* in = vertices;
        int n = nverts;
        bool plus = false;
        for (int i = clipstart; i < n; i++) plus |= vertices[i].Position[comp] > vertices[i].Position[3];
        if (plus)
        {
            int c = clipstart;
            if (clipstart == 2) { temp[0] = vertices[0]; temp[1] = vertices[1]; }
            for (int i = clipstart; i < n; i++)
            {
                const Vertex* vtx = &vertices[i];
                if (vtx->Position[comp] > vtx->Position[3])
                {
                    if ((comp == 2) && (!(gpu.CurPolygonAttr & (1<<12)))) return 0;
                    const Vertex* vprev = &vertices[i == 0 ? n - 1 : i - 1];
                    if (vprev->Position[comp] <= vprev->Position[3]) ClipSegment<comp, 1, attribs>(&temp[c++], vtx, vprev);
                    const Vertex* vnext = &vertices[i + 1 >= n ? 0 : i + 1];
                    if (vnext->Position[comp] <= vnext->Position[3]) ClipSegment<comp, 1, attribs>(&temp[c++], vtx, vnext);
                }
                else
                    temp[c++] = *vtx;
            }
            in = temp; n = c;
        }
        bool minus = false;
        for (int i = clipstart; i < n; i++) minus |= in[i].Position[comp] < -in[i].Position[3];
        if (minus)
        {
            Vertex* out = in == vertices ? temp : vertices;
            int c = clipstart;
            if (clipstart == 2 && out == temp) { temp[0] = vertices[0]; temp[1] = vertices[1]; }
            for (int i = clipstart; i < n; i++)
            {
                const Vertex* vtx = &in[i];
                if (vtx->Position[comp] < -vtx->Position[3])
                {
                    const Vertex* vprev = &in[i == 0 ? n - 1 : i - 1];
                    if (vprev->Position[comp] >= -vprev->Position[3]) ClipSegment<comp, -1, attribs>(&out[c++], vtx, vprev);
                    const Vertex* vnext = &in[i + 1 >= n ? 0 : i + 1];
                    if (vnext->Position[comp] >= -vnext->Position[3]) ClipSegment<comp, -1, attribs>(&out[c++], vtx, vnext);
                }
                else
                    out[c++] = *vtx;
            }
            in = out; n = c;
        }
        if (in != vertices)
            for (int i = 0; i < n; i++) vertices[i] = in[i];
        for (int i = 0; i < n; i++)
        {
            Vertex* vtx = &vertices[i];
            vtx->Color[0] &= ~0xFFF; vtx->Color[0] += 0xFFF;
            vtx->Color[1] &= ~0xFFF; vtx->Color[1] += 0xFFF;
            vtx->Color[2] &= ~0xFFF; vtx->Color[2] += 0xFFF;
        }
        return n;
    }
#endif
    Vertex temp[10];
    int prev, next;
    int c = clipstart;

    if (clipstart == 2)
    {
        temp[0] = vertices[0];
        temp[1] = vertices[1];
    }

    for (int i = clipstart; i < nverts; i++)
    {
        prev = i-1; if (prev < 0) prev = nverts-1;
        next = i+1; if (next >= nverts) next = 0;

        Vertex vtx = vertices[i];
        if (vtx.Position[comp] > vtx.Position[3])
        {
            if ((comp == 2) && (!(gpu.CurPolygonAttr & (1<<12)))) return 0;

            Vertex* vprev = &vertices[prev];
            if (vprev->Position[comp] <= vprev->Position[3])
            {
                ClipSegment<comp, 1, attribs>(&temp[c], &vtx, vprev);
                c++;
            }

            Vertex* vnext = &vertices[next];
            if (vnext->Position[comp] <= vnext->Position[3])
            {
                ClipSegment<comp, 1, attribs>(&temp[c], &vtx, vnext);
                c++;
            }
        }
        else
            temp[c++] = vtx;
    }

    nverts = c; c = clipstart;
    for (int i = clipstart; i < nverts; i++)
    {
        prev = i-1; if (prev < 0) prev = nverts-1;
        next = i+1; if (next >= nverts) next = 0;

        Vertex vtx = temp[i];
        if (vtx.Position[comp] < -vtx.Position[3])
        {
            Vertex* vprev = &temp[prev];
            if (vprev->Position[comp] >= -vprev->Position[3])
            {
                ClipSegment<comp, -1, attribs>(&vertices[c], &vtx, vprev);
                c++;
            }

            Vertex* vnext = &temp[next];
            if (vnext->Position[comp] >= -vnext->Position[3])
            {
                ClipSegment<comp, -1, attribs>(&vertices[c], &vtx, vnext);
                c++;
            }
        }
        else
            vertices[c++] = vtx;
    }

    // checkme
    for (int i = 0; i < c; i++)
    {
        Vertex* vtx = &vertices[i];

        vtx->Color[0] &= ~0xFFF; vtx->Color[0] += 0xFFF;
        vtx->Color[1] &= ~0xFFF; vtx->Color[1] += 0xFFF;
        vtx->Color[2] &= ~0xFFF; vtx->Color[2] += 0xFFF;
    }

    return c;
}

template<bool attribs>
int ClipPolygon(GPU3D& gpu, Vertex* vertices, int nverts, int clipstart)
{
    // clip.
    // for each vertex:
    // if it's outside, check if the previous and next vertices are inside
    // if so, place a new vertex at the edge of the view volume

    // TODO: check for 1-dot polygons
    // TODO: the hardware seems to use a different algorithm. it reacts differently to vertices with W=0
    // some vertices that should get Y=-0x1000 get Y=0x1000 for some reason on hardware. it doesn't make sense.
    // clipping seems to process the Y plane before the X plane.

    // Z clipping
    nverts = ClipAgainstPlane<2, attribs>(gpu, vertices, nverts, clipstart);

    // Y clipping
    nverts = ClipAgainstPlane<1, attribs>(gpu, vertices, nverts, clipstart);

    // X clipping
    nverts = ClipAgainstPlane<0, attribs>(gpu, vertices, nverts, clipstart);

    return nverts;
}

bool ClipCoordsEqual(Vertex* a, Vertex* b)
{
    return a->Position[0] == b->Position[0] &&
           a->Position[1] == b->Position[1] &&
           a->Position[2] == b->Position[2] &&
           a->Position[3] == b->Position[3];
}

#ifdef LITEV_GX_CLIP_REJECT
// ClipPolygon returns 0 for these without the copy passes (clipstart 0 only: kept strip vertices
// make the clipper return at least 2). Planes in its order (Z, Y, X); the first one with a vertex
// outside decides: every vertex past the same side -> 0 (the + pass, or the - pass after the +
// pass copied them all), a vertex past the far plane without attr bit 12 -> 0; anything else clips.
// Planes with no vertex outside leave the positions as they are.
static bool ClipTrivialReject(const Vertex* v, int n, u32 attr)
{
    for (int comp = 2; comp >= 0; comp--)
    {
        bool anyOut = false, allPlus = true, allMinus = true;
        for (int i = 0; i < n; i++)
        {
            const s32 p = v[i].Position[comp], w = v[i].Position[3];
            const bool plus = p > w, minus = !plus && p < -w;
            if (comp == 2 && plus && !(attr & (1<<12))) return true;
            anyOut |= plus | minus; allPlus &= plus; allMinus &= minus;
        }
        if (anyOut) return allPlus || allMinus;
    }
    return false;
}
#endif

void GPU3D::SubmitPolygon() noexcept
{
#ifdef LITEV_GX_VTX_PREFETCH
    // the polygon's vertices are stored at CurVertexRAM[NumVertices..] (a 384 KB bank, colder than
    // the caches): start the write-allocate of the next lines now, while it clips (no state change)
    if (NumVertices + 8 <= 6144)
    {
        const char* p = (const char*)&CurVertexRAM[NumVertices];
        __builtin_prefetch(p + 64, 1, 3); __builtin_prefetch(p + 128, 1, 3);
        __builtin_prefetch(p + 192, 1, 3); __builtin_prefetch(p + 256, 1, 3);
    }
#endif
    Vertex clippedvertices[10];
    Vertex* reusedvertices[2];
    int clipstart = 0;
    int lastpolyverts = 0;

    int nverts = PolygonMode & 0x1 ? 4:3;
    int prev, next;

    // submitting a polygon starts the polygon pipeline
    // noting that for now we are only reserving one vertex slot
    // further slots only get reserved if the polygon makes it through culling/clipping
    PolygonPipeline = 8;
    VertexSlotCounter = 1;
    VertexSlotsFree = 0b11110;

    // culling
    // TODO: work out how it works on the real thing
    // the normalization part is a wild guess

    Vertex *v0, *v1, *v2, *v3;
    s64 normalX, normalY, normalZ;
    s64 dot;

    v0 = &TempVertexBuffer[0];
    v1 = &TempVertexBuffer[1];
    v2 = &TempVertexBuffer[2];
    v3 = &TempVertexBuffer[3];

#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
    // LITEV_GEOM_NEON2: the per-polygon backface-cull normal is a (x,y,w) cross
    // product — six s64 widening products folded into two vmull_s32 pairs + one
    // scalar lane (see NeonCullNormal, bit-exact vs the scalar form below).
    {
        s64 nrm[3];
        NeonCullNormal(v0->Position, v1->Position, v2->Position, nrm);
        normalX = nrm[0]; normalY = nrm[1]; normalZ = nrm[2];
    }
#else
    normalX = ((s64)(v0->Position[1]-v1->Position[1]) * (v2->Position[3]-v1->Position[3]))
        - ((s64)(v0->Position[3]-v1->Position[3]) * (v2->Position[1]-v1->Position[1]));
    normalY = ((s64)(v0->Position[3]-v1->Position[3]) * (v2->Position[0]-v1->Position[0]))
        - ((s64)(v0->Position[0]-v1->Position[0]) * (v2->Position[3]-v1->Position[3]));
    normalZ = ((s64)(v0->Position[0]-v1->Position[0]) * (v2->Position[1]-v1->Position[1]))
        - ((s64)(v0->Position[1]-v1->Position[1]) * (v2->Position[0]-v1->Position[0]));
#endif

    while ((((normalX>>31) ^ (normalX>>63)) != 0) ||
           (((normalY>>31) ^ (normalY>>63)) != 0) ||
           (((normalZ>>31) ^ (normalZ>>63)) != 0))
    {
        normalX >>= 4;
        normalY >>= 4;
        normalZ >>= 4;
    }

    // after normalization normalX/Y/Z all fit in s32 (the loop shifts until
    // (n>>31)^(n>>63)==0), so v1coord*normal is an exact s32*s32->s64 product.
#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
    {
        int32x2_t vv = { v1->Position[0], v1->Position[1] };
        int32x2_t nn = { (s32)normalX,   (s32)normalY };
        int64x2_t dp = vmull_s32(vv, nn);   // {v1x*normalX, v1y*normalY}
        dot = vgetq_lane_s64(dp, 0) + vgetq_lane_s64(dp, 1)
            + ((s64)v1->Position[3] * (s32)normalZ);
    }
#else
    dot = ((s64)v1->Position[0] * normalX) + ((s64)v1->Position[1] * normalY) + ((s64)v1->Position[3] * normalZ);
#endif

    bool facingview = (dot <= 0);

    if (dot < 0)
    {
        if (!(CurPolygonAttr & (1<<7)))
        {
            LastStripPolygon = NULL;
            return;
        }
    }
    else if (dot > 0)
    {
        if (!(CurPolygonAttr & (1<<6)))
        {
            LastStripPolygon = NULL;
            return;
        }
    }

    // for strips, check whether we can attach to the previous polygon
    // this requires two original vertices shared with the previous polygon, and that
    // the two polygons be of the same type

    if (PolygonMode >= 2 && LastStripPolygon)
    {
        int id0, id1;
        if (PolygonMode == 2)
        {
            if (NumConsecutivePolygons & 1)
            {
                id0 = 2;
                id1 = 1;
            }
            else
            {
                id0 = 0;
                id1 = 2;
            }

            lastpolyverts = 3;
        }
        else
        {
            id0 = 3;
            id1 = 2;

            lastpolyverts = 4;
        }

        if (LastStripPolygon->NumVertices == lastpolyverts &&
            !LastStripPolygon->Vertices[id0]->Clipped &&
            !LastStripPolygon->Vertices[id1]->Clipped)
        {
            reusedvertices[0] = LastStripPolygon->Vertices[id0];
            reusedvertices[1] = LastStripPolygon->Vertices[id1];

            clippedvertices[0] = *reusedvertices[0];
            clippedvertices[1] = *reusedvertices[1];

            clipstart = 2;
        }
    }

    for (int i = clipstart; i < nverts; i++)
        clippedvertices[i] = TempVertexBuffer[i];

    // detect lines, for the OpenGL renderer

    int polytype = 0;
    if (nverts == 3)
    {
        if (ClipCoordsEqual(&clippedvertices[0], &clippedvertices[1]) ||
            ClipCoordsEqual(&clippedvertices[0], &clippedvertices[2]) ||
            ClipCoordsEqual(&clippedvertices[1], &clippedvertices[2]))
        {
            polytype = 1;
        }
    }
    else if (nverts == 4)
    {
        // TODO
    }

    // clipping

#if defined(__ANDROID__)
    // Trivial-accept fast path (debug.litev.trivclip): ClipPolygon runs UNCONDITIONALLY, but a
    // polygon fully inside all 6 frustum planes emerges from the full 3-plane/2-pass clip byte-
    // identical to its input EXCEPT for the 5-bit vertex-colour clamp ClipAgainstPlane applies to
    // every output vertex (idempotent, applied 3x -> once suffices). Detect the fully-inside case
    // with the clipper's OWN comparisons (pos>W is its +plane test, pos<-W its -plane test) and,
    // when it holds for every vertex (with W>0 so the plane test is well-defined; W<=0 defers to the
    // faithful full clipper), apply only the clamp and skip the six copy passes. nverts and vertex
    // order are unchanged -> guest RAM_COUNT identical -> MP-safe. Validate via headless
    // record/verify-trace (byte-exact) before default-on. NOTE: leaves Clipped flags untouched,
    // exactly as the clipper's else-branch does for unclipped vertices.
    // Default ON: proven byte-exact (900-frame race trace) and removes ~1.9% of emu-thread
    // self-time (the three ClipAgainstPlane passes vanish from the profile). debug.litev.trivclip=0
    // disables it for A/B.
    static const int _trivclip = litevGxPropDefault("debug.litev.trivclip", 1);
    bool _trivial = (_trivclip != 0);
    if (_trivial)
    {
        for (int i = 0; i < nverts; i++)
        {
            const s32 w = clippedvertices[i].Position[3];
            const s32 x = clippedvertices[i].Position[0];
            const s32 y = clippedvertices[i].Position[1];
            const s32 z = clippedvertices[i].Position[2];
            if (w <= 0 || x > w || x < -w || y > w || y < -w || z > w || z < -w)
            { _trivial = false; break; }
        }
    }
    if (_trivial)
    {
        for (int i = 0; i < nverts; i++)
        {
            Vertex* vtx = &clippedvertices[i];
            vtx->Color[0] = (vtx->Color[0] & ~0xFFF) + 0xFFF;
            vtx->Color[1] = (vtx->Color[1] & ~0xFFF) + 0xFFF;
            vtx->Color[2] = (vtx->Color[2] & ~0xFFF) + 0xFFF;
        }
        // nverts unchanged; skip ClipPolygon.
    }
    else
#endif
#ifdef LITEV_GX_CLIP_REJECT
    if (clipstart == 0 && ClipTrivialReject(clippedvertices, nverts, CurPolygonAttr)) nverts = 0;
    else
#endif
    nverts = ClipPolygon<true>(*this, clippedvertices, nverts, clipstart);
    if (nverts == 0)
    {
        LastStripPolygon = NULL;
        return;
    }

    // reject the polygon if it's not going to fit in polygon/vertex RAM

    if (NumPolygons >= 2048 || NumVertices+nverts > 6144)
    {
        LastStripPolygon = NULL;
        DispCnt |= (1<<13);
        return;
    }

    // compute screen coordinates

    for (int i = clipstart; i < nverts; i++)
    {
        Vertex* vtx = &clippedvertices[i];

        // W is truncated to 24 bits at this point
        // if this W is zero, the polygon isn't rendered
        vtx->Position[3] &= 0x00FFFFFF;

        // viewport transform
        // note: the DS performs these divisions using a 32-bit divider
        // thus, if W is greater than 0xFFFF, some precision is sacrificed
        // to make the numbers fit into the divider
        u32 posX, posY;
        u32 w = vtx->Position[3];
        if (w == 0)
        {
            posX = 0;
            posY = 0;
        }
#ifdef LITEV_GX_POLY_LEAN
        // common case (w <= 0xFFFF, vertex inside the clip volume): one 32-bit divide gives the
        // hi-res position, and the 9/8-bit position is it >> 4 (floor(floor(16n/d)/16) =
        // floor(n/d); 16 * 2w * 511 < 2^32, so nothing wraps). Same values as below.
        else if (w <= 0xFFFF && (u32)(vtx->Position[0] + (s32)w) <= 2 * w && (u32)(-vtx->Position[1] + (s32)w) <= 2 * w
                 && !LITEV_HEADLESS(Headless))
        {
            const u32 den = w << 1;
            const u32 hx = (((u32)(vtx->Position[0] + (s32)w) * Viewport[4]) << 4) / den;
            const u32 hy = (((u32)(-vtx->Position[1] + (s32)w) * Viewport[5]) << 4) / den;
            vtx->FinalPosition[0] = ((hx >> 4) + Viewport[0]) & 0x1FF;
            vtx->FinalPosition[1] = ((hy >> 4) + Viewport[3]) & 0xFF;
            vtx->HiresPosition[0] = (hx + (Viewport[0] << 4)) & 0x1FFF;
            vtx->HiresPosition[1] = (hy + (Viewport[3] << 4)) & 0xFFF;
            continue;
        }
#endif
        else
        {
            posX = vtx->Position[0] + w;
            posY = -vtx->Position[1] + w;
            u32 den = w;

            if (w > 0xFFFF)
            {
                posX >>= 1;
                posY >>= 1;
                den  >>= 1;
            }

            den <<= 1;
            posX = ((posX * Viewport[4]) / den) + Viewport[0];
            posY = ((posY * Viewport[5]) / den) + Viewport[3];
        }

        vtx->FinalPosition[0] = posX & 0x1FF;
        vtx->FinalPosition[1] = posY & 0xFF;

        // hi-res positions
        // to consider: only do this when using the GL renderer? apply the aforementioned quirk to this?
        if (w != 0 && !LITEV_HEADLESS(Headless))   // (two 64-bit divides per vertex, only for the hi-res renderer)
        {
            posX = ((((s64)(vtx->Position[0] + w) * Viewport[4]) << 4) / (((s64)w) << 1)) + (Viewport[0] << 4);
            posY = ((((s64)(-vtx->Position[1] + w) * Viewport[5]) << 4) / (((s64)w) << 1)) + (Viewport[3] << 4);

            vtx->HiresPosition[0] = posX & 0x1FFF;
            vtx->HiresPosition[1] = posY & 0xFFF;
        }
    }

    // zero-dot W check:
    // * if the polygon's vertices all have the same screen coordinates, it is considered to be zero-dot
    // * if all the vertices have a W greater than the threshold defined in register 0x04000610,
    //   the polygon is rejected, unless bit13 in the polygon attributes is set

    if (!(CurPolygonAttr & (1<<13)))
    {
        bool zerodot = true;
        bool allbehind = true;

        for (int i = 0; i < nverts; i++)
        {
            Vertex* vtx = &clippedvertices[i];

            if (vtx->FinalPosition[0] != clippedvertices[0].FinalPosition[0] ||
                vtx->FinalPosition[1] != clippedvertices[0].FinalPosition[1])
            {
                zerodot = false;
                break;
            }

            if (vtx->Position[3] <= ZeroDotWLimit)
            {
                allbehind = false;
                break;
            }
        }

        if (zerodot && allbehind)
        {
            LastStripPolygon = NULL;
            return;
        }
    }

    // build the actual polygon

    if (nverts == 4)
    {
        PolygonPipeline = 35;
        VertexSlotCounter = 1;
        if (PolygonMode & 0x2) VertexSlotsFree = 0b11100;
        else                   VertexSlotsFree = 0b11110;
    }
    else
    {
        PolygonPipeline = 26;
        VertexSlotCounter = 1;
        if (PolygonMode & 0x2) VertexSlotsFree = 0b1000;
        else                   VertexSlotsFree = 0b1110;
    }

    Polygon* poly = &CurPolygonRAM[NumPolygons++];
    poly->NumVertices = 0;

    poly->Attr = CurPolygonAttr;
    poly->TexParam = TexParam;
    poly->TexPalette = TexPalette;

    poly->Degenerate = false;
    poly->Type = 0;

    poly->FacingView = facingview;

    u32 texfmt = (TexParam >> 26) & 0x7;
    u32 polyalpha = (CurPolygonAttr >> 16) & 0x1F;
    poly->Translucent = (texfmt == 1 || texfmt == 6) || (polyalpha > 0 && polyalpha < 31);

    poly->IsShadowMask = ((CurPolygonAttr & 0x3F000030) == 0x00000030);
    poly->IsShadow = ((CurPolygonAttr & 0x30) == 0x30) && !poly->IsShadowMask;

    if (!poly->Translucent) NumOpaquePolygons++;

    poly->Type = polytype;

    if (LastStripPolygon && clipstart > 0)
    {
        if (nverts == lastpolyverts)
        {
            poly->Vertices[0] = reusedvertices[0];
            poly->Vertices[1] = reusedvertices[1];
        }
        else
        {
            Vertex v0 = *reusedvertices[0];
            Vertex v1 = *reusedvertices[1];

            CurVertexRAM[NumVertices] = v0;
            poly->Vertices[0] = &CurVertexRAM[NumVertices];
            CurVertexRAM[NumVertices+1] = v1;
            poly->Vertices[1] = &CurVertexRAM[NumVertices+1];
            NumVertices += 2;
        }

        poly->NumVertices += 2;
    }

    for (int i = clipstart; i < nverts; i++)
    {
        Vertex* vtx = &CurVertexRAM[NumVertices];
        *vtx = clippedvertices[i];
        poly->Vertices[i] = vtx;

        NumVertices++;
        poly->NumVertices++;

        if (LITEV_HEADLESS(Headless)) continue;  // colours below only feed rendering
        vtx->FinalColor[0] = vtx->Color[0] >> 12;
        if (vtx->FinalColor[0]) vtx->FinalColor[0] = ((vtx->FinalColor[0] << 4) + 0xF);
        vtx->FinalColor[1] = vtx->Color[1] >> 12;
        if (vtx->FinalColor[1]) vtx->FinalColor[1] = ((vtx->FinalColor[1] << 4) + 0xF);
        vtx->FinalColor[2] = vtx->Color[2] >> 12;
        if (vtx->FinalColor[2]) vtx->FinalColor[2] = ((vtx->FinalColor[2] << 4) + 0xF);
    }

    // A console nobody watches stops here: bounds, sort key and per-vertex depth (a 64-bit
    // divide each) only feed rendering. Counts, clipping and strip linking above are emulated.
    if (LITEV_HEADLESS(Headless))
    {
        LastStripPolygon = (PolygonMode >= 2) ? poly : NULL;
#ifdef LITEV_FF_HEADLESS3D
        BankBuiltHeadless = true;
#endif
        return;
    }

    // determine bounds of the polygon
    // also determine the W shift and normalize W
    // normalization works both ways
    // (ie two W's that span 12 bits or less will be brought to 16 bits)

    u32 vtop = 0, vbot = 0;
    s32 ytop = 192, ybot = 0;
    s32 xtop = 256, xbot = 0;
    u32 wsize = 0;

    for (int i = 0; i < nverts; i++)
    {
        Vertex* vtx = poly->Vertices[i];

        if (vtx->FinalPosition[1] < ytop)
        {
            xtop = vtx->FinalPosition[0];
            ytop = vtx->FinalPosition[1];
            vtop = i;
        }
        if (vtx->FinalPosition[1] > ybot || (vtx->FinalPosition[1] == ybot && vtx->FinalPosition[0] > xbot))
        {
            xbot = vtx->FinalPosition[0];
            ybot = vtx->FinalPosition[1];
            vbot = i;
        }

        u32 w = (u32)vtx->Position[3];
        if (w == 0) poly->Degenerate = true;

        while ((w >> wsize) && (wsize < 32))
            wsize += 4;
    }

    poly->VTop = vtop; poly->VBottom = vbot;
    poly->YTop = ytop; poly->YBottom = ybot;
    poly->XTop = xtop; poly->XBottom = xbot;

    if (ybot > 192) poly->Degenerate = true;

    poly->SortKey = (ybot << 8) | ytop;
    if (poly->Translucent) poly->SortKey |= 0x10000;

    poly->WBuffer = (FlushAttributes & 0x2);

    for (int i = 0; i < nverts; i++)
    {
        Vertex* vtx = poly->Vertices[i];
        s32 w, wshifted;

        // W is normalized, such that all the polygon's W values fit within 16 bits
        // the viewport transform for X/Y/Z uses the original W values, but
        // when W-buffering is used, the normalized W is used
        // W normalization is applied to separate polygons, even within strips

        if (wsize < 16)
        {
            w = vtx->Position[3] << (16 - wsize);
            wshifted = w >> (16 - wsize);
        }
        else
        {
            w = vtx->Position[3] >> (wsize - 16);
            wshifted = w << (wsize - 16);
        }

        s32 z;
        if (FlushAttributes & 0x2)
            z = wshifted;
        else if (vtx->Position[3])
            z = ((((s64)vtx->Position[2] * 0x4000) / vtx->Position[3]) + 0x3FFF) * 0x200;
        else
            z = 0x7FFE00;

        // checkme (Z<0 shouldn't be possible, but Z>0xFFFFFF is possible)
        if (z < 0) z = 0;
        else if (z > 0xFFFFFF) z = 0xFFFFFF;

        poly->FinalZ[i] = z;
        poly->FinalW[i] = w;
    }

    if (PolygonMode >= 2)
        LastStripPolygon = poly;
    else
        LastStripPolygon = NULL;
}

void GPU3D::SubmitVertex() noexcept
{
    s64 vertex[4] = {(s64)CurVertex[0], (s64)CurVertex[1], (s64)CurVertex[2], 0x1000};
    Vertex* vertextrans = &TempVertexBuffer[VertexNumInPoly];

    UpdateClipMatrix();
#if defined(LITEV_NEON_GEOMETRY) && defined(__ARM_NEON)
    // vertex[] components fit in s32 (CurVertex is s16, w = 0x1000), so the
    // widening 32x32->64 NEON multiply matches the scalar s64 products exactly.
    NeonMat4Vec4_s64<12>(vertextrans->Position, ClipMatrix,
                         (s32)vertex[0], (s32)vertex[1], (s32)vertex[2], (s32)vertex[3]);
#else
    vertextrans->Position[0] = (vertex[0]*ClipMatrix[0] + vertex[1]*ClipMatrix[4] + vertex[2]*ClipMatrix[8] + vertex[3]*ClipMatrix[12]) >> 12;
    vertextrans->Position[1] = (vertex[0]*ClipMatrix[1] + vertex[1]*ClipMatrix[5] + vertex[2]*ClipMatrix[9] + vertex[3]*ClipMatrix[13]) >> 12;
    vertextrans->Position[2] = (vertex[0]*ClipMatrix[2] + vertex[1]*ClipMatrix[6] + vertex[2]*ClipMatrix[10] + vertex[3]*ClipMatrix[14]) >> 12;
    vertextrans->Position[3] = (vertex[0]*ClipMatrix[3] + vertex[1]*ClipMatrix[7] + vertex[2]*ClipMatrix[11] + vertex[3]*ClipMatrix[15]) >> 12;
#endif

    // this probably shouldn't be.
    // the way color is handled during clipping needs investigation. TODO
    vertextrans->Color[0] = (VertexColor[0] << 12) + 0xFFF;
    vertextrans->Color[1] = (VertexColor[1] << 12) + 0xFFF;
    vertextrans->Color[2] = (VertexColor[2] << 12) + 0xFFF;

    if ((TexParam >> 30) == 3)
    {
#if defined(LITEV_NEON_GEOMETRY) && defined(__ARM_NEON)
        int64x2_t tc = NeonTex2_s64<24>(TexMatrix, (s32)vertex[0], (s32)vertex[1], (s32)vertex[2]);
        vertextrans->TexCoords[0] = (s32)vgetq_lane_s64(tc, 0) + RawTexCoords[0];
        vertextrans->TexCoords[1] = (s32)vgetq_lane_s64(tc, 1) + RawTexCoords[1];
#else
        vertextrans->TexCoords[0] = ((vertex[0]*TexMatrix[0] + vertex[1]*TexMatrix[4] + vertex[2]*TexMatrix[8]) >> 24) + RawTexCoords[0];
        vertextrans->TexCoords[1] = ((vertex[0]*TexMatrix[1] + vertex[1]*TexMatrix[5] + vertex[2]*TexMatrix[9]) >> 24) + RawTexCoords[1];
#endif
    }
    else
    {
        vertextrans->TexCoords[0] = TexCoords[0];
        vertextrans->TexCoords[1] = TexCoords[1];
    }

    vertextrans->Clipped = false;

    VertexNum++;
    VertexNumInPoly++;

    switch (PolygonMode)
    {
    case 0: // triangle
        if (VertexNumInPoly == 3)
        {
            VertexNumInPoly = 0;
            SubmitPolygon();
            NumConsecutivePolygons++;
        }
        break;

    case 1: // quad
        if (VertexNumInPoly == 4)
        {
            VertexNumInPoly = 0;
            SubmitPolygon();
            NumConsecutivePolygons++;
        }
        break;

    case 2: // triangle strip
        if (NumConsecutivePolygons & 1)
        {
            Vertex tmp = TempVertexBuffer[1];
            TempVertexBuffer[1] = TempVertexBuffer[0];
            TempVertexBuffer[0] = tmp;

            VertexNumInPoly = 2;
            SubmitPolygon();
            NumConsecutivePolygons++;

            TempVertexBuffer[1] = TempVertexBuffer[2];
        }
        else if (VertexNumInPoly == 3)
        {
            VertexNumInPoly = 2;
            SubmitPolygon();
            NumConsecutivePolygons++;

            TempVertexBuffer[0] = TempVertexBuffer[1];
            TempVertexBuffer[1] = TempVertexBuffer[2];
        }
        break;

    case 3: // quad strip
        if (VertexNumInPoly == 4)
        {
            Vertex tmp = TempVertexBuffer[3];
            TempVertexBuffer[3] = TempVertexBuffer[2];
            TempVertexBuffer[2] = tmp;

            VertexNumInPoly = 2;
            SubmitPolygon();
            NumConsecutivePolygons++;

            TempVertexBuffer[0] = TempVertexBuffer[3];
            TempVertexBuffer[1] = TempVertexBuffer[2];
        }
        break;
    }

    VertexPipeline = 7;
    AddCycles(3);
}

void GPU3D::CalculateLighting() noexcept
{
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LightingCalls);

#ifdef LITEV_FF_HEADLESS3D
    if (LITEV_HEADLESS(Headless) && !FFHeadlessNow)
#else
    if (LITEV_HEADLESS(Headless))
#endif
    {
        s32 c = __builtin_popcount(CurPolygonAttr & 0xF);
        NormalPipeline = 7;
        AddCycles(c < 1 ? 1 : c);
        return;
    }

    if ((TexParam >> 30) == 2)
    {
#if defined(LITEV_NEON_GEOMETRY) && defined(__ARM_NEON)
        // Same 3-term/2-output, 64-bit, >>21 shape as the position texcoord gen.
        int64x2_t tc = NeonTex2_s64<21>(TexMatrix, (s32)Normal[0], (s32)Normal[1], (s32)Normal[2]);
        TexCoords[0] = RawTexCoords[0] + (s32)vgetq_lane_s64(tc, 0);
        TexCoords[1] = RawTexCoords[1] + (s32)vgetq_lane_s64(tc, 1);
#else
        TexCoords[0] = RawTexCoords[0] + (((s64)Normal[0]*TexMatrix[0] + (s64)Normal[1]*TexMatrix[4] + (s64)Normal[2]*TexMatrix[8]) >> 21);
        TexCoords[1] = RawTexCoords[1] + (((s64)Normal[0]*TexMatrix[1] + (s64)Normal[1]*TexMatrix[5] + (s64)Normal[2]*TexMatrix[9]) >> 21);
#endif
    }

    s32 normaltrans[3]; // should be 1 bit sign 10 bits frac
#if defined(LITEV_NEON_GEOMETRY) && defined(__ARM_NEON)
    // 32-bit math (no s64 cast in the scalar reference): the products and the
    // 3-term sum are computed modulo 2^32, then << 9 (modular) and >> 21
    // (arithmetic). vmulq_n_s32/vmlaq_n_s32 keep the low 32 bits per lane, and
    // vshlq/vshrq_n_s32 match the scalar shift semantics exactly. Lane 3 is
    // computed from VecMatrix[3/7/11] but discarded.
    {
        int32x4_t nt = vmulq_n_s32(vld1q_s32(VecMatrix + 0), (s32)Normal[0]);
        nt = vmlaq_n_s32(nt, vld1q_s32(VecMatrix + 4), (s32)Normal[1]);
        nt = vmlaq_n_s32(nt, vld1q_s32(VecMatrix + 8), (s32)Normal[2]);
        nt = vshrq_n_s32(vshlq_n_s32(nt, 9), 21);
        normaltrans[0] = vgetq_lane_s32(nt, 0);
        normaltrans[1] = vgetq_lane_s32(nt, 1);
        normaltrans[2] = vgetq_lane_s32(nt, 2);
    }
#else
    normaltrans[0] = ((Normal[0]*VecMatrix[0] + Normal[1]*VecMatrix[4] + Normal[2]*VecMatrix[8]) << 9) >> 21;
    normaltrans[1] = ((Normal[0]*VecMatrix[1] + Normal[1]*VecMatrix[5] + Normal[2]*VecMatrix[9]) << 9) >> 21;
    normaltrans[2] = ((Normal[0]*VecMatrix[2] + Normal[1]*VecMatrix[6] + Normal[2]*VecMatrix[10]) << 9) >> 21;
#endif

    s32 c = 0;
    u32 vtxbuff[3] =
    {
        (u32)MatEmission[0] << 14,
        (u32)MatEmission[1] << 14,
        (u32)MatEmission[2] << 14
    };
    for (int i = 0; i < 4; i++)
    {
        if (!(CurPolygonAttr & (1<<i)))
            continue;

        // (credit to azusa for working out most of the details of the diff. algorithm, and essentially the entire spec. algorithm)
        
        // calculate dot product
        // bottom 9 bits are discarded after multiplying and before adding
        s32 dot = ((LightDirection[i][0]*normaltrans[0]) >> 9) +
                  ((LightDirection[i][1]*normaltrans[1]) >> 9) +
                  ((LightDirection[i][2]*normaltrans[2]) >> 9);

        s32 shinelevel;
        if (dot > 0) 
        {
            // -- diffuse lighting --
            
            // convert dot to signed 11 bit int
            // then we truncate the result of the multiplications to an unsigned 20 bits before adding to the vtx color
            s32 diffdot = (dot << 21) >> 21;
            vtxbuff[0] += (MatDiffuse[0] * LightColor[i][0] * diffdot) & 0xFFFFF;
            vtxbuff[1] += (MatDiffuse[1] * LightColor[i][1] * diffdot) & 0xFFFFF;
            vtxbuff[2] += (MatDiffuse[2] * LightColor[i][2] * diffdot) & 0xFFFFF;

            // -- specular lighting --
        
            // reuse the dot product from diffuse lighting
            dot += normaltrans[2];

            // convert to s11, then square it, and truncate to 10 bits
            dot = (dot << 21) >> 21;
            dot = ((dot * dot) >> 10) & 0x3FF;

            // multiply dot and reciprocal, the subtract '1'
            shinelevel = ((dot * SpecRecip[i]) >> 8) - (1<<9);

            if (shinelevel < 0) shinelevel = 0;
            else
            {
                // sign extend to convert to signed 14 bit integer
                shinelevel = (shinelevel << 18) >> 18;
                if (shinelevel < 0) shinelevel = 0; // for some reason there seems to be a redundant check for <0?
                else if (shinelevel > 0x1FF) shinelevel = 0x1FF;
            }
        }
        else shinelevel = 0;

        // convert shinelevel to use for lookup in the shininess table if enabled.
        if (UseShininessTable)
        {
            shinelevel >>= 2;
            shinelevel = ShininessTable[shinelevel];
            shinelevel <<= 1;
        }

        // Note: ambient seems to be a plain bitshift
        vtxbuff[0] += ((MatSpecular[0] * shinelevel) + (MatAmbient[0] << 9)) * LightColor[i][0];
        vtxbuff[1] += ((MatSpecular[1] * shinelevel) + (MatAmbient[1] << 9)) * LightColor[i][1];
        vtxbuff[2] += ((MatSpecular[2] * shinelevel) + (MatAmbient[2] << 9)) * LightColor[i][2];

        c++;
    }

    VertexColor[0] = (vtxbuff[0] >> 14 > 31) ? 31 : (vtxbuff[0] >> 14);
    VertexColor[1] = (vtxbuff[1] >> 14 > 31) ? 31 : (vtxbuff[1] >> 14);
    VertexColor[2] = (vtxbuff[2] >> 14 > 31) ? 31 : (vtxbuff[2] >> 14);

    if (c < 1) c = 1;
    NormalPipeline = 7;
    AddCycles(c);
}


void GPU3D::BoxTest(const u32* params) noexcept
{
    Vertex cube[8];
    Vertex face[10];
    int res;

    AddCycles(254);
    GXStat &= ~(1<<1);

    s16 x0 = (s16)(params[0] & 0xFFFF);
    s16 y0 = ((s32)params[0]) >> 16;
    s16 z0 = (s16)(params[1] & 0xFFFF);
    s16 x1 = ((s32)params[1]) >> 16;
    s16 y1 = (s16)(params[2] & 0xFFFF);
    s16 z1 = ((s32)params[2]) >> 16;

    x1 += x0;
    y1 += y0;
    z1 += z0;

    cube[0].Position[0] = x0; cube[0].Position[1] = y0; cube[0].Position[2] = z0;
    cube[1].Position[0] = x1; cube[1].Position[1] = y0; cube[1].Position[2] = z0;
    cube[2].Position[0] = x1; cube[2].Position[1] = y1; cube[2].Position[2] = z0;
    cube[3].Position[0] = x0; cube[3].Position[1] = y1; cube[3].Position[2] = z0;
    cube[4].Position[0] = x0; cube[4].Position[1] = y1; cube[4].Position[2] = z1;
    cube[5].Position[0] = x0; cube[5].Position[1] = y0; cube[5].Position[2] = z1;
    cube[6].Position[0] = x1; cube[6].Position[1] = y0; cube[6].Position[2] = z1;
    cube[7].Position[0] = x1; cube[7].Position[1] = y1; cube[7].Position[2] = z1;

    UpdateClipMatrix();
    for (int i = 0; i < 8; i++)
    {
        s32 x = cube[i].Position[0];
        s32 y = cube[i].Position[1];
        s32 z = cube[i].Position[2];

#if defined(LITEV_GEOM_NEON3) && defined(__ARM_NEON)
        // Identical clip-matrix transform to the per-vertex path in SubmitVertex
        // (w = 0x1000). x/y/z are captured above so out=cube[i].Position does not
        // alias the scalar inputs, and ClipMatrix (M) is distinct storage. Reuses
        // the proven NeonMat4Vec4_s64<12> kernel -> bit-exact vs the scalar below.
        NeonMat4Vec4_s64<12>(cube[i].Position, ClipMatrix, x, y, z, 0x1000);
#else
        cube[i].Position[0] = ((s64)x*ClipMatrix[0] + (s64)y*ClipMatrix[4] + (s64)z*ClipMatrix[8] + (s64)0x1000*ClipMatrix[12]) >> 12;
        cube[i].Position[1] = ((s64)x*ClipMatrix[1] + (s64)y*ClipMatrix[5] + (s64)z*ClipMatrix[9] + (s64)0x1000*ClipMatrix[13]) >> 12;
        cube[i].Position[2] = ((s64)x*ClipMatrix[2] + (s64)y*ClipMatrix[6] + (s64)z*ClipMatrix[10] + (s64)0x1000*ClipMatrix[14]) >> 12;
        cube[i].Position[3] = ((s64)x*ClipMatrix[3] + (s64)y*ClipMatrix[7] + (s64)z*ClipMatrix[11] + (s64)0x1000*ClipMatrix[15]) >> 12;
#endif
    }

    // front face (-Z)
    face[0] = cube[0]; face[1] = cube[1]; face[2] = cube[2]; face[3] = cube[3];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }

    // back face (+Z)
    face[0] = cube[4]; face[1] = cube[5]; face[2] = cube[6]; face[3] = cube[7];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }

    // left face (-X)
    face[0] = cube[0]; face[1] = cube[3]; face[2] = cube[4]; face[3] = cube[5];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }

    // right face (+X)
    face[0] = cube[1]; face[1] = cube[2]; face[2] = cube[7]; face[3] = cube[6];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }

    // bottom face (-Y)
    face[0] = cube[0]; face[1] = cube[1]; face[2] = cube[6]; face[3] = cube[5];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }

    // top face (+Y)
    face[0] = cube[2]; face[1] = cube[3]; face[2] = cube[4]; face[3] = cube[7];
    res = ClipPolygon<false>(*this, face, 4, 0);
    if (res > 0)
    {
        GXStat |= (1<<1);
        return;
    }
}

void GPU3D::PosTest() noexcept
{
    s64 vertex[4] = {(s64)CurVertex[0], (s64)CurVertex[1], (s64)CurVertex[2], 0x1000};

    UpdateClipMatrix();
#if defined(LITEV_GEOM_NEON3) && defined(__ARM_NEON)
    // Same clip-matrix transform as SubmitVertex/BoxTest: CurVertex is s16 (fits
    // s32), w = 0x1000, PosTestResult is s32[4]. Reuses the proven kernel; the
    // scalar's (s64)vertex[i] widening equals the kernel's 32x32->64 mul since the
    // inputs fit in s32. Bit-exact.
    NeonMat4Vec4_s64<12>(PosTestResult, ClipMatrix,
                         (s32)vertex[0], (s32)vertex[1], (s32)vertex[2], (s32)vertex[3]);
#else
    PosTestResult[0] = (vertex[0]*ClipMatrix[0] + vertex[1]*ClipMatrix[4] + vertex[2]*ClipMatrix[8] + vertex[3]*ClipMatrix[12]) >> 12;
    PosTestResult[1] = (vertex[0]*ClipMatrix[1] + vertex[1]*ClipMatrix[5] + vertex[2]*ClipMatrix[9] + vertex[3]*ClipMatrix[13]) >> 12;
    PosTestResult[2] = (vertex[0]*ClipMatrix[2] + vertex[1]*ClipMatrix[6] + vertex[2]*ClipMatrix[10] + vertex[3]*ClipMatrix[14]) >> 12;
    PosTestResult[3] = (vertex[0]*ClipMatrix[3] + vertex[1]*ClipMatrix[7] + vertex[2]*ClipMatrix[11] + vertex[3]*ClipMatrix[15]) >> 12;
#endif

    AddCycles(5);
}

void GPU3D::VecTest(u32 param) noexcept
{
    // TODO: maybe it overwrites the normal registers, too

    s16 normal[3];

    normal[0] = (s16)((param & 0x000003FF) << 6) >> 6;
    normal[1] = (s16)((param & 0x000FFC00) >> 4) >> 6;
    normal[2] = (s16)((param & 0x3FF00000) >> 14) >> 6;

#if defined(LITEV_GEOM_NEON3) && defined(__ARM_NEON)
    // 32-bit modular dot with arithmetic >>9. The scalar has NO (s64) casts, so
    // each product and the 3-term sum are computed modulo 2^32 exactly as
    // CalculateLighting's shipping normaltrans NEON does: vmulq_n_s32 /
    // vmlaq_n_s32 keep the low 32 bits per lane (== scalar int products/sum mod
    // 2^32), and vshrq_n_s32 is the arithmetic >>9 on a signed 32-bit lane. Lane j
    // = normal[0]*VM[j] + normal[1]*VM[4+j] + normal[2]*VM[8+j] (j=0..2, matching
    // VecMatrix[0/4/8], [1/5/9], [2/6/10]); lane 3 is computed and discarded.
    // Assigning a lane to the s16 VecTestResult[j] narrows identically to the
    // scalar store, and the 0x1000/0xF000 sign-extend below then runs unchanged.
    {
        int32x4_t nt = vmulq_n_s32(vld1q_s32(VecMatrix + 0), (s32)normal[0]);
        nt = vmlaq_n_s32(nt, vld1q_s32(VecMatrix + 4), (s32)normal[1]);
        nt = vmlaq_n_s32(nt, vld1q_s32(VecMatrix + 8), (s32)normal[2]);
        nt = vshrq_n_s32(nt, 9);
        VecTestResult[0] = vgetq_lane_s32(nt, 0);
        VecTestResult[1] = vgetq_lane_s32(nt, 1);
        VecTestResult[2] = vgetq_lane_s32(nt, 2);
    }
#else
    VecTestResult[0] = (normal[0]*VecMatrix[0] + normal[1]*VecMatrix[4] + normal[2]*VecMatrix[8]) >> 9;
    VecTestResult[1] = (normal[0]*VecMatrix[1] + normal[1]*VecMatrix[5] + normal[2]*VecMatrix[9]) >> 9;
    VecTestResult[2] = (normal[0]*VecMatrix[2] + normal[1]*VecMatrix[6] + normal[2]*VecMatrix[10]) >> 9;
#endif

    if (VecTestResult[0] & 0x1000) VecTestResult[0] |= 0xF000;
    if (VecTestResult[1] & 0x1000) VecTestResult[1] |= 0xF000;
    if (VecTestResult[2] & 0x1000) VecTestResult[2] |= 0xF000;

    AddCycles(4);
}



void GPU3D::CmdFIFOWrite(const CmdFIFOEntry& entry) noexcept
{
    // Single source of truth: the body lives in GPU3D_GXFIFO_inl.h so DMA.cpp can inline it.
    CmdFIFOWrite_Inline(entry);
}

#if defined(LITEV_GXFIFO_UNIFIED) && defined(LITEV_GXFIFO_READ_INLINE)
void GPU3D::CmdFIFORefill() noexcept
{
    {
#else
GPU3D::CmdFIFOEntry GPU3D::CmdFIFORead() noexcept
{
#endif
#ifdef LITEV_GXFIFO_UNIFIED
#ifndef LITEV_GXFIFO_READ_INLINE
    // callers only read a non-empty PIPE
    CmdFIFOEntry ret = CmdQ[CmdQHead];
    CmdQHead = (CmdQHead + 1) & 511;
    PipeN--;

    if (PipeN <= 2)
    {
#endif
        u32 n = FifoN < 2 ? FifoN : 2;   // the first FIFO entries become PIPE entries in place
        PipeN += n;
        FifoN -= n;
#else
    CmdFIFOEntry ret = CmdPIPE.Read();

    if (CmdPIPE.Level() <= 2)
    {
        if (!CmdFIFO.IsEmpty())
            CmdPIPE.Write(CmdFIFO.Read());
        if (!CmdFIFO.IsEmpty())
            CmdPIPE.Write(CmdFIFO.Read());
#endif

        // empty stall queue if needed
        // CmdFIFO should not be full at this point.
        if (!CmdStallQueue.IsEmpty())
        {
            while (!CmdStallQueue.IsEmpty())
            {
                if (FifoFull()) break;
                CmdFIFOEntry entry = CmdStallQueue.Read();
                CmdFIFOWrite(entry);
            }

            if (CmdStallQueue.IsEmpty())
                NDS.GXFIFOUnstall();
        }

        CheckFIFODMA();
        CheckFIFOIRQ();
    }
#if !(defined(LITEV_GXFIFO_UNIFIED) && defined(LITEV_GXFIFO_READ_INLINE))
    return ret;
#endif
}

#ifdef LITEV_GX_BULK
#if !defined(LITEV_GXFIFO_THREADED) || !defined(LITEV_GXFIFO_UNIFIED) || !defined(LITEV_DMA_GXFIFO_FAST)
#error "LITEV_GX_BULK needs LITEV_GXFIFO_THREADED, LITEV_GXFIFO_UNIFIED and LITEV_DMA_GXFIFO_FAST"
#endif
void GPU3D::ExecuteCommand() noexcept { ExecuteCommandT<false>(); }

template<bool Bulk>
void GPU3D::ExecuteCommandT() noexcept
#else
void GPU3D::ExecuteCommand() noexcept
#endif
{
#ifdef LITEV_GXFIFO_THREADED
    // DraStic #3 (backlog D.7 §3): batched threaded-code interpreter. Run()'s
    // drain loop is hoisted in here — after each command the loop-tail below
    // jumps back to this label, so a whole batch drains with ONE call, no
    // per-command bl/ret or prologue/epilogue. Reuses the computed-goto
    // tables below. Bit-exact (CmdFIFORead still fires per command
    // in identical order, keeping DMA/IRQ/audio timing).
gxfifo_threaded_top:
#endif
    // M6.11: count GXFIFO commands (cheap add only; GPU3DNs times the whole
    // Run()/drain batch so per-command clock_gettime does not distort it).
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.GXCommands);

#ifdef LITEV_GX_BULK
    CmdFIFOEntry entry;
    if constexpr (Bulk)
    {
        // a SWAP_BUFFERS waits for its turn in the FIFO (it resets CycleCount)
        if (BulkPtr == BulkEnd || BulkPtr->Command == 0x50) return;
        entry = *BulkPtr++;
        // CmdFIFOWrite counts these per entry; their handlers count them down
        if (entry.Command == 0x11 || entry.Command == 0x12) NumPushPopCommands++;
        else if (entry.Command >= 0x70 && entry.Command <= 0x72) NumTestCommands++;
    }
    else entry = CmdFIFORead();
#else
    CmdFIFOEntry entry = CmdFIFORead();
#endif

    //printf("FIFO: processing %02X %08X. Levels: FIFO=%d, PIPE=%d\n", entry.Command, entry.Param, CmdFIFO->Level(), CmdPIPE->Level());

    // each FIFO entry takes 1 cycle to be processed
    // commands (presumably) run when all the needed parameters have been read
    // which is where we add the remaining cycles if any

#ifdef LITEV_ACCESS_STATS
    { extern u64 LitevAccess[6][0x10000]; LitevAccess[5][0xC000 | entry.Command]++; }   // GX FIFO entries by command
#endif
    u32 paramsRequiredCount = CmdNumParams[entry.Command];
    if (paramsRequiredCount <= 1)
    {
        // fast path for command which only have a single parameter

        /*printf("[GXS:%08X] 0x%02X,  0x%08X", GXStat, entry.Command, entry.Param);*/

#ifdef LITEV_GXFIFO_THREADED
        // liteDS-v2 gxfifo: DraStic-style threaded-code dispatch (teardown
        // docs/drastic-teardown/04-gpu3d-geometry.md §3). A flat 256-entry
        // label table indexed by the command byte replaces the switch's
        // implicit [0x10..0x72] range-check + default fall-through: one table
        // load + one indirect branch per command, no bounds test. The handler
        // bodies below are statement-for-statement identical to the switch in
        // the #else arm, so geometry output AND cycle accounting are bit-exact
        // — only the dispatch mechanism changes.
        {
        static const void* const gxfFast[256] =
        {
            [0 ... 255] = &&gxf_default,
            [0x10] = &&gxf_10, [0x11] = &&gxf_11, [0x12] = &&gxf_12,
            [0x13] = &&gxf_13, [0x14] = &&gxf_14, [0x15] = &&gxf_15,
            [0x20] = &&gxf_20, [0x21] = &&gxf_21, [0x22] = &&gxf_22,
            [0x24] = &&gxf_24, [0x25] = &&gxf_25, [0x26] = &&gxf_26,
            [0x27] = &&gxf_27, [0x28] = &&gxf_28, [0x29] = &&gxf_29,
            [0x2A] = &&gxf_2A, [0x2B] = &&gxf_2B, [0x30] = &&gxf_30,
            [0x31] = &&gxf_31, [0x32] = &&gxf_32, [0x33] = &&gxf_33,
            [0x40] = &&gxf_40, [0x41] = &&gxf_41, [0x50] = &&gxf_50,
            [0x60] = &&gxf_60, [0x72] = &&gxf_72,
        };
        goto *gxfFast[entry.Command];

        gxf_10: // matrix mode
            VertexPipelineCmdDelayed4();
            MatrixMode = entry.Param & 0x3;
            goto gxf_end;

        gxf_11: // push matrix
            VertexPipelineCmdDelayed4();
            NumPushPopCommands--;
            if (MatrixMode == 0)
            {
                if (ProjMatrixStackPointer > 0) GXStat |= (1<<15);
                memcpy(ProjMatrixStack, ProjMatrix, 16*4);
                ProjMatrixStackPointer++;
                ProjMatrixStackPointer &= 0x1;
            }
            else if (MatrixMode == 3)
            {
                if (TexMatrixStackPointer > 0) GXStat |= (1<<15);
                memcpy(TexMatrixStack, TexMatrix, 16*4);
                TexMatrixStackPointer++;
                TexMatrixStackPointer &= 0x1;
            }
            else
            {
                if (PosMatrixStackPointer > 30) GXStat |= (1<<15);
                memcpy(PosMatrixStack[PosMatrixStackPointer & 0x1F], PosMatrix, 16*4);
                memcpy(VecMatrixStack[PosMatrixStackPointer & 0x1F], VecMatrix, 16*4);
                PosMatrixStackPointer++;
                PosMatrixStackPointer &= 0x3F;
            }
            AddCycles(16);
            goto gxf_end;

        gxf_12: // pop matrix
            VertexPipelineCmdDelayed4();
            NumPushPopCommands--;
            if (MatrixMode == 0)
            {
                if (ProjMatrixStackPointer == 0) GXStat |= (1<<15);
                ProjMatrixStackPointer--;
                ProjMatrixStackPointer &= 0x1;
                memcpy(ProjMatrix, ProjMatrixStack, 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            else if (MatrixMode == 3)
            {
                if (TexMatrixStackPointer == 0) GXStat |= (1<<15);
                TexMatrixStackPointer--;
                TexMatrixStackPointer &= 0x1;
                memcpy(TexMatrix, TexMatrixStack, 16*4);
                AddCycles(17);
            }
            else
            {
                s32 offset = (s32)(entry.Param << 26) >> 26;
                PosMatrixStackPointer -= offset;
                PosMatrixStackPointer &= 0x3F;
                if (PosMatrixStackPointer > 30) GXStat |= (1<<15);
                memcpy(PosMatrix, PosMatrixStack[PosMatrixStackPointer & 0x1F], 16*4);
                memcpy(VecMatrix, VecMatrixStack[PosMatrixStackPointer & 0x1F], 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            goto gxf_end;

        gxf_13: // store matrix
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                memcpy(ProjMatrixStack, ProjMatrix, 16*4);
            }
            else if (MatrixMode == 3)
            {
                memcpy(TexMatrixStack, TexMatrix, 16*4);
            }
            else
            {
                u32 addr = entry.Param & 0x1F;
                if (addr > 30) GXStat |= (1<<15);
                memcpy(PosMatrixStack[addr], PosMatrix, 16*4);
                memcpy(VecMatrixStack[addr], VecMatrix, 16*4);
            }
            AddCycles(16);
            goto gxf_end;

        gxf_14: // restore matrix
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                memcpy(ProjMatrix, ProjMatrixStack, 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            else if (MatrixMode == 3)
            {
                memcpy(TexMatrix, TexMatrixStack, 16*4);
                AddCycles(17);
            }
            else
            {
                u32 addr = entry.Param & 0x1F;
                if (addr > 30) GXStat |= (1<<15);
                memcpy(PosMatrix, PosMatrixStack[addr], 16*4);
                memcpy(VecMatrix, VecMatrixStack[addr], 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            goto gxf_end;

        gxf_15: // identity
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                MatrixLoadIdentity(ProjMatrix);
                ClipMatrixDirty = true;
                AddCycles(18);
            }
            else if (MatrixMode == 3)
                MatrixLoadIdentity(TexMatrix);
            else
            {
                MatrixLoadIdentity(PosMatrix);
                if (MatrixMode == 2)
                    MatrixLoadIdentity(VecMatrix);
                ClipMatrixDirty = true;
                AddCycles(18);
            }
            goto gxf_end;

        gxf_20: // vertex color
            VertexPipelineCmdDelayed6();
            {
                u32 c = entry.Param;
                u32 r = c & 0x1F;
                u32 g = (c >> 5) & 0x1F;
                u32 b = (c >> 10) & 0x1F;
                VertexColor[0] = r;
                VertexColor[1] = g;
                VertexColor[2] = b;
            }
            goto gxf_end;

        gxf_21: // normal
            VertexPipelineCmdDelayed4();
            Normal[0] = (s16)((entry.Param & 0x000003FF) << 6) >> 6;
            Normal[1] = (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
            Normal[2] = (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
            CalculateLighting();
            goto gxf_end;

        gxf_22: // texcoord
            VertexPipelineCmdDelayed4();
            RawTexCoords[0] = entry.Param & 0xFFFF;
            RawTexCoords[1] = entry.Param >> 16;
            if ((TexParam >> 30) == 1)
            {
                TexCoords[0] = (RawTexCoords[0]*TexMatrix[0] + RawTexCoords[1]*TexMatrix[4] + TexMatrix[8] + TexMatrix[12]) >> 12;
                TexCoords[1] = (RawTexCoords[0]*TexMatrix[1] + RawTexCoords[1]*TexMatrix[5] + TexMatrix[9] + TexMatrix[13]) >> 12;
            }
            else
            {
                TexCoords[0] = RawTexCoords[0];
                TexCoords[1] = RawTexCoords[1];
            }
            goto gxf_end;

        gxf_24: // 10-bit vertex
            VertexPipelineSubmitCmd();
            CurVertex[0] = (entry.Param & 0x000003FF) << 6;
            CurVertex[1] = (entry.Param & 0x000FFC00) >> 4;
            CurVertex[2] = (entry.Param & 0x3FF00000) >> 14;
            SubmitVertex();
            goto gxf_end;

        gxf_25: // vertex XY
            VertexPipelineSubmitCmd();
            CurVertex[0] = entry.Param & 0xFFFF;
            CurVertex[1] = entry.Param >> 16;
            SubmitVertex();
            goto gxf_end;

        gxf_26: // vertex XZ
            VertexPipelineSubmitCmd();
            CurVertex[0] = entry.Param & 0xFFFF;
            CurVertex[2] = entry.Param >> 16;
            SubmitVertex();
            goto gxf_end;

        gxf_27: // vertex YZ
            VertexPipelineSubmitCmd();
            CurVertex[1] = entry.Param & 0xFFFF;
            CurVertex[2] = entry.Param >> 16;
            SubmitVertex();
            goto gxf_end;

        gxf_28: // 10-bit delta vertex
            VertexPipelineSubmitCmd();
            CurVertex[0] += (s16)((entry.Param & 0x000003FF) << 6) >> 6;
            CurVertex[1] += (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
            CurVertex[2] += (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
            SubmitVertex();
            goto gxf_end;

        gxf_29: // polygon attributes
            VertexPipelineCmdDelayed8();
            PolygonAttr = entry.Param;
            goto gxf_end;

        gxf_2A: // texture param
            VertexPipelineCmdDelayed8();
            TexParam = entry.Param;
            goto gxf_end;

        gxf_2B: // texture palette
            VertexPipelineCmdDelayed8();
            TexPalette = entry.Param & 0x1FFF;
            goto gxf_end;

        gxf_30: // diffuse/ambient material
            VertexPipelineCmdDelayed6();
            MatDiffuse[0] = entry.Param & 0x1F;
            MatDiffuse[1] = (entry.Param >> 5) & 0x1F;
            MatDiffuse[2] = (entry.Param >> 10) & 0x1F;
            MatAmbient[0] = (entry.Param >> 16) & 0x1F;
            MatAmbient[1] = (entry.Param >> 21) & 0x1F;
            MatAmbient[2] = (entry.Param >> 26) & 0x1F;
            if (entry.Param & 0x8000)
            {
                VertexColor[0] = MatDiffuse[0];
                VertexColor[1] = MatDiffuse[1];
                VertexColor[2] = MatDiffuse[2];
            }
            AddCycles(3);
            goto gxf_end;

        gxf_31: // specular/emission material
            VertexPipelineCmdDelayed6();
            MatSpecular[0] = entry.Param & 0x1F;
            MatSpecular[1] = (entry.Param >> 5) & 0x1F;
            MatSpecular[2] = (entry.Param >> 10) & 0x1F;
            MatEmission[0] = (entry.Param >> 16) & 0x1F;
            MatEmission[1] = (entry.Param >> 21) & 0x1F;
            MatEmission[2] = (entry.Param >> 26) & 0x1F;
            UseShininessTable = (entry.Param & 0x8000) != 0;
            AddCycles(3);
            goto gxf_end;

        gxf_32: // light direction
            StallPolygonPipeline(8 + 1,  2); // 0x32 can run 6 cycles after a vertex
            {
                u32 l = entry.Param >> 30;
                s16 dir[3];
                dir[0] = (s16)((entry.Param & 0x000003FF) << 6) >> 6;
                dir[1] = (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
                dir[2] = (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
                LightDirection[l][0] = (-((dir[0]*VecMatrix[0] + dir[1]*VecMatrix[4] + dir[2]*VecMatrix[8] ) >> 12) << 21) >> 21;
                LightDirection[l][1] = (-((dir[0]*VecMatrix[1] + dir[1]*VecMatrix[5] + dir[2]*VecMatrix[9] ) >> 12) << 21) >> 21;
                LightDirection[l][2] = (-((dir[0]*VecMatrix[2] + dir[1]*VecMatrix[6] + dir[2]*VecMatrix[10]) >> 12) << 21) >> 21;
                s32 den =              -(((dir[0]*VecMatrix[2] + dir[1]*VecMatrix[6] + dir[2]*VecMatrix[10]) << 9) >> 21) + (1<<9);
                if (den == 0) SpecRecip[l] = 0;
                else SpecRecip[l] = (1<<18) / den;
            }
            AddCycles(5);
            goto gxf_end;

        gxf_33: // light color
            VertexPipelineCmdDelayed8();
            {
                u32 l = entry.Param >> 30;
                LightColor[l][0] = entry.Param & 0x1F;
                LightColor[l][1] = (entry.Param >> 5) & 0x1F;
                LightColor[l][2] = (entry.Param >> 10) & 0x1F;
            }
            AddCycles(1);
            goto gxf_end;

        gxf_40: // begin polygons
            StallPolygonPipeline(1, 0);
            PolygonMode = entry.Param & 0x3;
            VertexNum = 0;
            VertexNumInPoly = 0;
            NumConsecutivePolygons = 0;
            LastStripPolygon = NULL;
            CurPolygonAttr = PolygonAttr;
            goto gxf_end;

        gxf_41: // end polygons
            VertexPipelineCmdDelayed8();
            goto gxf_end;

        gxf_50: // flush
            VertexPipelineCmdDelayed4();
            FlushRequest = 1;
            SwapCount++;
            FlushAttributes = entry.Param & 0x3;
            CycleCount = 325;
            VertexPipeline = 0;
            NormalPipeline = 0;
            PolygonPipeline = 0;
            VertexSlotCounter = 0;
            VertexSlotsFree = 1;
            goto gxf_end;

        gxf_60: // viewport x1,y1,x2,y2
            VertexPipelineCmdDelayed8();
            // note: viewport Y coordinates are upside-down
            Viewport[0] = entry.Param & 0xFF;                             // x0
            Viewport[1] = (191 - ((entry.Param >> 8) & 0xFF)) & 0xFF;     // y0
            Viewport[2] = (entry.Param >> 16) & 0xFF;                     // x1
            Viewport[3] = (191 - (entry.Param >> 24)) & 0xFF;             // y1
            Viewport[4] = (Viewport[2] - Viewport[0] + 1) & 0x1FF;          // width
            Viewport[5] = (Viewport[1] - Viewport[3] + 1) & 0xFF;           // height
            goto gxf_end;

        gxf_72: // vec test
            VertexPipelineCmdDelayed6();
            NumTestCommands--;
            VecTest(entry.Param);
            goto gxf_end;

        gxf_default:
            VertexPipelineCmdDelayed4();
            //printf("!! UNKNOWN GX COMMAND %02X %08X\n", entry.Command, entry.Param);
            goto gxf_end;

        gxf_end: ;
        }
#else
        switch (entry.Command)
        {
        case 0x10: // matrix mode
            VertexPipelineCmdDelayed4();
            MatrixMode = entry.Param & 0x3;
            break;

        case 0x11: // push matrix
            VertexPipelineCmdDelayed4();
            NumPushPopCommands--;
            if (MatrixMode == 0)
            {
                if (ProjMatrixStackPointer > 0) GXStat |= (1<<15);

                memcpy(ProjMatrixStack, ProjMatrix, 16*4);
                ProjMatrixStackPointer++;
                ProjMatrixStackPointer &= 0x1;
            }
            else if (MatrixMode == 3)
            {
                if (TexMatrixStackPointer > 0) GXStat |= (1<<15);

                memcpy(TexMatrixStack, TexMatrix, 16*4);
                TexMatrixStackPointer++;
                TexMatrixStackPointer &= 0x1;
            }
            else
            {
                if (PosMatrixStackPointer > 30) GXStat |= (1<<15);

                memcpy(PosMatrixStack[PosMatrixStackPointer & 0x1F], PosMatrix, 16*4);
                memcpy(VecMatrixStack[PosMatrixStackPointer & 0x1F], VecMatrix, 16*4);
                PosMatrixStackPointer++;
                PosMatrixStackPointer &= 0x3F;
            }
            AddCycles(16);
            break;

        case 0x12: // pop matrix
            VertexPipelineCmdDelayed4();
            NumPushPopCommands--;
            if (MatrixMode == 0)
            {
                if (ProjMatrixStackPointer == 0) GXStat |= (1<<15);

                ProjMatrixStackPointer--;
                ProjMatrixStackPointer &= 0x1;
                memcpy(ProjMatrix, ProjMatrixStack, 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            else if (MatrixMode == 3)
            {
                if (TexMatrixStackPointer == 0) GXStat |= (1<<15);

                TexMatrixStackPointer--;
                TexMatrixStackPointer &= 0x1;
                memcpy(TexMatrix, TexMatrixStack, 16*4);
                AddCycles(17);
            }
            else
            {
                s32 offset = (s32)(entry.Param << 26) >> 26;
                PosMatrixStackPointer -= offset;
                PosMatrixStackPointer &= 0x3F;

                if (PosMatrixStackPointer > 30) GXStat |= (1<<15);

                memcpy(PosMatrix, PosMatrixStack[PosMatrixStackPointer & 0x1F], 16*4);
                memcpy(VecMatrix, VecMatrixStack[PosMatrixStackPointer & 0x1F], 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            break;

        case 0x13: // store matrix
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                memcpy(ProjMatrixStack, ProjMatrix, 16*4);
            }
            else if (MatrixMode == 3)
            {
                memcpy(TexMatrixStack, TexMatrix, 16*4);
            }
            else
            {
                u32 addr = entry.Param & 0x1F;
                if (addr > 30) GXStat |= (1<<15);

                memcpy(PosMatrixStack[addr], PosMatrix, 16*4);
                memcpy(VecMatrixStack[addr], VecMatrix, 16*4);
            }
            AddCycles(16);
            break;

        case 0x14: // restore matrix
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                memcpy(ProjMatrix, ProjMatrixStack, 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            else if (MatrixMode == 3)
            {
                memcpy(TexMatrix, TexMatrixStack, 16*4);
                AddCycles(17);
            }
            else
            {
                u32 addr = entry.Param & 0x1F;
                if (addr > 30) GXStat |= (1<<15);

                memcpy(PosMatrix, PosMatrixStack[addr], 16*4);
                memcpy(VecMatrix, VecMatrixStack[addr], 16*4);
                ClipMatrixDirty = true;
                AddCycles(35);
            }
            break;

        case 0x15: // identity
            VertexPipelineCmdDelayed4();
            if (MatrixMode == 0)
            {
                MatrixLoadIdentity(ProjMatrix);
                ClipMatrixDirty = true;
                AddCycles(18);
            }
            else if (MatrixMode == 3)
                MatrixLoadIdentity(TexMatrix);
            else
            {
                MatrixLoadIdentity(PosMatrix);
                if (MatrixMode == 2)
                    MatrixLoadIdentity(VecMatrix);
                ClipMatrixDirty = true;
                AddCycles(18);
            }
            break;

        case 0x20: // vertex color
            VertexPipelineCmdDelayed6();
            {
                u32 c = entry.Param;
                u32 r = c & 0x1F;
                u32 g = (c >> 5) & 0x1F;
                u32 b = (c >> 10) & 0x1F;
                VertexColor[0] = r;
                VertexColor[1] = g;
                VertexColor[2] = b;
            }
            break;

        case 0x21: // normal
            VertexPipelineCmdDelayed4();
            Normal[0] = (s16)((entry.Param & 0x000003FF) << 6) >> 6;
            Normal[1] = (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
            Normal[2] = (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
            CalculateLighting();
            break;

        case 0x22: // texcoord
            VertexPipelineCmdDelayed4();
            RawTexCoords[0] = entry.Param & 0xFFFF;
            RawTexCoords[1] = entry.Param >> 16;
            if ((TexParam >> 30) == 1)
            {
                TexCoords[0] = (RawTexCoords[0]*TexMatrix[0] + RawTexCoords[1]*TexMatrix[4] + TexMatrix[8] + TexMatrix[12]) >> 12;
                TexCoords[1] = (RawTexCoords[0]*TexMatrix[1] + RawTexCoords[1]*TexMatrix[5] + TexMatrix[9] + TexMatrix[13]) >> 12;
            }
            else
            {
                TexCoords[0] = RawTexCoords[0];
                TexCoords[1] = RawTexCoords[1];
            }
            break;

        case 0x24: // 10-bit vertex
            VertexPipelineSubmitCmd();
            CurVertex[0] = (entry.Param & 0x000003FF) << 6;
            CurVertex[1] = (entry.Param & 0x000FFC00) >> 4;
            CurVertex[2] = (entry.Param & 0x3FF00000) >> 14;
            SubmitVertex();
            break;

        case 0x25: // vertex XY
            VertexPipelineSubmitCmd();
            CurVertex[0] = entry.Param & 0xFFFF;
            CurVertex[1] = entry.Param >> 16;
            SubmitVertex();
            break;

        case 0x26: // vertex XZ
            VertexPipelineSubmitCmd();
            CurVertex[0] = entry.Param & 0xFFFF;
            CurVertex[2] = entry.Param >> 16;
            SubmitVertex();
            break;

        case 0x27: // vertex YZ
            VertexPipelineSubmitCmd();
            CurVertex[1] = entry.Param & 0xFFFF;
            CurVertex[2] = entry.Param >> 16;
            SubmitVertex();
            break;

        case 0x28: // 10-bit delta vertex
            VertexPipelineSubmitCmd();
            CurVertex[0] += (s16)((entry.Param & 0x000003FF) << 6) >> 6;
            CurVertex[1] += (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
            CurVertex[2] += (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
            SubmitVertex();
            break;

        case 0x29: // polygon attributes
            VertexPipelineCmdDelayed8();
            PolygonAttr = entry.Param;
            break;

        case 0x2A: // texture param
            VertexPipelineCmdDelayed8();
            TexParam = entry.Param;
            break;

        case 0x2B: // texture palette
            VertexPipelineCmdDelayed8();
            TexPalette = entry.Param & 0x1FFF;
            break;

        case 0x30: // diffuse/ambient material
            VertexPipelineCmdDelayed6();
            MatDiffuse[0] = entry.Param & 0x1F;
            MatDiffuse[1] = (entry.Param >> 5) & 0x1F;
            MatDiffuse[2] = (entry.Param >> 10) & 0x1F;
            MatAmbient[0] = (entry.Param >> 16) & 0x1F;
            MatAmbient[1] = (entry.Param >> 21) & 0x1F;
            MatAmbient[2] = (entry.Param >> 26) & 0x1F;
            if (entry.Param & 0x8000)
            {
                VertexColor[0] = MatDiffuse[0];
                VertexColor[1] = MatDiffuse[1];
                VertexColor[2] = MatDiffuse[2];
            }
            AddCycles(3);
            break;

        case 0x31: // specular/emission material
            VertexPipelineCmdDelayed6();
            MatSpecular[0] = entry.Param & 0x1F;
            MatSpecular[1] = (entry.Param >> 5) & 0x1F;
            MatSpecular[2] = (entry.Param >> 10) & 0x1F;
            MatEmission[0] = (entry.Param >> 16) & 0x1F;
            MatEmission[1] = (entry.Param >> 21) & 0x1F;
            MatEmission[2] = (entry.Param >> 26) & 0x1F;
            UseShininessTable = (entry.Param & 0x8000) != 0;
            AddCycles(3);
            break;

        case 0x32: // light direction
            StallPolygonPipeline(8 + 1,  2); // 0x32 can run 6 cycles after a vertex
            {
                u32 l = entry.Param >> 30;
                s16 dir[3];
                dir[0] = (s16)((entry.Param & 0x000003FF) << 6) >> 6;
                dir[1] = (s16)((entry.Param & 0x000FFC00) >> 4) >> 6;
                dir[2] = (s16)((entry.Param & 0x3FF00000) >> 14) >> 6;
                // the order of operations here is very specific: discard bottom 12 bits -> negate -> then sign extend to convert to 11 bit signed int
                // except for when used to calculate the specular reciprocal; then it's: sign extend -> discard lsb -> negate.
                LightDirection[l][0] = (-((dir[0]*VecMatrix[0] + dir[1]*VecMatrix[4] + dir[2]*VecMatrix[8] ) >> 12) << 21) >> 21;
                LightDirection[l][1] = (-((dir[0]*VecMatrix[1] + dir[1]*VecMatrix[5] + dir[2]*VecMatrix[9] ) >> 12) << 21) >> 21;
                LightDirection[l][2] = (-((dir[0]*VecMatrix[2] + dir[1]*VecMatrix[6] + dir[2]*VecMatrix[10]) >> 12) << 21) >> 21;
                s32 den =              -(((dir[0]*VecMatrix[2] + dir[1]*VecMatrix[6] + dir[2]*VecMatrix[10]) << 9) >> 21) + (1<<9);

                if (den == 0) SpecRecip[l] = 0;
                else SpecRecip[l] = (1<<18) / den;
            }
            AddCycles(5);
            break;

        case 0x33: // light color
            VertexPipelineCmdDelayed8();
            {
                u32 l = entry.Param >> 30;
                LightColor[l][0] = entry.Param & 0x1F;
                LightColor[l][1] = (entry.Param >> 5) & 0x1F;
                LightColor[l][2] = (entry.Param >> 10) & 0x1F;
            }
            AddCycles(1);
            break;

        case 0x40: // begin polygons
            StallPolygonPipeline(1, 0);
            // TODO: check if there was a polygon being defined but incomplete
            // such cases seem to freeze the GPU
            PolygonMode = entry.Param & 0x3;
            VertexNum = 0;
            VertexNumInPoly = 0;
            NumConsecutivePolygons = 0;
            LastStripPolygon = NULL;
            CurPolygonAttr = PolygonAttr;
            break;

        case 0x41: // end polygons
            VertexPipelineCmdDelayed8();
            // TODO: research this?
            // it doesn't seem to have any effect whatsoever, but
            // its timing characteristics are different from those of other
            // no-op commands
            break;

        case 0x50: // flush
            VertexPipelineCmdDelayed4();
            FlushRequest = 1;
            SwapCount++;
            FlushAttributes = entry.Param & 0x3;
            CycleCount = 325;
            // probably safe to just reset all pipelines
            // but needs checked
            VertexPipeline = 0;
            NormalPipeline = 0;
            PolygonPipeline = 0;
            VertexSlotCounter = 0;
            VertexSlotsFree = 1;
            break;

        case 0x60: // viewport x1,y1,x2,y2
            VertexPipelineCmdDelayed8();
            // note: viewport Y coordinates are upside-down
            Viewport[0] = entry.Param & 0xFF;                             // x0
            Viewport[1] = (191 - ((entry.Param >> 8) & 0xFF)) & 0xFF;     // y0
            Viewport[2] = (entry.Param >> 16) & 0xFF;                     // x1
            Viewport[3] = (191 - (entry.Param >> 24)) & 0xFF;             // y1
            Viewport[4] = (Viewport[2] - Viewport[0] + 1) & 0x1FF;          // width
            Viewport[5] = (Viewport[1] - Viewport[3] + 1) & 0xFF;           // height
            break;

        case 0x72: // vec test
            VertexPipelineCmdDelayed6();
            NumTestCommands--;
            VecTest(entry.Param);
            break;

        default:
            VertexPipelineCmdDelayed4();
            //printf("!! UNKNOWN GX COMMAND %02X %08X\n", entry.Command, entry.Param);
            break;
        }
#endif // LITEV_GXFIFO_THREADED
    }
    else
    {
        ExecParams[ExecParamCount] = entry.Param;
        ExecParamCount++;

        if (ExecParamCount == 1)
        {
            // delay the first command entry as needed
            switch (entry.Command)
            {
            // commands that stall the polygon pipeline
            case 0x23: VertexPipelineSubmitCmd(); break;
            case 0x34:
            case 0x71:
                VertexPipelineCmdDelayed8();
                break;
            case 0x70: StallPolygonPipeline(10 + 1, 0); break;
            default: VertexPipelineCmdDelayed4(); break;
            }
        }
        else
        {
            AddCycles(1);

            if (ExecParamCount >= paramsRequiredCount)
            {
                /*printf("[GXS:%08X] 0x%02X,  ", GXStat, entry.Command);
                for (int k = 0; k < ExecParamCount; k++) printf("0x%08X, ", ExecParams[k]);
                printf("\n");*/

                ExecParamCount = 0;

#ifdef LITEV_GXFIFO_THREADED
                // liteDS-v2 gxfifo: threaded-code dispatch for the multi-param
                // completion path (hot for VTX_16 / MTX_MULT). Statement-
                // identical handlers to the #else switch; bit-exact.
                {
                static const void* const gxfFull[256] =
                {
                    [0 ... 255] = &&gxc_default,
                    [0x16] = &&gxc_16, [0x17] = &&gxc_17, [0x18] = &&gxc_18,
                    [0x19] = &&gxc_19, [0x1A] = &&gxc_1A, [0x1B] = &&gxc_1B,
                    [0x1C] = &&gxc_1C, [0x23] = &&gxc_23, [0x34] = &&gxc_34,
                    [0x71] = &&gxc_71, [0x70] = &&gxc_70,
                };
                goto *gxfFull[entry.Command];

                gxc_16: // load 4x4
                    if (MatrixMode == 0)
                    {
                        MatrixLoad4x4(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixLoad4x4(TexMatrix, (s32*)ExecParams);
                        AddCycles(10);
                    }
                    else
                    {
                        MatrixLoad4x4(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                            MatrixLoad4x4(VecMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    goto gxc_end;

                gxc_17: // load 4x3
                    if (MatrixMode == 0)
                    {
                        MatrixLoad4x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixLoad4x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(7);
                    }
                    else
                    {
                        MatrixLoad4x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                            MatrixLoad4x3(VecMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    goto gxc_end;

                gxc_18: // mult 4x4
                    if (MatrixMode == 0)
                    {
                        MatrixMult4x4(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 16);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult4x4(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 16);
                    }
                    else
                    {
                        MatrixMult4x4(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult4x4(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 16);
                        }
                        else AddCycles(35 - 16);
                        ClipMatrixDirty = true;
                    }
                    goto gxc_end;

                gxc_19: // mult 4x3
                    if (MatrixMode == 0)
                    {
                        MatrixMult4x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 12);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult4x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 12);
                    }
                    else
                    {
                        MatrixMult4x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult4x3(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 12);
                        }
                        else AddCycles(35 - 12);
                        ClipMatrixDirty = true;
                    }
                    goto gxc_end;

                gxc_1A: // mult 3x3
                    if (MatrixMode == 0)
                    {
                        MatrixMult3x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 9);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult3x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 9);
                    }
                    else
                    {
                        MatrixMult3x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult3x3(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 9);
                        }
                        else AddCycles(35 - 9);
                        ClipMatrixDirty = true;
                    }
                    goto gxc_end;

                gxc_1B: // scale
                    if (MatrixMode == 0)
                    {
                        MatrixScale(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixScale(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 3);
                    }
                    else
                    {
                        MatrixScale(PosMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    goto gxc_end;

                gxc_1C: // translate
                    if (MatrixMode == 0)
                    {
                        MatrixTranslate(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixTranslate(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 3);
                    }
                    else
                    {
                        MatrixTranslate(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixTranslate(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 3);
                        }
                        else AddCycles(35 - 3);
                        ClipMatrixDirty = true;
                    }
                    goto gxc_end;

                gxc_23: // full vertex
                    CurVertex[0] = ExecParams[0] & 0xFFFF;
                    CurVertex[1] = ExecParams[0] >> 16;
                    CurVertex[2] = ExecParams[1] & 0xFFFF;
                    SubmitVertex();
                    goto gxc_end;

                gxc_34: // shininess table
                    {
                        for (int i = 0; i < 128; i += 4)
                        {
                            u32 val = ExecParams[i >> 2];
                            ShininessTable[i + 0] = val & 0xFF;
                            ShininessTable[i + 1] = (val >> 8) & 0xFF;
                            ShininessTable[i + 2] = (val >> 16) & 0xFF;
                            ShininessTable[i + 3] = val >> 24;
                        }
                    }
                    goto gxc_end;

                gxc_71: // pos test
                    NumTestCommands -= 2;
                    CurVertex[0] = ExecParams[0] & 0xFFFF;
                    CurVertex[1] = ExecParams[0] >> 16;
                    CurVertex[2] = ExecParams[1] & 0xFFFF;
                    PosTest();
                    goto gxc_end;

                gxc_70: // box test
                    NumTestCommands -= 3;
                    BoxTest(ExecParams);
                    goto gxc_end;

                gxc_default:
                    __builtin_unreachable();

                gxc_end: ;
                }
#else
                switch (entry.Command)
                {
                case 0x16: // load 4x4
                    if (MatrixMode == 0)
                    {
                        MatrixLoad4x4(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixLoad4x4(TexMatrix, (s32*)ExecParams);
                        AddCycles(10);
                    }
                    else
                    {
                        MatrixLoad4x4(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                            MatrixLoad4x4(VecMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    break;

                case 0x17: // load 4x3
                    if (MatrixMode == 0)
                    {
                        MatrixLoad4x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixLoad4x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(7);
                    }
                    else
                    {
                        MatrixLoad4x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                            MatrixLoad4x3(VecMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(18);
                    }
                    break;

                case 0x18: // mult 4x4
                    if (MatrixMode == 0)
                    {
                        MatrixMult4x4(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 16);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult4x4(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 16);
                    }
                    else
                    {
                        MatrixMult4x4(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult4x4(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 16);
                        }
                        else AddCycles(35 - 16);
                        ClipMatrixDirty = true;
                    }
                    break;

                case 0x19: // mult 4x3
                    if (MatrixMode == 0)
                    {
                        MatrixMult4x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 12);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult4x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 12);
                    }
                    else
                    {
                        MatrixMult4x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult4x3(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 12);
                        }
                        else AddCycles(35 - 12);
                        ClipMatrixDirty = true;
                    }
                    break;

                case 0x1A: // mult 3x3
                    if (MatrixMode == 0)
                    {
                        MatrixMult3x3(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 9);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixMult3x3(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 9);
                    }
                    else
                    {
                        MatrixMult3x3(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixMult3x3(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 9);
                        }
                        else AddCycles(35 - 9);
                        ClipMatrixDirty = true;
                    }
                    break;

                case 0x1B: // scale
                    if (MatrixMode == 0)
                    {
                        MatrixScale(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixScale(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 3);
                    }
                    else
                    {
                        MatrixScale(PosMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    break;

                case 0x1C: // translate
                    if (MatrixMode == 0)
                    {
                        MatrixTranslate(ProjMatrix, (s32*)ExecParams);
                        ClipMatrixDirty = true;
                        AddCycles(35 - 3);
                    }
                    else if (MatrixMode == 3)
                    {
                        MatrixTranslate(TexMatrix, (s32*)ExecParams);
                        AddCycles(33 - 3);
                    }
                    else
                    {
                        MatrixTranslate(PosMatrix, (s32*)ExecParams);
                        if (MatrixMode == 2)
                        {
                            MatrixTranslate(VecMatrix, (s32*)ExecParams);
                            AddCycles(35 + 30 - 3);
                        }
                        else AddCycles(35 - 3);
                        ClipMatrixDirty = true;
                    }
                    break;

                case 0x23: // full vertex
                    CurVertex[0] = ExecParams[0] & 0xFFFF;
                    CurVertex[1] = ExecParams[0] >> 16;
                    CurVertex[2] = ExecParams[1] & 0xFFFF;
                    SubmitVertex();
                    break;

                case 0x34: // shininess table
                    {
                        for (int i = 0; i < 128; i += 4)
                        {
                            u32 val = ExecParams[i >> 2];
                            ShininessTable[i + 0] = val & 0xFF;
                            ShininessTable[i + 1] = (val >> 8) & 0xFF;
                            ShininessTable[i + 2] = (val >> 16) & 0xFF;
                            ShininessTable[i + 3] = val >> 24;
                        }
                    }
                    break;

                case 0x71: // pos test
                    NumTestCommands -= 2;
                    CurVertex[0] = ExecParams[0] & 0xFFFF;
                    CurVertex[1] = ExecParams[0] >> 16;
                    CurVertex[2] = ExecParams[1] & 0xFFFF;
                    PosTest();
                    break;

                case 0x70: // box test
                    NumTestCommands -= 3;
                    BoxTest(ExecParams);
                    break;

                default:
                    __builtin_unreachable();
                }
#endif // LITEV_GXFIFO_THREADED
            }
        }
    }
#ifdef LITEV_GXFIFO_THREADED
    // Threaded loop-tail: instead of returning, re-run the whole ExecuteCommand
    // body for the next queued command (mirrors Run()'s drain-loop condition).
    // One call drains the batch -> no per-command bl/ret or prologue/epilogue.
#ifdef LITEV_GX_BULK
    if constexpr (Bulk) goto gxfifo_threaded_top;
#endif
    if (CycleCount <= 0 && !PipeEmpty())
    {
        if (NumPushPopCommands == 0) GXStat &= ~(1<<14);
        if (NumTestCommands == 0)    GXStat &= ~(1<<0);
        goto gxfifo_threaded_top;
    }
#endif
}

#ifdef LITEV_GX_BULK
bool GPU3D::BulkReady() const noexcept
{
    return !FlushRequest && PipeEmpty() && FifoEmpty() && CmdStallQueue.IsEmpty();
}

// Geometry DMA in bulk (deterministic, approximate timing). Called by DMA::Run9 with the FIFO
// empty. The words are decoded exactly as WriteToGXFIFO does and the commands run at once,
// ahead of the guest clock: their cycles pile up in CycleCount, so the engine stays busy
// (GXSTAT bit 27) for the same total time, which Run() then counts down. The game sees the
// FIFO empty throughout instead of filling and draining.
void GPU3D::BulkWords(const u32* words, u32 n) noexcept
{
    Run(); // bring the engine clock up to now (the FIFO is empty: only time passes)
#ifdef LITEV_A9HLE_GXCHECK
    if (A9HLE::GxTap) A9HLE::GxTap->insert(A9HLE::GxTap->end(), words, words + n);
#endif

    CmdFIFOEntry q[4*64]; // a word holds at most 4 commands; Run9 passes <= 64 words
    u32 m = 0;
    u32 numCmds = NumCommands, cur = CurCommand, pc = ParamCount, tp = TotalParams;
    for (u32 i = 0; i < n; i++)
    {
        const u32 val = words[i];
        // same decode as WriteToGXFIFO_Inline, with the state in registers
        if (numCmds == 0)
        {
            numCmds = 4;
            cur = val;
            pc = 0;
            tp = CmdNumParams[cur & 0xFF];
            if (tp > 0) continue;
        }
        else
            pc++;

        for (;;)
        {
            if ((cur & 0xFF) || (numCmds == 4 && cur == 0))
            {
                q[m]._contents = 0;
                q[m].Command = cur & 0xFF;
                q[m].Param = val;
                m++;
            }

            if (pc >= tp)
            {
                cur >>= 8;
                numCmds--;
                if (numCmds == 0) break;

                pc = 0;
                tp = CmdNumParams[cur & 0xFF];
            }
            if (pc < tp)
                break;
        }
    }
    NumCommands = numCmds; CurCommand = cur; ParamCount = pc; TotalParams = tp;
    if (m == 0) return;

    GXStat |= (1<<27);
    BulkPtr = q; BulkEnd = q + m;
    ExecuteCommandT<true>();
    // from a SWAP_BUFFERS on: the FIFO, as usual (fits: the FIFO+PIPE hold 260)
    for (; BulkPtr < BulkEnd; BulkPtr++) CmdFIFOWrite_Inline(*BulkPtr);
    BulkPtr = BulkEnd = nullptr;

    if (NumPushPopCommands == 0) GXStat &= ~(1<<14);
    if (NumTestCommands == 0)    GXStat &= ~(1<<0);
    CheckFIFODMA();
    CheckFIFOIRQ();
}
#endif


#ifdef LITEV_GX_CPUSEND
namespace GXSend
{
namespace
{
constexpr u32 kSend32[6] = {0xE080C002, 0xE150000C, 0xB8B00004, 0xB5812000, 0xBAFFFFFB, 0xE12FFF1E};
// ponytail: fixed estimate of the guest loop (cmp + ldm main RAM + str IO + branch per word)
constexpr s32 kCyclesBase = 6, kCyclesPerWord = 12;

bool On()
{
#if defined(__ANDROID__)
    static const bool on = litevGxPropDefault("debug.litev.gxsend", 1) != 0;
#else
    static const bool on = !getenv("LITEV_GXSEND") || atoi(getenv("LITEV_GXSEND")) != 0;
#endif
    return on;
}

bool IsSend32(melonDS::NDS& nds, u32 addr)
{
    for (u32 i = 1; i < 6; i++)
        if (nds.ARM9Read32(addr + i * 4) != kSend32[i]) return false;
    return true;
}
}

int IsHook(NDS& nds, u32 addr, u32 instr)
{
    return instr == kSend32[0] && On() && IsSend32(nds, addr) ? 1 : 0;
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 0 || (cpu->CPSR & 0x20) || cpu->CurInstr != kSend32[0] || !On()) return false;
    const u32 pc = cpu->R[15] - 8;
    melonDS::NDS& nds = cpu->NDS;
    if (!jit && !IsSend32(nds, pc)) return false;
    GPU3D& gx = nds.GPU.GPU3D;
    const u32 src = cpu->R[0], dst = cpu->R[1], size = cpu->R[2];
    // main RAM or DTCM (all of the range in one, no wrap) -> GXFIFO, whole words, FIFO empty:
    // else the guest loop runs
    auto* c9 = (melonDS::ARMv5*)cpu;
    const u32* w = nullptr;
    if ((dst & ~0x3Fu) == 0x04000400 && !(src & 3) && !(size & 3) && (s32)size > 0 && size <= 0x4000
        && gx.GeometryEnabled && gx.BulkReady())
    {
        const u32 last = src + size - 4;
        const bool d0 = (src & c9->DTCMMask) == c9->DTCMBase, d1 = (last & c9->DTCMMask) == c9->DTCMBase;
        if (d0 && d1 && (src & (DTCMPhysicalSize - 1)) <= (last & (DTCMPhysicalSize - 1)))
            w = (const u32*)&c9->DTCM[src & (DTCMPhysicalSize - 1)];
        else if (!d0 && !d1 && src >= c9->ITCMSize && (src >> 24) == 0x02 && (last >> 24) == 0x02
                 && (src & nds.MainRAMMask) <= (last & nds.MainRAMMask))
            w = (const u32*)&nds.MainRAM[src & nds.MainRAMMask];
    }
    if (!w)
    {
        u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
        ARMInterpreter::ARMInstrTable[icode](cpu);   // the add; the guest loop follows
        return true;
    }
    const u32 n = size >> 2;
    u32 done = 0;
    while (done < n && gx.BulkReady())
    {
        const u32 m = n - done < 64 ? n - done : 64;
        gx.BulkWords(w + done, m);
        done += m;
    }
    const u32 end = src + size;
    cpu->R[2] = w[done - 1];
    cpu->R[0] = src + done * 4;
    cpu->R[12] = end;
    cpu->Cycles += kCyclesBase + kCyclesPerWord * (s32)done;
    if (done < n)
    {
        cpu->JumpTo(pc + 4);        // FIFO no longer empty (a SWAP_BUFFERS queued): the guest loop goes on
        return true;
    }
    cpu->CPSR = (cpu->CPSR & 0x0FFFFFFF) | 0x60000000;   // cmp r0, ip: equal
    cpu->JumpTo(cpu->R[14]);
    return true;
}
}
#endif

s32 GPU3D::CyclesToRunFor() const noexcept
{
    if (CycleCount < 0) return 0;
    return CycleCount;
}

void GPU3D::FinishWork(s32 cycles) noexcept
{
    AddCycles(cycles);
    if (NormalPipeline)
        NormalPipeline -= std::min(NormalPipeline, cycles);

    CycleCount = 0;

    if (VertexPipeline || NormalPipeline || PolygonPipeline)
        return;

    GXStat &= ~(1<<27);
}

void GPU3D::Run() noexcept
{
    if (!GeometryEnabled || FlushRequest ||
        (PipeEmpty() && !(GXStat & (1<<27))))
    {
        Timestamp = NDS.ARM9Timestamp >> NDS.ARM9ClockShift;
        return;
    }

    s32 cycles = (NDS.ARM9Timestamp >> NDS.ARM9ClockShift) - Timestamp;
    CycleCount -= cycles;
    Timestamp = NDS.ARM9Timestamp >> NDS.ARM9ClockShift;

#ifdef LITEV_GXFIFO_THREADED
    // DraStic #3: one ExecuteCommand() call drains the whole batch via its threaded
    // loop-tail (no per-command call/ret). Equivalent to the while loop below.
    if (CycleCount <= 0 && !PipeEmpty())
    {
        if (NumPushPopCommands == 0) GXStat &= ~(1<<14);
        if (NumTestCommands == 0)    GXStat &= ~(1<<0);
        ExecuteCommand();
    }
#else
    if (CycleCount <= 0)
    {
        while (CycleCount <= 0 && !PipeEmpty())
        {
            if (NumPushPopCommands == 0) GXStat &= ~(1<<14);
            if (NumTestCommands == 0)    GXStat &= ~(1<<0);

            ExecuteCommand();
        }
    }
#endif

    if (CycleCount <= 0 && PipeEmpty())
    {
        if (GXStat & (1<<27)) FinishWork(-CycleCount);
        else                  CycleCount = 0;

        if (NumPushPopCommands == 0) GXStat &= ~(1<<14);
        if (NumTestCommands == 0)    GXStat &= ~(1<<0);
    }
}


void GPU3D::CheckFIFOIRQ() noexcept
{
    bool irq = false;
    switch (GXStat >> 30)
    {
    case 1: irq = (FifoLevel() < 128); break;
    case 2: irq = FifoEmpty(); break;
    }

    // Only touch the IRQ line when the GXFIFO IRQ state actually CHANGES. CmdFIFORead fires this
    // every ~2 commands; unguarded it calls SetIRQ/ClearIRQ -> UpdateIRQ every time, redundantly
    // recomputing an unchanged ARM9 IRQ line (with no GXFIFO IRQ configured — GXStat>>30==0 — that's
    // a ClearIRQ per read). Bit-exact: this is the ARM9 (cpu 0, no ARM7 sleep-wakeup path) and
    // UpdateIRQ is idempotent when IF is unchanged (IE/IME writes run their own UpdateIRQ), so the
    // skipped Set/Clear are pure no-ops. debug.litev.irqguard=0 restores the unconditional path.
#if defined(__ANDROID__)
    static const int _irqguard = litevGxPropDefault("debug.litev.irqguard", 1);
#else
    static const int _irqguard = 1;
#endif
    if (_irqguard)
    {
        const bool cur = (NDS.IF[0] >> IRQ_GXFIFO) & 1;
        if (irq && !cur)      NDS.SetIRQ(0, IRQ_GXFIFO);
        else if (!irq && cur) NDS.ClearIRQ(0, IRQ_GXFIFO);
    }
    else
    {
        if (irq) NDS.SetIRQ(0, IRQ_GXFIFO);
        else     NDS.ClearIRQ(0, IRQ_GXFIFO);
    }
}

void GPU3D::CheckFIFODMA() noexcept
{
    if (FifoLevel() < 128)
        NDS.CheckDMAs(0, 0x07);
}


bool YSort(Polygon* a, Polygon* b)
{
    // polygon sorting rules:
    // * opaque polygons come first
    // * polygons with lower bottom Y come first
    // * upon equal bottom Y, polygons with lower top Y come first
    // * upon equal bottom AND top Y, original ordering is used
    // the SortKey is calculated as to implement these rules

    return a->SortKey < b->SortKey;
}

void GPU3D::VBlank() noexcept
{
    if (GeometryEnabled)
    {
        // a console nobody watches skips preparing the frame for the renderer (polygon sort,
        // Render* copies): only the renderer reads that state
#ifdef LITEV_FF_HEADLESS3D
        // a bank partly built unwatched is never handed to the renderer: skip its preparation and
        // the render it would feed (the last rendered 3D stays on screen)
        const bool badBank = FlushRequest && BankBuiltHeadless;
        if (FlushRequest)
        {
            const u32 period = FFFrame - FFLastFlush;
            FFPeriod = (period >= 1 && period <= 4) ? period : 1;
            FFLastFlush = FFFrame;
            // a bank partly built unwatched lands on a frame that renders 3D: the guess was wrong
            if (badBank && !GPU.SkipThisFrame) FFBackoff = 120;
        }
        if (FlushRequest) RenderStale = !RenderingEnabled || LITEV_HEADLESS(Headless) || badBank;
        if (RenderingEnabled && !LITEV_HEADLESS(Headless) && !badBank)
#else
        if (RenderingEnabled && !LITEV_HEADLESS(Headless))
#endif
        {
#ifdef LITEV_SOFT3D_ASYNC
        // LITEV_SOFT3D_ASYNC: if this VBlank changes nothing the renderer reads, skip
        // the whole Render* rewrite. Every store below would be value-identical
        // anyway (that is exactly what NeedsRenderBarrier() tested), so skipping is
        // semantically a no-op — but it also means we touch NO renderer-visible state
        // while an async raster is still reading it, which is what lets GPU::VBlank
        // sail past without a barrier.
        if (!NeedsRenderBarrier())
        {
            RenderFrameIdentical = true;
        }
        else
#endif
        {
            if (FlushRequest)
            {
                if (NumPolygons)
                {
                    // separate translucent polygons from opaque ones

                    u32 io = 0, it = NumOpaquePolygons;
                    for (u32 i = 0; i < NumPolygons; i++)
                    {
                        Polygon* poly = &CurPolygonRAM[i];
                        if (poly->Translucent)
                            RenderPolygonRAM[it++] = poly;
                        else
                            RenderPolygonRAM[io++] = poly;
                    }

                    // apply Y-sorting

#ifdef LITEV_POLY_RADIX
                    // 4-pass LSD radix on the u32 SortKey: each pass is a stable counting
                    // scatter, so the final permutation is IDENTICAL to
                    // std::stable_sort(YSort) (same single-key order; equal keys keep
                    // submission order) — but with zero per-frame allocation and linear
                    // scatters instead of merge memcpys. stable_sort here was the #2
                    // emu-thread backend-stall source (~13%, attribution 2026-07-17).
                    {
                        const u32 n = (FlushAttributes & 0x1) ? NumOpaquePolygons : NumPolygons;
                        Polygon** src = RenderPolygonRAM.data();
                        Polygon** dst = PolySortScratch.data();
                        for (int shift = 0; shift < 32; shift += 8)
                        {
                            u32 cnt[257] = {};
                            for (u32 i = 0; i < n; i++) cnt[((src[i]->SortKey >> shift) & 0xFF) + 1]++;
                            for (int b = 0; b < 256; b++) cnt[b + 1] += cnt[b];
                            for (u32 i = 0; i < n; i++) dst[cnt[(src[i]->SortKey >> shift) & 0xFF]++] = src[i];
                            std::swap(src, dst);
                        }
                        // 4 passes (even count) -> the sorted order is back in
                        // RenderPolygonRAM[0..n); entries >= n were never touched.
                    }
#else
                    std::stable_sort(RenderPolygonRAM.begin(),
                        RenderPolygonRAM.begin() + ((FlushAttributes & 0x1) ? NumOpaquePolygons : NumPolygons),
                        YSort);
#endif
                }

                RenderNumPolygons = NumPolygons;
                RenderFrameIdentical = false;
            }
            else
            {
                RenderFrameIdentical = RenderDispCnt == DispCnt
                    && RenderAlphaRef == AlphaRef
                    && RenderClearAttr1 == ClearAttr1
                    && RenderClearAttr2 == ClearAttr2
                    && RenderFogColor == FogColor
                    && RenderFogOffset == FogOffset * 0x200
                    && memcmp(RenderEdgeTable, EdgeTable, 8*2) == 0
                    && memcmp(RenderFogDensityTable + 1, FogDensityTable, 32) == 0
                    && memcmp(RenderToonTable, ToonTable, 32*2) == 0;
            }

            RenderDispCnt = DispCnt;
            RenderAlphaRef = AlphaRef;

            memcpy(RenderEdgeTable, EdgeTable, 8*2);
            memcpy(RenderToonTable, ToonTable, 32*2);

            RenderFogColor = FogColor;
            RenderFogOffset = FogOffset * 0x200;
            RenderFogShift = (RenderDispCnt >> 8) & 0xF;
            RenderFogDensityTable[0] = FogDensityTable[0];
            memcpy(&RenderFogDensityTable[1], FogDensityTable, 32);
            RenderFogDensityTable[33] = FogDensityTable[31];

            RenderClearAttr1 = ClearAttr1;
            RenderClearAttr2 = ClearAttr2;
        }
        }   // LITEV_SOFT3D_ASYNC: close the "renderer-visible state changes" branch

        if (FlushRequest)
        {
            CurRAMBank = CurRAMBank?0:1;
            CurVertexRAM = &VertexRAM[CurRAMBank ? 6144 : 0];
            CurPolygonRAM = &PolygonRAM[CurRAMBank ? 2048 : 0];

            NumVertices = 0;
            NumPolygons = 0;
            NumOpaquePolygons = 0;
#ifdef LITEV_FF_HEADLESS3D
            BankBuiltHeadless = false;
#endif

            FlushRequest = 0;
        }
    }
}


void GPU3D::SetRenderXPos(u16 xpos, u16 mask) noexcept
{
    if (!RenderingEnabled) return;

    RenderXPos = (RenderXPos & ~mask) | (xpos & mask & 0x01FF);
}


void GPU3D::WriteToGXFIFO(u32 val) noexcept
{
    // Single source of truth: the body lives in GPU3D_GXFIFO_inl.h so DMA.cpp can inline it.
    WriteToGXFIFO_Inline(val);
}


u8 GPU3D::Read8(u32 addr) noexcept
{
    switch (addr)
    {
    case 0x04000600:
        Run();
        return GXStat & 0xFF;
    case 0x04000601:
        {
            Run();
            return ((GXStat >> 8) & 0xFF) |
                   (PosMatrixStackPointer & 0x1F) |
                   ((ProjMatrixStackPointer & 0x1) << 5);
        }
    case 0x04000602:
        {
            Run();

            u32 fifolevel = FifoLevel();

            return fifolevel & 0xFF;
        }
    case 0x04000603:
        {
            Run();

            u32 fifolevel = FifoLevel();

            return ((GXStat >> 24) & 0xFF) |
                   (fifolevel >> 8) |
                   (fifolevel < 128 ? (1<<1) : 0) |
                   (fifolevel == 0  ? (1<<2) : 0);
        }
    }

    Log(LogLevel::Debug, "unknown GPU3D read8 %08X\n", addr);
    return 0;
}

u16 GPU3D::Read16(u32 addr) noexcept
{
    switch (addr)
    {
    case 0x04000060:
        return DispCnt;

    case 0x04000320:
        return 46; // TODO, eventually

    case 0x04000600:
        {
            Run();

            return (GXStat & 0xFFFF) |
                   ((PosMatrixStackPointer & 0x1F) << 8) |
                   ((ProjMatrixStackPointer & 0x1) << 13);
        }
    case 0x04000602:
        {
            Run();

            u32 fifolevel = FifoLevel();

            return (GXStat >> 16) |
                   fifolevel |
                   (fifolevel < 128 ? (1<<9) : 0) |
                   (fifolevel == 0  ? (1<<10) : 0);
        }

    case 0x04000604:
        return NumPolygons;
    case 0x04000606:
        return NumVertices;

    case 0x04000630: return VecTestResult[0];
    case 0x04000632: return VecTestResult[1];
    case 0x04000634: return VecTestResult[2];
    }

    Log(LogLevel::Debug, "unknown GPU3D read16 %08X\n", addr);
    return 0;
}

u32 GPU3D::Read32(u32 addr) noexcept
{
    switch (addr)
    {
    case 0x04000060:
        return DispCnt;

    case 0x04000320:
        return 46; // TODO, eventually

    case 0x04000600:
        {
            Run();

            u32 fifolevel = FifoLevel();

            return GXStat |
                   ((PosMatrixStackPointer & 0x1F) << 8) |
                   ((ProjMatrixStackPointer & 0x1) << 13) |
                   (fifolevel << 16) |
                   (fifolevel < 128 ? (1<<25) : 0) |
                   (fifolevel == 0  ? (1<<26) : 0);
        }

    case 0x04000604:
        return NumPolygons | (NumVertices << 16);

    case 0x04000620: return PosTestResult[0];
    case 0x04000624: return PosTestResult[1];
    case 0x04000628: return PosTestResult[2];
    case 0x0400062C: return PosTestResult[3];

    case 0x04000680: return VecMatrix[0];
    case 0x04000684: return VecMatrix[1];
    case 0x04000688: return VecMatrix[2];
    case 0x0400068C: return VecMatrix[4];
    case 0x04000690: return VecMatrix[5];
    case 0x04000694: return VecMatrix[6];
    case 0x04000698: return VecMatrix[8];
    case 0x0400069C: return VecMatrix[9];
    case 0x040006A0: return VecMatrix[10];
    }

    if (addr >= 0x04000640 && addr < 0x04000680)
    {
        UpdateClipMatrix();
        return ClipMatrix[(addr & 0x3C) >> 2];
    }

    //printf("unknown GPU3D read32 %08X\n", addr);
    return 0;
}

void GPU3D::Write8(u32 addr, u8 val) noexcept
{
#ifdef LITEV_A9HLE_GXCHECK
    if (A9HLE::GxTap && addr >= 0x04000400 && addr < 0x040005CC) A9HLE::GxOtherSeen = true;
#endif
    if (!RenderingEnabled && addr >= 0x04000320 && addr < 0x04000400) return;
    if (!GeometryEnabled  && addr >= 0x04000400 && addr < 0x04000700) return;

    switch (addr)
    {
    case 0x04000340:
        AlphaRefVal = val & 0x1F;
        AlphaRef = (DispCnt & (1<<2)) ? AlphaRefVal : 0;
        return;

    case 0x04000601:
        if (val & 0x80)
        {
            GXStat &= ~0x8000;
            ProjMatrixStackPointer = 0;
            //PosMatrixStackPointer = 0;
            TexMatrixStackPointer = 0; // CHECKME
        }
        return;
    case 0x04000603:
        val &= 0xC0;
        GXStat &= 0x3FFFFFFF;
        GXStat |= (val << 24);
        CheckFIFOIRQ();
        return;
    }

    if (addr >= 0x04000330 && addr < 0x04000340)
    {
        ((u8*)EdgeTable)[addr - 0x04000330] = val;
        return;
    }

    if (addr >= 0x04000360 && addr < 0x04000380)
    {
        FogDensityTable[addr - 0x04000360] = val & 0x7F;
        return;
    }

    if (addr >= 0x04000380 && addr < 0x040003C0)
    {
        ((u8*)ToonTable)[addr - 0x04000380] = val;
        return;
    }

    Log(LogLevel::Debug, "unknown GPU3D write8 %08X %02X\n", addr, val);
}

void GPU3D::Write16(u32 addr, u16 val) noexcept
{
#ifdef LITEV_A9HLE_GXCHECK
    if (A9HLE::GxTap && addr >= 0x04000400 && addr < 0x040005CC) A9HLE::GxOtherSeen = true;
#endif
    if (!RenderingEnabled && addr >= 0x04000320 && addr < 0x04000400) return;
    if (!GeometryEnabled  && addr >= 0x04000400 && addr < 0x04000700) return;

    switch (addr)
    {
    case 0x04000060:
        DispCnt = (val & 0x4FFF) | (DispCnt & 0x3000);
        if (val & (1<<12)) DispCnt &= ~(1<<12);
        if (val & (1<<13)) DispCnt &= ~(1<<13);
        AlphaRef = (DispCnt & (1<<2)) ? AlphaRefVal : 0;
        return;

    case 0x04000340:
        AlphaRefVal = val & 0x1F;
        AlphaRef = (DispCnt & (1<<2)) ? AlphaRefVal : 0;
        return;

    case 0x04000350:
        ClearAttr1 = (ClearAttr1 & 0xFFFF0000) | val;
        return;
    case 0x04000352:
        ClearAttr1 = (ClearAttr1 & 0xFFFF) | (val << 16);
        return;
    case 0x04000354:
        ClearAttr2 = (ClearAttr2 & 0xFFFF0000) | val;
        return;
    case 0x04000356:
        ClearAttr2 = (ClearAttr2 & 0xFFFF) | (val << 16);
        return;

    case 0x04000358:
        FogColor = (FogColor & 0xFFFF0000) | val;
        return;
    case 0x0400035A:
        FogColor = (FogColor & 0xFFFF) | (val << 16);
        return;
    case 0x0400035C:
        FogOffset = val & 0x7FFF;
        return;

    case 0x04000600:
        if (val & 0x8000)
        {
            GXStat &= ~0x8000;
            ProjMatrixStackPointer = 0;
            //PosMatrixStackPointer = 0;
            TexMatrixStackPointer = 0; // CHECKME
        }
        return;
    case 0x04000602:
        val &= 0xC000;
        GXStat &= 0x3FFFFFFF;
        GXStat |= (val << 16);
        CheckFIFOIRQ();
        return;

    case 0x04000610:
        val &= 0x7FFF;
        ZeroDotWLimit = (val * 0x200) + 0x1FF;
        return;
    }

    if (addr >= 0x04000330 && addr < 0x04000340)
    {
        EdgeTable[(addr - 0x04000330) >> 1] = val;
        return;
    }

    if (addr >= 0x04000360 && addr < 0x04000380)
    {
        addr -= 0x04000360;
        FogDensityTable[addr] = val & 0x7F;
        FogDensityTable[addr+1] = (val >> 8) & 0x7F;
        return;
    }

    if (addr >= 0x04000380 && addr < 0x040003C0)
    {
        ToonTable[(addr - 0x04000380) >> 1] = val;
        return;
    }

    Log(LogLevel::Debug, "unknown GPU3D write16 %08X %04X\n", addr, val);
}

void GPU3D::Write32(u32 addr, u32 val) noexcept
{
#ifdef LITEV_A9HLE_GXCHECK
    if (A9HLE::GxTap && addr >= 0x04000440 && addr < 0x040005CC) A9HLE::GxOtherSeen = true;
#endif
    if (!RenderingEnabled && addr >= 0x04000320 && addr < 0x04000400) return;
    if (!GeometryEnabled  && addr >= 0x04000400 && addr < 0x04000700) return;

    switch (addr)
    {
    case 0x04000060:
        DispCnt = (val & 0x4FFF) | (DispCnt & 0x3000);
        if (val & (1<<12)) DispCnt &= ~(1<<12);
        if (val & (1<<13)) DispCnt &= ~(1<<13);
        AlphaRef = (DispCnt & (1<<2)) ? AlphaRefVal : 0;
        return;

    case 0x04000340:
        AlphaRefVal = val & 0x1F;
        AlphaRef = (DispCnt & (1<<2)) ? AlphaRefVal : 0;
        return;

    case 0x04000350:
        ClearAttr1 = val;
        return;
    case 0x04000354:
        ClearAttr2 = val;
        return;

    case 0x04000358:
        FogColor = val;
        return;
    case 0x0400035C:
        FogOffset = val & 0x7FFF;
        return;

    case 0x04000600:
        if (val & 0x8000)
        {
            GXStat &= ~0x8000;
            ProjMatrixStackPointer = 0;
            //PosMatrixStackPointer = 0;
            TexMatrixStackPointer = 0; // CHECKME
        }
        val &= 0xC0000000;
        GXStat &= 0x3FFFFFFF;
        GXStat |= val;
        CheckFIFOIRQ();
        return;

    case 0x04000610:
        val &= 0x7FFF;
        ZeroDotWLimit = (val * 0x200) + 0x1FF;
        return;
    }

    if (addr >= 0x04000400 && addr < 0x04000440)
    {
        WriteToGXFIFO(val);
        return;
    }

    if (addr >= 0x04000440 && addr < 0x040005CC)
    {
        CmdFIFOEntry entry;
        entry.Command = (addr & 0x1FC) >> 2;
        entry.Param = val;
        CmdFIFOWrite(entry);
        return;
    }

    if (addr >= 0x04000330 && addr < 0x04000340)
    {
        addr = (addr - 0x04000330) >> 1;
        EdgeTable[addr] = val & 0xFFFF;
        EdgeTable[addr+1] = val >> 16;
        return;
    }

    if (addr >= 0x04000360 && addr < 0x04000380)
    {
        addr -= 0x04000360;
        FogDensityTable[addr] = val & 0x7F;
        FogDensityTable[addr+1] = (val >> 8) & 0x7F;
        FogDensityTable[addr+2] = (val >> 16) & 0x7F;
        FogDensityTable[addr+3] = (val >> 24) & 0x7F;
        return;
    }

    if (addr >= 0x04000380 && addr < 0x040003C0)
    {
        addr = (addr - 0x04000380) >> 1;
        ToonTable[addr] = val & 0xFFFF;
        ToonTable[addr+1] = val >> 16;
        return;
    }

    Log(LogLevel::Debug, "unknown GPU3D write32 %08X %08X\n", addr, val);
}

}

