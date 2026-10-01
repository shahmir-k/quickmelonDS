/*
    Copyright 2016-2025 melonDS team
    liteDS-v2 modifications: NEON SIMD 2D output helpers (Milestone 4).

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

#include "types.h"

// NEON-accelerated 2D output helpers for the software renderer.
//
// These are bit-exact SIMD ports of the scalar loops in
// GPU_Soft.cpp::ApplyMasterBrightness and GPU_Soft.cpp::ExpandColor.
// Each function keeps a scalar tail for counts that are not a multiple of the
// vector width. The whole file is only compiled when LITEV_NEON_RENDERER is
// enabled AND the target is AArch64 (see src/CMakeLists.txt), so the NEON path
// is always taken; the scalar fallbacks are retained for reference/safety.
//
// Internal pixel format (melonDS software renderer, pre-ExpandColor):
//   [31:24] = flags/alpha   [21:16] = Blue6   [13:8] = Green6   [5:0] = Red6
//
// BGRA output format (after ExpandColor):
//   [31:24] = 0xFF   [23:16] = Red8   [15:8] = Green8   [7:0] = Blue8
//   where X8 = (X6 << 2) | (X6 >> 4)   (standard 6->8 bit expansion)

namespace melonDS
{
namespace GPU2DNeon
{

// Convert buf[0..count) from internal 6-bit format to 32-bit BGRA in-place.
// Bit-exact with the scalar u64 loop in SoftRenderer::ExpandColor.
void ConvertToBGRA(u32* buf, int count) noexcept;

// Apply master brightness increase to buf[0..count).
// Bit-exact with ColorBrightnessUp(buf[i], factor, 0x0) per pixel.
void BrightnessUp(u32* buf, int count, u32 factor) noexcept;

// Apply master brightness decrease to buf[0..count).
// Bit-exact with ColorBrightnessDown(buf[i], factor, 0xF) per pixel.
void BrightnessDown(u32* buf, int count, u32 factor) noexcept;

#ifdef LITEV_SOFT2D_NEON
// NEON port of SoftRenderer2D::ColorComposite's per-pixel BG/OBJ colour-special-
// effects loop (the DrawScanline_BGOBJ final compositor). Processes 4 pixels per
// iteration, branchless via NEON masks/selects.
//   dst[i]        = composited output pixel (256 pixels)
//   bgobj[0..255] = top-layer values (val1); bgobj[256..511] = bottom (val2)
//   windowMask    = per-pixel window mask byte
//   blendCnt, eva, evb, evy = GPU2D.BlendCnt / EVA / EVB / EVY
// Bit-exact with the scalar ColorComposite loop. 256 must be a multiple of 4.
void ColorCompositeLine(u32* dst, const u32* bgobj, const u8* windowMask,
                        u32 blendCnt, u32 eva, u32 evb, u32 evy) noexcept;
#endif

#ifdef LITEV_SOFT2D_BG3DNEON
// NEON port of SoftRenderer2D::DrawBG_3D's per-pixel 3D-layer compositor loop.
// For each of the 256 pixels, when the 3D pixel is opaque ((c>>24)!=0) AND the
// window layer-0 bit is set, it shifts the current top BGOBJLine pixel into the
// backing slot (bgobj[i+256] = bgobj[i]) and writes the 3D colour tagged with
// 0x40000000 (bgobj[i] = c | 0x40000000). 4 px/iter, fully branchless via NEON
// masks/selects. Bit-exact with the scalar DrawBG_3D loop. 256 is a multiple of 4.
//   bgobj[0..255]   = top-layer values; bgobj[256..511] = backing/bottom layer
//   out3d[0..255]   = per-line 3D output (Cur3DLine)
//   windowMask      = per-pixel window mask byte (bit 0 = layer-0 visible)
void DrawBG3DLine(u32* bgobj, const u32* out3d, const u8* windowMask) noexcept;
#endif

} // namespace GPU2DNeon
} // namespace melonDS
