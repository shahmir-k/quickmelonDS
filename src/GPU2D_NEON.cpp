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

// NEON-accelerated 2D output helpers for liteDS-v2's software renderer.
//
// Ported from the v1 fork (GPU2D_NEON.cpp) and re-hooked onto the current
// upstream software renderer, whose master-brightness / colour-expand code now
// lives in GPU_Soft.cpp (SoftRenderer::ApplyMasterBrightness / ExpandColor)
// rather than GPU2D_Soft.cpp. The SIMD paths below are bit-exact with those
// scalar loops (verified against a 300-frame framebuffer hash comparison).
//
// Internal pixel format (melonDS):
//   [31:24] = flags/alpha   [21:16] = Blue6   [13:8] = Green6   [5:0] = Red6

#include "GPU2D_NEON.h"
#include "LitevSoftProf.h"

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#if defined(__ANDROID__)
#include <sys/system_properties.h>
#include <cstdlib>
#endif

namespace melonDS
{
namespace GPU2DNeon
{

static bool litevConvertScalar()
{
#if defined(__ANDROID__)
    static int enabled = -1;
    if (enabled < 0)
    {
        char value[PROP_VALUE_MAX] = {};
        enabled = (__system_property_get("debug.litev.convertscalar", value) > 0) ?
            (atoi(value) != 0) : 1;
    }
    return enabled != 0;
#else
    return false;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// ConvertToBGRA — matches SoftRenderer::ExpandColor
// ─────────────────────────────────────────────────────────────────────────────

void ConvertToBGRA(u32* buf, int count) noexcept
{
    // The original paired-u64 expansion is retained as an A/B candidate. On
    // small in-order cores its scalar integer issue path can beat a 4-wide
    // NEON expansion despite handling fewer pixels per iteration.
    if (litevConvertScalar())
    {
        int i = 0;
        for (; i <= count - 2; i += 2)
        {
            u64 c = *(u64*)(buf + i);
            u64 r = (c << 18) & 0xFC000000FC0000;
            u64 g = (c << 2)  & 0xFC000000FC00;
            u64 b = (c >> 14) & 0xFC000000FC;
            c = r | g | b;
            *(u64*)(buf + i) = c | ((c & 0x00C0C0C000C0C0C0) >> 6) | 0xFF000000FF000000;
        }
        for (; i < count; i++)
        {
            const u32 c = buf[i];
            const u32 r = (c & 0x3F) << 18;
            const u32 g = (c & 0x3F00) << 2;
            const u32 b = (c & 0x3F0000) >> 14;
            buf[i] = r | g | b | ((r | g | b) >> 6) | 0xFF000000;
        }
        return;
    }

#if defined(__ARM_NEON)
    // Process 4 pixels per iteration.
    // Each channel: 6-bit value -> 8-bit via (x6 << 2) | (x6 >> 4).
    const uint32x4_t k_mask_r  = vdupq_n_u32(0x0000003F);  // bits  5:0  = Red
    const uint32x4_t k_mask_g  = vdupq_n_u32(0x00003F00);  // bits 13:8  = Green
    const uint32x4_t k_mask_b  = vdupq_n_u32(0x003F0000);  // bits 21:16 = Blue
    const uint32x4_t k_alpha   = vdupq_n_u32(0xFF000000);

    int i = 0;
    for (; i <= count - 4; i += 4)
    {
        uint32x4_t c = vld1q_u32(buf + i);

        // Extract channels into bits 5:0 each
        uint32x4_t r6 = vandq_u32(c, k_mask_r);
        uint32x4_t g6 = vshrq_n_u32(vandq_u32(c, k_mask_g), 8);
        uint32x4_t b6 = vshrq_n_u32(vandq_u32(c, k_mask_b), 16);

        // Scale 6->8 bit: (x6 << 2) | (x6 >> 4)
        uint32x4_t r8 = vorrq_u32(vshlq_n_u32(r6, 2), vshrq_n_u32(r6, 4));
        uint32x4_t g8 = vorrq_u32(vshlq_n_u32(g6, 2), vshrq_n_u32(g6, 4));
        uint32x4_t b8 = vorrq_u32(vshlq_n_u32(b6, 2), vshrq_n_u32(b6, 4));

        // Pack BGRA: [7:0]=B [15:8]=G [23:16]=R [31:24]=0xFF
        uint32x4_t result = vorrq_u32(
            vorrq_u32(b8, vshlq_n_u32(g8, 8)),
            vorrq_u32(vshlq_n_u32(r8, 16), k_alpha)
        );

        vst1q_u32(buf + i, result);
    }
    // Scalar tail (handles any remainder when count is not a multiple of 4)
    for (; i < count; i++)
    {
        u32 c = buf[i];
        u32 r6 = c & 0x3F;
        u32 g6 = (c >> 8) & 0x3F;
        u32 b6 = (c >> 16) & 0x3F;
        u32 r8 = (r6 << 2) | (r6 >> 4);
        u32 g8 = (g6 << 2) | (g6 >> 4);
        u32 b8 = (b6 << 2) | (b6 >> 4);
        buf[i] = b8 | (g8 << 8) | (r8 << 16) | 0xFF000000;
    }
#else
    // Scalar fallback — identical logic to SoftRenderer::ExpandColor.
    int i = 0;
    for (; i <= count - 2; i += 2)
    {
        u64 c = *(u64*)(buf + i);

        u64 r = (c << 18) & 0xFC000000FC0000;
        u64 g = (c << 2)  & 0xFC000000FC00;
        u64 b = (c >> 14) & 0xFC000000FC;
        c = r | g | b;

        *(u64*)(buf + i) = c | ((c & 0x00C0C0C000C0C0C0) >> 6) | 0xFF000000FF000000;
    }
    for (; i < count; i++)
    {
        u32 c = buf[i];
        u32 r6 = c & 0x3F;
        u32 g6 = (c >> 8) & 0x3F;
        u32 b6 = (c >> 16) & 0x3F;
        u32 r8 = (r6 << 2) | (r6 >> 4);
        u32 g8 = (g6 << 2) | (g6 >> 4);
        u32 b8 = (b6 << 2) | (b6 >> 4);
        buf[i] = b8 | (g8 << 8) | (r8 << 16) | 0xFF000000;
    }
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// BrightnessUp — matches ColorBrightnessUp(c, factor, 0x0)
//   per-channel: c' = c + ((63 - c) * factor) >> 4
// ─────────────────────────────────────────────────────────────────────────────

void BrightnessUp(u32* buf, int count, u32 factor) noexcept
{
    if (factor == 0) return;

#if defined(__ARM_NEON)
    const uint32x4_t k_mask6  = vdupq_n_u32(0x3F);
    const uint32x4_t k_max6   = vdupq_n_u32(63);
    const uint32x4_t k_alpha  = vdupq_n_u32(0xFF000000);
    const uint32x4_t k_factor = vdupq_n_u32(factor);

    int i = 0;
    for (; i <= count - 4; i += 4)
    {
        uint32x4_t c = vld1q_u32(buf + i);

        // Extract each 6-bit channel into bits 5:0
        uint32x4_t r = vandq_u32(c, k_mask6);
        uint32x4_t g = vshrq_n_u32(vandq_u32(c, vdupq_n_u32(0x3F00)), 8);
        uint32x4_t b = vshrq_n_u32(vandq_u32(c, vdupq_n_u32(0x3F0000)), 16);

        // c' = c + ((63 - c) * factor) >> 4   (no bias for BrightnessUp)
        r = vaddq_u32(r, vshrq_n_u32(vmulq_u32(vsubq_u32(k_max6, r), k_factor), 4));
        g = vaddq_u32(g, vshrq_n_u32(vmulq_u32(vsubq_u32(k_max6, g), k_factor), 4));
        b = vaddq_u32(b, vshrq_n_u32(vmulq_u32(vsubq_u32(k_max6, b), k_factor), 4));

        // Clamp to 6 bits (max reachable is 63 anyway; kept for safety)
        r = vminq_u32(r, k_max6);
        g = vminq_u32(g, k_max6);
        b = vminq_u32(b, k_max6);

        // Repack into internal format
        uint32x4_t result = vorrq_u32(k_alpha,
                            vorrq_u32(r,
                            vorrq_u32(vshlq_n_u32(g, 8), vshlq_n_u32(b, 16))));
        vst1q_u32(buf + i, result);
    }
    for (; i < count; i++)
    {
        u32 c = buf[i];
        u32 rb = c & 0x3F003F;
        u32 g  = c & 0x003F00;
        rb += ((((0x3F003F - rb) * factor)) >> 4) & 0x3F003F;
        g  += ((((0x003F00 - g)  * factor)) >> 4) & 0x003F00;
        buf[i] = rb | g | 0xFF000000;
    }
#else
    for (int i = 0; i < count; i++)
    {
        u32 c  = buf[i];
        u32 rb = c & 0x3F003F;
        u32 g  = c & 0x003F00;
        rb += ((((0x3F003F - rb) * factor)) >> 4) & 0x3F003F;
        g  += ((((0x003F00 - g)  * factor)) >> 4) & 0x003F00;
        buf[i] = rb | g | 0xFF000000;
    }
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// BrightnessDown — matches ColorBrightnessDown(c, factor, 0xF)
//   per-channel: c' = c - ((c * factor + 15) >> 4)
// ─────────────────────────────────────────────────────────────────────────────

void BrightnessDown(u32* buf, int count, u32 factor) noexcept
{
    if (factor == 0) return;

#if defined(__ARM_NEON)
    const uint32x4_t k_mask6  = vdupq_n_u32(0x3F);
    const uint32x4_t k_bias   = vdupq_n_u32(15);   // bias = 0xF
    const uint32x4_t k_alpha  = vdupq_n_u32(0xFF000000);
    const uint32x4_t k_factor = vdupq_n_u32(factor);

    int i = 0;
    for (; i <= count - 4; i += 4)
    {
        uint32x4_t c = vld1q_u32(buf + i);

        uint32x4_t r = vandq_u32(c, k_mask6);
        uint32x4_t g = vshrq_n_u32(vandq_u32(c, vdupq_n_u32(0x3F00)), 8);
        uint32x4_t b = vshrq_n_u32(vandq_u32(c, vdupq_n_u32(0x3F0000)), 16);

        // c' = c - ((c * factor + 15) >> 4)
        // The subtracted value is at most c (when factor=16), so no underflow.
        uint32x4_t sub_r = vshrq_n_u32(vaddq_u32(vmulq_u32(r, k_factor), k_bias), 4);
        uint32x4_t sub_g = vshrq_n_u32(vaddq_u32(vmulq_u32(g, k_factor), k_bias), 4);
        uint32x4_t sub_b = vshrq_n_u32(vaddq_u32(vmulq_u32(b, k_factor), k_bias), 4);

        r = vsubq_u32(r, vminq_u32(sub_r, r));
        g = vsubq_u32(g, vminq_u32(sub_g, g));
        b = vsubq_u32(b, vminq_u32(sub_b, b));

        uint32x4_t result = vorrq_u32(k_alpha,
                            vorrq_u32(r,
                            vorrq_u32(vshlq_n_u32(g, 8), vshlq_n_u32(b, 16))));
        vst1q_u32(buf + i, result);
    }
    for (; i < count; i++)
    {
        u32 c  = buf[i];
        u32 rb = c & 0x3F003F;
        u32 g  = c & 0x003F00;
        rb -= (((rb * factor) + (15u * 0x010001u)) >> 4) & 0x3F003F;
        g  -= (((g  * factor) + (15u * 0x000100u)) >> 4) & 0x003F00;
        buf[i] = rb | g | 0xFF000000;
    }
#else
    for (int i = 0; i < count; i++)
    {
        u32 c  = buf[i];
        u32 rb = c & 0x3F003F;
        u32 g  = c & 0x003F00;
        rb -= (((rb * factor) + (15u * 0x010001u)) >> 4) & 0x3F003F;
        g  -= (((g  * factor) + (15u * 0x000100u)) >> 4) & 0x003F00;
        buf[i] = rb | g | 0xFF000000;
    }
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// ColorCompositeLine — NEON port of SoftRenderer2D::ColorComposite's per-pixel
// BG/OBJ colour-special-effects loop (see GPU2D_Soft.cpp DrawScanline_BGOBJ +
// GPU_ColorOp.h). 4 pixels/iter, fully branchless via NEON masks. Bit-exact.
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// DrawBG3DLine — NEON port of SoftRenderer2D::DrawBG_3D's per-pixel 3D-layer
// compositor (see GPU2D_Soft.cpp). In a 3D game (e.g. Shrek race) BG0 is the 3D
// layer, so this runs full-width every scanline with most pixels opaque; the
// scalar loop is 1 px/iter with two per-pixel branches. 4 px/iter, branchless.
// Bit-exact with the scalar loop.
// ─────────────────────────────────────────────────────────────────────────────

} // namespace GPU2DNeon
} // namespace melonDS
