/*
    Copyright 2016-2025 melonDS team
    liteDS-v2 modifications: NEON SIMD 2D output helpers (Milestone 4).
    LITEV_SOFT2D_BITMASK compositor adapted from DSperate
    (src/core/gpu/kernels_neon.cpp composite_line, blend16/brighten16/darken16),
    Copyright (C) 2026 DSperate contributors, GPL-3.0-or-later.

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
#if defined(LITEV_SOFT2D_BITMASK) && defined(__aarch64__)
#define LITEV_S2DBITS_ON 1
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#ifdef LITEV_SOFT2D_NEON
#if defined(__ARM_NEON)

// ColorBlend4(v1,v2,eva,evb) — per-lane eva/evb (uint32x4).
static inline uint32x4_t Blend4_neon(uint32x4_t v1, uint32x4_t v2,
                                     uint32x4_t eva, uint32x4_t evb) noexcept
{
    // Red  (bits 5:0)
    uint32x4_t r = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x00003F)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x00003F)), evb)),
        vdupq_n_u32(0x000008));
    r = vshrq_n_u32(r, 4);
    r = vminq_u32(r, vdupq_n_u32(0x00003F));
    // Green (bits 13:8)
    uint32x4_t g = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x003F00)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x003F00)), evb)),
        vdupq_n_u32(0x000800));
    g = vandq_u32(vshrq_n_u32(g, 4), vdupq_n_u32(0x007F00));
    g = vminq_u32(g, vdupq_n_u32(0x003F00));
    // Blue  (bits 21:16)
    uint32x4_t b = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x3F0000)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x3F0000)), evb)),
        vdupq_n_u32(0x080000));
    b = vandq_u32(vshrq_n_u32(b, 4), vdupq_n_u32(0x7F0000));
    b = vminq_u32(b, vdupq_n_u32(0x3F0000));

    return vorrq_u32(vorrq_u32(vorrq_u32(r, g), b), vdupq_n_u32(0xFF000000));
}

// ColorBlend5(v1,v2) — eva=((v1>>24)&0x1F)+1, evb=32-eva; returns v1 where eva==32.
static inline uint32x4_t Blend5_neon(uint32x4_t v1, uint32x4_t v2) noexcept
{
    uint32x4_t eva = vaddq_u32(vandq_u32(vshrq_n_u32(v1, 24), vdupq_n_u32(0x1F)),
                               vdupq_n_u32(1));
    uint32x4_t evb = vsubq_u32(vdupq_n_u32(32), eva);

    uint32x4_t r = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x00003F)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x00003F)), evb)),
        vdupq_n_u32(0x000010));
    r = vshrq_n_u32(r, 5);
    r = vminq_u32(r, vdupq_n_u32(0x00003F));
    uint32x4_t g = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x003F00)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x003F00)), evb)),
        vdupq_n_u32(0x001000));
    g = vandq_u32(vshrq_n_u32(g, 5), vdupq_n_u32(0x007F00));
    g = vminq_u32(g, vdupq_n_u32(0x003F00));
    uint32x4_t b = vaddq_u32(
        vaddq_u32(vmulq_u32(vandq_u32(v1, vdupq_n_u32(0x3F0000)), eva),
                  vmulq_u32(vandq_u32(v2, vdupq_n_u32(0x3F0000)), evb)),
        vdupq_n_u32(0x100000));
    b = vandq_u32(vshrq_n_u32(b, 5), vdupq_n_u32(0x7F0000));
    b = vminq_u32(b, vdupq_n_u32(0x3F0000));

    uint32x4_t res = vorrq_u32(vorrq_u32(vorrq_u32(r, g), b), vdupq_n_u32(0xFF000000));
    // eva==32 (i.e. (v1>>24)&0x1F == 31) => return v1 unchanged.
    uint32x4_t keep = vceqq_u32(eva, vdupq_n_u32(32));
    return vbslq_u32(keep, v1, res);
}

// ColorBrightnessUp(v1, factor=evy, bias=8)
static inline uint32x4_t BrightUp8_neon(uint32x4_t v1, uint32x4_t evy) noexcept
{
    uint32x4_t rb = vandq_u32(v1, vdupq_n_u32(0x3F003F));
    uint32x4_t g  = vandq_u32(v1, vdupq_n_u32(0x003F00));
    uint32x4_t drb = vandq_u32(
        vshrq_n_u32(vaddq_u32(vmulq_u32(vsubq_u32(vdupq_n_u32(0x3F003F), rb), evy),
                              vdupq_n_u32(8u * 0x010001u)), 4),
        vdupq_n_u32(0x3F003F));
    uint32x4_t dg = vandq_u32(
        vshrq_n_u32(vaddq_u32(vmulq_u32(vsubq_u32(vdupq_n_u32(0x003F00), g), evy),
                              vdupq_n_u32(8u * 0x000100u)), 4),
        vdupq_n_u32(0x003F00));
    rb = vaddq_u32(rb, drb);
    g  = vaddq_u32(g, dg);
    return vorrq_u32(vorrq_u32(rb, g), vdupq_n_u32(0xFF000000));
}

// ColorBrightnessDown(v1, factor=evy, bias=7)
static inline uint32x4_t BrightDown7_neon(uint32x4_t v1, uint32x4_t evy) noexcept
{
    uint32x4_t rb = vandq_u32(v1, vdupq_n_u32(0x3F003F));
    uint32x4_t g  = vandq_u32(v1, vdupq_n_u32(0x003F00));
    uint32x4_t drb = vandq_u32(
        vshrq_n_u32(vaddq_u32(vmulq_u32(rb, evy), vdupq_n_u32(7u * 0x010001u)), 4),
        vdupq_n_u32(0x3F003F));
    uint32x4_t dg = vandq_u32(
        vshrq_n_u32(vaddq_u32(vmulq_u32(g, evy), vdupq_n_u32(7u * 0x000100u)), 4),
        vdupq_n_u32(0x003F00));
    rb = vsubq_u32(rb, drb);
    g  = vsubq_u32(g, dg);
    return vorrq_u32(vorrq_u32(rb, g), vdupq_n_u32(0xFF000000));
}

// Diagnostic A/B for the complete brightness-up line proof below. It is cached
// at first use: this is a launch-time setting, never a per-line property read.
static bool litevCompositeBrightUp()
{
#if defined(__ANDROID__)
    static int enabled = -1;
    if (enabled < 0)
    {
        char value[PROP_VALUE_MAX] = {};
        // Shipping default is ON; explicit 0 keeps the same-binary A/B path.
        enabled = (__system_property_get("debug.litev.compositebrightup", value) > 0) ?
                  (atoi(value) != 0) : 1;
    }
    return enabled != 0;
#else
    return false;
#endif
}

#ifdef LITEV_S2DBITS_ON
static void ColorCompositeLineOld(u32* dst, const u32* bgobj, const u8* windowMask,
                                  u32 blendCnt, u32 eva_g, u32 evb_g, u32 evy) noexcept
#else
void ColorCompositeLine(u32* dst, const u32* bgobj, const u8* windowMask,
                        u32 blendCnt, u32 eva_g, u32 evb_g, u32 evy) noexcept
#endif
{
#if defined(__ANDROID__) && defined(LITEV_SOFTPROF)
    static const bool profileCE = [] {
        char value[PROP_VALUE_MAX] = {};
        // This counter is a deep diagnostic: it performs four contended atomics
        // per compositor vector. Keep the ordinary whole-frame profiler free of
        // that measurement perturbation.
        return __system_property_get("debug.litev.softprof", value) > 0 && atoi(value) != 0;
    }();
#else
    constexpr bool profileCE = false;
#endif
    const uint32x4_t vblend = vdupq_n_u32(blendCnt);
    const uint32x4_t vEVA   = vdupq_n_u32(eva_g);
    const uint32x4_t vEVB   = vdupq_n_u32(evb_g);
    const uint32x4_t vEVY   = vdupq_n_u32(evy);
    // coloreffect for the "else" (BG/backdrop) branch is a per-scanline scalar.
    const u32 ceElse = (blendCnt >> 6) & 0x3;
    const uint32x4_t vCeElse = vdupq_n_u32(ceElse);

    // A complete, line-wide proof for the common brightness-up case. The old
    // unconditional shortcut was wrong: sprite/3D alpha can override the
    // BG/backdrop effect per pixel. This scan replays precisely the selector
    // predicates (not the blend/brightness arithmetic) and only bypasses the
    // general loop when every output lane is therefore ce==2.
    //
    // Do not hide the explicit CE census when a diagnostic asked for it.
    if (!profileCE && litevCompositeBrightUp() && ceElse == 2)
    {
        bool allBrightUp = true;
        for (int i = 0; i < 256; i++)
        {
            const u32 f1 = bgobj[i] >> 24;
            const u32 f2 = bgobj[256 + i] >> 24;
            const u32 target2 = (f2 & 0x80) ? 0x1000 :
                                ((f2 & 0x40) ? 0x0100 : (f2 << 8));
            const bool blendT2 = (blendCnt & target2) != 0;
            const u32 flag1b = (f1 & 0x80) ? 0x10 :
                               ((f1 & 0x40) ? 0x01 : f1);
            if (((f1 & 0xC0) && blendT2) ||
                !(blendCnt & flag1b) || !(windowMask[i] & 0x20))
            {
                allBrightUp = false;
                break;
            }
        }
        if (allBrightUp)
        {
            for (int i = 0; i < 256; i += 4)
                vst1q_u32(dst + i, BrightUp8_neon(vld1q_u32(bgobj + i), vEVY));
            return;
        }
    }

    for (int i = 0; i < 256; i += 4)
    {
        uint32x4_t v1 = vld1q_u32(bgobj + i);
        uint32x4_t v2 = vld1q_u32(bgobj + 256 + i);
        // window mask: 4 bytes -> 4x u32
        uint32_t wm4;
        __builtin_memcpy(&wm4, windowMask + i, 4);
        uint8x8_t wm8 = vreinterpret_u8_u32(vdup_n_u32(wm4));
        uint16x8_t wm16 = vmovl_u8(wm8);
        uint32x4_t wm = vmovl_u16(vget_low_u16(wm16));

        uint32x4_t f1 = vshrq_n_u32(v1, 24);
        uint32x4_t f2 = vshrq_n_u32(v2, 24);

        // target2 = f2&0x80 ? 0x1000 : (f2&0x40 ? 0x0100 : f2<<8)
        uint32x4_t m2_80 = vtstq_u32(f2, vdupq_n_u32(0x80));
        uint32x4_t m2_40 = vtstq_u32(f2, vdupq_n_u32(0x40));
        uint32x4_t target2 = vshlq_n_u32(f2, 8);
        target2 = vbslq_u32(m2_40, vdupq_n_u32(0x0100), target2);
        target2 = vbslq_u32(m2_80, vdupq_n_u32(0x1000), target2);

        uint32x4_t blendT2 = vtstq_u32(vblend, target2);      // (blendCnt & target2)!=0

        uint32x4_t m1_80 = vtstq_u32(f1, vdupq_n_u32(0x80));
        uint32x4_t m1_40 = vtstq_u32(f1, vdupq_n_u32(0x40));

        uint32x4_t condSprite = vandq_u32(m1_80, blendT2);
        uint32x4_t cond3D     = vandq_u32(m1_40, blendT2);

        // else-branch: flag1b = m1_80?0x10 : (m1_40?0x01 : f1)
        uint32x4_t flag1b = vbslq_u32(m1_40, vdupq_n_u32(0x01), f1);
        flag1b = vbslq_u32(m1_80, vdupq_n_u32(0x10), flag1b);
        uint32x4_t condElse = vandq_u32(vtstq_u32(vblend, flag1b),
                                        vtstq_u32(wm, vdupq_n_u32(0x20)));

        // coloreffect (priority: else < 3D < sprite)
        uint32x4_t ce = vdupq_n_u32(0);
        if (ceElse != 0)
        {
            if (ceElse == 1)
                ce = vbslq_u32(vandq_u32(condElse, blendT2), vdupq_n_u32(1), ce);
            else
                ce = vbslq_u32(condElse, vCeElse, ce);
        }
        ce = vbslq_u32(cond3D,     vdupq_n_u32(4), ce);
        ce = vbslq_u32(condSprite, vdupq_n_u32(1), ce);

#if defined(__ANDROID__) && defined(LITEV_SOFTPROF)
        if (profileCE)
        {
            u32 lanes[4];
            vst1q_u32(lanes, ce);
            for (int lane = 0; lane < 4; lane++)
                LitevSP::S.CompositeCE[lanes[lane]].fetch_add(1, std::memory_order_relaxed);
        }
#endif

        // eva/evb for ce==1: default EVA/EVB, except sprite+alpha (m1_40) lanes.
        uint32x4_t eva = vEVA, evb = vEVB;
        uint32x4_t evaSp = vandq_u32(f1, vdupq_n_u32(0x1F));
        uint32x4_t evbSp = vsubq_u32(vdupq_n_u32(16), evaSp);
        uint32x4_t spAlpha = vandq_u32(condSprite, m1_40);
        eva = vbslq_u32(spAlpha, evaSp, eva);
        evb = vbslq_u32(spAlpha, evbSp, evb);

        // Compute the 4 non-trivial results, select by ce (0 => v1 passthrough).
        uint32x4_t out = v1;
        out = vbslq_u32(vceqq_u32(ce, vdupq_n_u32(1)), Blend4_neon(v1, v2, eva, evb), out);
        out = vbslq_u32(vceqq_u32(ce, vdupq_n_u32(2)), BrightUp8_neon(v1, vEVY), out);
        out = vbslq_u32(vceqq_u32(ce, vdupq_n_u32(3)), BrightDown7_neon(v1, vEVY), out);
        out = vbslq_u32(vceqq_u32(ce, vdupq_n_u32(4)), Blend5_neon(v1, v2), out);

        vst1q_u32(dst + i, out);
    }
}

#ifdef LITEV_S2DBITS_ON
// ─────────────────────────────────────────────────────────────────────────────
// LITEV_SOFT2D_BITMASK: the compositor of DSperate (GPLv3, Copyright (C) 2026
// DSperate contributors; src/core/gpu/kernels_neon.cpp composite_line, see
// docs/techniques/03-2d-engines-and-scanline-pipeline.md §4-5), adapted to the
// melonDS ColorComposite rules. The pixel format is already DSperate's record
// layout (R 0-5, G 8-13, B 16-21, flags 24-31), so vld4 splits 16 pixels into
// R/G/B/flag byte planes. Per 16 pixels: build the three effect masks (sprite
// blend, 3D blend, BLDCNT effect) as byte-lane masks; a group with none copies
// val1 through; otherwise only the effects present in the group are computed,
// as byte arithmetic (u8 x u8 -> u16 multiply-accumulate). Per pixel the
// arithmetic is ColorBlend4/ColorBlend5/ColorBrightnessUp/Down exactly: the
// fields are <= 63, EVA/EVB/EVY <= 16 (GPU2D clamps them; sprite alpha 1..16),
// so every sum fits u16 and the scalar's & 0x7F masks are no-ops.
// ─────────────────────────────────────────────────────────────────────────────
template <int shift>
static inline uint8x16_t BlendBytes(uint8x16_t a, uint8x16_t b, uint8x16_t ea, uint8x16_t eb) noexcept
{
    // (a*ea + b*eb + 2^(shift-1)) >> shift, clamped to 63
    const uint16x8_t rnd = vdupq_n_u16(1u << (shift - 1));
    const uint16x8_t lo = vmlal_u8(vmlal_u8(rnd, vget_low_u8(a), vget_low_u8(ea)), vget_low_u8(b), vget_low_u8(eb));
    const uint16x8_t hi = vmlal_high_u8(vmlal_high_u8(rnd, a, ea), b, eb);
    return vminq_u8(vcombine_u8(vshrn_n_u16(lo, shift), vshrn_n_u16(hi, shift)), vdupq_n_u8(63));
}
// c + (((63 - c) * f + 8) >> 4)
static inline uint8x16_t BrightUpBytes(uint8x16_t c, uint8x16_t f) noexcept
{
    const uint8x16_t inv = vsubq_u8(vdupq_n_u8(63), c);
    const uint16x8_t lo = vmlal_u8(vdupq_n_u16(8), vget_low_u8(inv), vget_low_u8(f));
    const uint16x8_t hi = vmlal_high_u8(vdupq_n_u16(8), inv, f);
    return vaddq_u8(c, vcombine_u8(vshrn_n_u16(lo, 4), vshrn_n_u16(hi, 4)));
}
// c - ((c * f + 7) >> 4)
static inline uint8x16_t BrightDownBytes(uint8x16_t c, uint8x16_t f) noexcept
{
    const uint16x8_t lo = vmlal_u8(vdupq_n_u16(7), vget_low_u8(c), vget_low_u8(f));
    const uint16x8_t hi = vmlal_high_u8(vdupq_n_u16(7), c, f);
    return vsubq_u8(c, vcombine_u8(vshrn_n_u16(lo, 4), vshrn_n_u16(hi, 4)));
}

static void ColorCompositeLineMasked(u32* dst, const u32* bgobj, const u8* windowMask,
                                     u32 blendCnt, u32 eva_g, u32 evb_g, u32 evy) noexcept
{
    const u32 ceElse = (blendCnt >> 6) & 0x3;
    const uint8x16_t bcLo = vdupq_n_u8(ceElse ? (u8)blendCnt : 0);   // no effect -> fx stays empty
    const uint8x16_t bcHi = vdupq_n_u8((u8)(blendCnt >> 8));
    const uint8x16_t vEVA = vdupq_n_u8((u8)eva_g), vEVB = vdupq_n_u8((u8)evb_g), vEVY = vdupq_n_u8((u8)evy);
    const uint8x16_t c80 = vdupq_n_u8(0x80), c40 = vdupq_n_u8(0x40), c3F = vdupq_n_u8(0x3F), c1F = vdupq_n_u8(0x1F);
    const uint8x16_t c10 = vdupq_n_u8(0x10), c01 = vdupq_n_u8(0x01), c20 = vdupq_n_u8(0x20);

    for (int i = 0; i < 256; i += 16)
    {
        const uint8x16x4_t A = vld4q_u8((const u8*)(bgobj + i));         // val1: R, G, B, flag1
        const uint8x16_t f1 = A.val[3];
        const uint8x16_t f2 = vld4q_u8((const u8*)(bgobj + 256 + i)).val[3];
        // target2 bit of val2's layer in BLDCNT[15:8]: OBJ (0x80) -> bit 4, 3D (0x40) -> bit 0, else the layer bits
        const uint8x16_t t2key = vbslq_u8(vtstq_u8(f2, c80), c10, vbslq_u8(vtstq_u8(f2, c40), c01, f2));
        const uint8x16_t blendT2 = vtstq_u8(t2key, bcHi);
        const uint8x16_t m1_80 = vtstq_u8(f1, c80), m1_40 = vtstq_u8(f1, c40);
        const uint8x16_t spr = vandq_u8(m1_80, blendT2);                 // semi-transparent/bitmap sprite blend
        const uint8x16_t b3d = vbicq_u8(vandq_u8(m1_40, blendT2), m1_80); // 3D layer blend
        // BLDCNT effect: first target (OBJ -> bit 4, 3D -> bit 0) and window colour-effect bit
        const uint8x16_t flag1b = vbslq_u8(m1_80, c10, vbslq_u8(m1_40, c01, f1));
        uint8x16_t fx = vandq_u8(vtstq_u8(flag1b, bcLo), vtstq_u8(vld1q_u8(windowMask + i), c20));
        fx = vbicq_u8(fx, vandq_u8(vorrq_u8(m1_80, m1_40), blendT2));
        if (ceElse == 1) fx = vandq_u8(fx, blendT2);

        const bool hasSpr = vmaxvq_u8(spr) != 0, has3D = vmaxvq_u8(b3d) != 0, hasFx = vmaxvq_u8(fx) != 0;
        if (!(hasSpr | has3D | hasFx))
        {
            // the whole group passes val1 through
            vst1q_u32(dst + i,      vld1q_u32(bgobj + i));
            vst1q_u32(dst + i + 4,  vld1q_u32(bgobj + i + 4));
            vst1q_u32(dst + i + 8,  vld1q_u32(bgobj + i + 8));
            vst1q_u32(dst + i + 12, vld1q_u32(bgobj + i + 12));
            continue;
        }

        const uint8x16x4_t B = vld4q_u8((const u8*)(bgobj + 256 + i));
        uint8x16_t a6[3], b6[3], o[3];
        for (int c = 0; c < 3; c++) { a6[c] = vandq_u8(A.val[c], c3F); b6[c] = vandq_u8(B.val[c], c3F); o[c] = A.val[c]; }
        uint8x16_t changed = vdupq_n_u8(0);   // lanes whose result is a fresh colour (alpha byte 0xFF)
        if (hasFx)
        {
            for (int c = 0; c < 3; c++)
            {
                uint8x16_t r;
                if (ceElse == 1)      r = BlendBytes<4>(a6[c], b6[c], vEVA, vEVB);
                else if (ceElse == 2) r = BrightUpBytes(a6[c], vEVY);
                else                  r = BrightDownBytes(a6[c], vEVY);
                o[c] = vbslq_u8(fx, r, o[c]);
            }
            changed = fx;
        }
        if (has3D)
        {
            // ColorBlend5: eva = alpha+1 of 32; eva == 32 returns val1 unchanged
            const uint8x16_t a3 = vaddq_u8(vandq_u8(f1, c1F), c01);
            const uint8x16_t eb3 = vsubq_u8(vdupq_n_u8(32), a3);
            const uint8x16_t sel = vbicq_u8(b3d, vceqq_u8(a3, vdupq_n_u8(32)));
            for (int c = 0; c < 3; c++) o[c] = vbslq_u8(sel, BlendBytes<5>(a6[c], b6[c], a3, eb3), o[c]);
            changed = vorrq_u8(changed, sel);
        }
        if (hasSpr)
        {
            // bitmap sprites (0xC0) blend with their own alpha: eva = flag & 0x1F, evb = 16 - eva
            const uint8x16_t ea = vbslq_u8(m1_40, vandq_u8(f1, c1F), vEVA);
            const uint8x16_t eb = vbslq_u8(m1_40, vsubq_u8(c10, vandq_u8(f1, c1F)), vEVB);
            for (int c = 0; c < 3; c++) o[c] = vbslq_u8(spr, BlendBytes<4>(a6[c], b6[c], ea, eb), o[c]);
            changed = vorrq_u8(changed, spr);
        }
        const uint8x16x4_t rec = {{o[0], o[1], o[2], vbslq_u8(changed, vdupq_n_u8(0xFF), f1)}};
        vst4q_u8((u8*)(dst + i), rec);
    }
}

// LITEV_S2DBITS_CHECK=1 (env): run the old compositor too and count mismatching pixels (printed at exit).
static std::atomic<u64> S2DBitsLines{0}, S2DBitsBad{0}, S2DBitsFx{0}, S2DBitsCe[4];
static bool litevS2DBitsCheck()
{
    static const bool on = [] {
        const char* e = getenv("LITEV_S2DBITS_CHECK");
        if (!(e && atoi(e))) return false;
        atexit([] { fprintf(stderr, "S2DBITS check: lines=%llu bad_pixels=%llu effect_pixels=%llu lines_by_bldcnt_effect=%llu/%llu/%llu/%llu\n",
                            (unsigned long long)S2DBitsLines.load(), (unsigned long long)S2DBitsBad.load(), (unsigned long long)S2DBitsFx.load(),
                            (unsigned long long)S2DBitsCe[0].load(), (unsigned long long)S2DBitsCe[1].load(),
                            (unsigned long long)S2DBitsCe[2].load(), (unsigned long long)S2DBitsCe[3].load()); });
        return true;
    }();
    return on;
}

void ColorCompositeLine(u32* dst, const u32* bgobj, const u8* windowMask,
                        u32 blendCnt, u32 eva, u32 evb, u32 evy) noexcept
{
    ColorCompositeLineMasked(dst, bgobj, windowMask, blendCnt, eva, evb, evy);
    if (!litevS2DBitsCheck()) return;
    alignas(16) u32 ref[256];
    ColorCompositeLineOld(ref, bgobj, windowMask, blendCnt, eva, evb, evy);
    u64 bad = 0;
    u64 fx = 0;
    for (int i = 0; i < 256; i++) { bad += ref[i] != dst[i]; fx += ref[i] != bgobj[i]; }
    S2DBitsLines.fetch_add(1, std::memory_order_relaxed);
    S2DBitsFx.fetch_add(fx, std::memory_order_relaxed);
    S2DBitsCe[(blendCnt >> 6) & 3].fetch_add(1, std::memory_order_relaxed);
    if (bad)
    {
        if (S2DBitsBad.fetch_add(bad, std::memory_order_relaxed) == 0)
            for (int i = 0; i < 256; i++)
                if (ref[i] != dst[i])
                {
                    fprintf(stderr, "S2DBITS mismatch px %d: ref %08x got %08x val1 %08x val2 %08x win %02x bc %04x eva %u evb %u evy %u\n",
                            i, ref[i], dst[i], bgobj[i], bgobj[256 + i], windowMask[i], blendCnt, eva, evb, evy);
                    break;
                }
        memcpy(dst, ref, sizeof(ref));   // keep the output exact while diagnosing
    }
}
#endif // LITEV_S2DBITS_ON

#endif // __ARM_NEON
#endif // LITEV_SOFT2D_NEON

// ─────────────────────────────────────────────────────────────────────────────
// DrawBG3DLine — NEON port of SoftRenderer2D::DrawBG_3D's per-pixel 3D-layer
// compositor (see GPU2D_Soft.cpp). In a 3D game (e.g. Shrek race) BG0 is the 3D
// layer, so this runs full-width every scanline with most pixels opaque; the
// scalar loop is 1 px/iter with two per-pixel branches. 4 px/iter, branchless.
// Bit-exact with the scalar loop.
// ─────────────────────────────────────────────────────────────────────────────
#ifdef LITEV_SOFT2D_BG3DNEON
#if defined(__ARM_NEON)

void DrawBG3DLine(u32* bgobj, const u32* out3d, const u8* windowMask) noexcept
{
    const uint32x4_t opaque = vdupq_n_u32(0xFF000000);
    const uint32x4_t win0   = vdupq_n_u32(0x01);
    const uint32x4_t tag    = vdupq_n_u32(0x40000000);

    for (int i = 0; i < 256; i += 4)
    {
        // Load 4 window bytes safely (WindowMask is exactly 256 B; no over-read).
        u32 wbits;
        __builtin_memcpy(&wbits, windowMask + i, 4);
        uint8x8_t  wm8  = vreinterpret_u8_u32(vdup_n_u32(wbits));
        uint32x4_t wm32 = vmovl_u16(vget_low_u16(vmovl_u8(wm8)));
        uint32x4_t wmask = vtstq_u32(wm32, win0);            // window layer-0 bit

        uint32x4_t c     = vld1q_u32(out3d + i);
        uint32x4_t amask = vtstq_u32(c, opaque);             // (c>>24) != 0
        uint32x4_t m     = vandq_u32(amask, wmask);

        uint32x4_t oldlow  = vld1q_u32(bgobj + i);
        uint32x4_t oldhigh = vld1q_u32(bgobj + i + 256);
        uint32x4_t newval  = vorrq_u32(c, tag);

        // where m: bgobj[i+256]=oldlow, bgobj[i]=c|0x40000000; else unchanged.
        vst1q_u32(bgobj + i + 256, vbslq_u32(m, oldlow, oldhigh));
        vst1q_u32(bgobj + i,       vbslq_u32(m, newval, oldlow));
    }
}

#endif // __ARM_NEON
#endif // LITEV_SOFT2D_BG3DNEON

} // namespace GPU2DNeon
} // namespace melonDS
