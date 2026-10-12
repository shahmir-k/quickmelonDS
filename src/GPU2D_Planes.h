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

// NEON line kernels of the software 2D: layer planes (LITEV_SOFT2D_PLANES), output stage
// (LITEV_SOFT2D_OUTNEON), sprite rows (LITEV_SOFT2D_OBJROW). Ported from / based on DSperate
// (https://github.com/beebono/DSperate, GPL-3.0-or-later, commit f5fa35e,
// src/core/gpu/kernels_neon.cpp: text_ctl, text_pair, text_one, text_row_16, text_row_256,
// merge16/select16, brighten16/darken16, composite_line_fade, output_line, capture_a15,
// capture_blend, obj_plot16), adapted to melonDS's line formats (OBJLine attribute words, 3D line,
// BGOBJLine colour records):
//
// SPDX-License-Identifier: GPL-3.0-or-later
// DSperate - Nintendo DS emulator. Copyright (C) 2026 DSperate contributors.

#pragma once

#include "types.h"
#include <arm_neon.h>
#include <cstring>

namespace melonDS
{

namespace S2DPlanes
{

// plane value: bit 15 opaque, bits 0-14 the index into the layer's colour table (0: transparent)
constexpr u16 LV_OPAQUE = 0x8000;

[[gnu::always_inline]] inline bool UniformU8(const u8* p)
{
    u64 a, b; memcpy(&a, p, 8); memcpy(&b, p + 8, 8);
    const u64 r = (a & 0xFF) * 0x0101010101010101ull;
    return ((a ^ r) | (b ^ r)) == 0;
}

// ---- byte-plane colour arithmetic -------------------------------------------------------------
// c + (((63 - c) * factor + bias) >> 4)
[[gnu::always_inline]] inline uint8x16_t Brighten16(uint8x16_t c, uint8x8_t factor, u16 bias)
{
    const uint8x16_t inv = vsubq_u8(vdupq_n_u8(63), c);
    const uint16x8_t lo = vmlal_u8(vdupq_n_u16(bias), vget_low_u8(inv), factor);
    const uint16x8_t hi = vmlal_u8(vdupq_n_u16(bias), vget_high_u8(inv), factor);
    return vaddq_u8(c, vcombine_u8(vshrn_n_u16(lo, 4), vshrn_n_u16(hi, 4)));
}
// c - ((c * factor + bias) >> 4)
[[gnu::always_inline]] inline uint8x16_t Darken16(uint8x16_t c, uint8x8_t factor, u16 bias)
{
    const uint16x8_t lo = vmlal_u8(vdupq_n_u16(bias), vget_low_u8(c), factor);
    const uint16x8_t hi = vmlal_u8(vdupq_n_u16(bias), vget_high_u8(c), factor);
    return vsubq_u8(c, vcombine_u8(vshrn_n_u16(lo, 4), vshrn_n_u16(hi, 4)));
}

// ---- text BG rows ------------------------------------------------------------------------------
// Two text tiles (16 pixels): flip is a per-half byte reverse gated by ctl bit 4, palette from ctl
// bits 0-3 (pal_shift 4/8/none), index 0 transparent. The u16 values are stored as interleaved low
// and high bytes (low: palette<<4|index or index, high: 0x80 [| palette for pal_shift 8]).
template <int pal_shift>
[[gnu::always_inline]] inline uint8x16_t TextPair(uint8x16_t idx, const u8* ctl, u16* v)
{
    const uint8x8_t c2 = vreinterpret_u8_u16(vld1_dup_u16(reinterpret_cast<const u16*>(ctl)));
    const uint8x16_t ctl16 = vcombine_u8(vdup_lane_u8(c2, 0), vdup_lane_u8(c2, 1));
    idx = vbslq_u8(vtstq_u8(ctl16, vdupq_n_u8(0x10)), vrev64q_u8(idx), idx);
    const uint8x16_t nz = vtstq_u8(idx, idx);
    uint8x16x2_t o;
    if (pal_shift == 4)
    {
        o.val[0] = vandq_u8(nz, vorrq_u8(idx, vshlq_n_u8(ctl16, 4)));   // ctl bit 4 shifts out
        o.val[1] = vandq_u8(nz, vdupq_n_u8(0x80));
    }
    else if (pal_shift == 8)
    {
        o.val[0] = idx;
        o.val[1] = vandq_u8(nz, vorrq_u8(vandq_u8(ctl16, vdupq_n_u8(0xF)), vdupq_n_u8(0x80)));
    }
    else
    {
        o.val[0] = idx;
        o.val[1] = vandq_u8(nz, vdupq_n_u8(0x80));
    }
    vst2q_u8(reinterpret_cast<u8*>(v), o);
    return idx;
}

// One tile, for an odd tail.
[[gnu::always_inline]] inline uint8x8_t TextOne(uint8x8_t idx, u8 c, u16 base, u16* v)
{
    idx = vbsl_u8(vdup_n_u8((c & 0x10) ? 0xFF : 0), vrev64_u8(idx), idx);
    const uint16x8_t i16 = vmovl_u8(idx);
    const uint16x8_t m = vmvnq_u16(vceqq_u16(i16, vdupq_n_u16(0)));
    vst1q_u16(v, vandq_u16(m, vorrq_u16(i16, vdupq_n_u16((u16)(LV_OPAQUE | base)))));
    return idx;
}

// Control bytes for a 33-entry map row: palette (bits 12-15) | hflip (bit 10 -> 4).
inline void TextCtl(const u16* tiles, u8* ctl)
{
    for (u32 t = 0; t < 32; t += 8)
    {
        const uint16x8_t tv = vld1q_u16(tiles + t);
        vst1_u8(ctl + t, vorr_u8(vmovn_u16(vshrq_n_u16(tv, 12)), vand_u8(vshrn_n_u16(tv, 6), vdup_n_u8(0x10))));
    }
    ctl[32] = (u8)((tiles[32] >> 12) | ((tiles[32] >> 6) & 0x10));
}

// 16-colour rows: values 0x8000 | pal<<4 | idx (0 for index 0). Returns whether any is opaque.
inline bool TextRow16(const u8* packed, const u8* ctl, u32 n, u16* v)
{
    uint8x16_t any = vdupq_n_u8(0);
    u32 t = 0;
    for (; t + 2 <= n; t += 2, packed += 8, v += 16)
    {
        const uint8x8_t raw = vld1_u8(packed);
        const uint8x8x2_t nib = vzip_u8(vand_u8(raw, vdup_n_u8(0xF)), vshr_n_u8(raw, 4));
        any = vorrq_u8(any, TextPair<4>(vcombine_u8(nib.val[0], nib.val[1]), ctl + t, v));
    }
    if (t < n)
    {
        u32 w; memcpy(&w, packed, 4);
        const uint8x8_t raw = vreinterpret_u8_u32(vdup_n_u32(w));
        const uint8x8x2_t nib = vzip_u8(vand_u8(raw, vdup_n_u8(0xF)), vshr_n_u8(raw, 4));
        any = vorrq_u8(any, vcombine_u8(TextOne(nib.val[0], ctl[t], (u16)((ctl[t] & 0xF) << 4), v), vdup_n_u8(0)));
    }
    return vmaxvq_u8(any) != 0;
}

// 256-colour rows: values 0x8000 | (ext ? pal<<8 : 0) | idx.
inline bool TextRow256(const u8* rows, const u8* ctl, u32 n, bool ext, u16* v)
{
    uint8x16_t any = vdupq_n_u8(0);
    u32 t = 0;
    for (; t + 2 <= n; t += 2, rows += 16, v += 16)
    {
        const uint8x16_t idx = vld1q_u8(rows);
        any = vorrq_u8(any, ext ? TextPair<8>(idx, ctl + t, v) : TextPair<0>(idx, ctl + t, v));
    }
    if (t < n) any = vorrq_u8(any, vcombine_u8(TextOne(vld1_u8(rows), ctl[t], ext ? (u16)((ctl[t] & 0xF) << 8) : 0, v), vdup_n_u8(0)));
    return vmaxvq_u8(any) != 0;
}

// ---- priority select -----------------------------------------------------------------------------
// Where m8: second = top, top = the layer (value, table id). The layer pushes the previous top down,
// exactly as SoftRenderer2D::DrawPixel does pixel by pixel.
template <bool second>
[[gnu::always_inline]] inline void Merge16(uint8x16_t m8, uint16x8_t v0, uint16x8_t v1, uint8x16_t tid,
                                           u16* top, u8* top_tid, u16* sec, u8* sec_tid)
{
    const int8x16_t ws = vreinterpretq_s8_u8(m8);
    const uint16x8_t m0 = vreinterpretq_u16_s16(vmovl_s8(vget_low_s8(ws))), m1 = vreinterpretq_u16_s16(vmovl_s8(vget_high_s8(ws)));
    const uint16x8_t t0 = vld1q_u16(top), t1 = vld1q_u16(top + 8);
    const uint8x16_t tt = vld1q_u8(top_tid);
    if (second)
    {
        vst1q_u16(sec, vbslq_u16(m0, t0, vld1q_u16(sec)));
        vst1q_u16(sec + 8, vbslq_u16(m1, t1, vld1q_u16(sec + 8)));
        vst1q_u8(sec_tid, vbslq_u8(m8, tt, vld1q_u8(sec_tid)));
    }
    vst1q_u16(top, vbslq_u16(m0, v0, t0));
    vst1q_u16(top + 8, vbslq_u16(m1, v1, t1));
    vst1q_u8(top_tid, vbslq_u8(m8, tid, tt));
}

// A BG plane: opaque (bit 15) and its window bit.
template <bool second, bool win>
inline void SelectBG(const u16* v, const u8* winmask, u8 wbit, u8 tid, u16* top, u8* top_tid, u16* sec, u8* sec_tid)
{
    const uint8x16_t vtid = vdupq_n_u8(tid);
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint16x8_t a = vld1q_u16(v + i), b = vld1q_u16(v + i + 8);
        uint8x16_t m8 = vtstq_u8(vcombine_u8(vshrn_n_u16(a, 8), vshrn_n_u16(b, 8)), vdupq_n_u8(0x80));
        if (win) m8 = vandq_u8(m8, vtstq_u8(vld1q_u8(winmask + i), vdupq_n_u8(wbit)));
        Merge16<second>(m8, a, b, vtid, top + i, top_tid + i, sec + i, sec_tid + i);
    }
}

// u32 lane masks -> one u8x16 mask
[[gnu::always_inline]] inline uint8x16_t Narrow4(uint32x4_t a, uint32x4_t b, uint32x4_t c, uint32x4_t d)
{
    return vcombine_u8(vmovn_u16(vcombine_u16(vmovn_u32(a), vmovn_u32(b))), vmovn_u16(vcombine_u16(vmovn_u32(c), vmovn_u32(d))));
}

// The OBJ layer at one priority: OBJLine words with (word & opaprio) == attr, window bit 0x10. The
// value is the pixel position (the colour is resolved by position).
template <bool second, bool win>
inline void SelectOBJ(const u32* obj, u32 opaprio, u32 attr, const u8* winmask, u8 tid, u16* top, u8* top_tid, u16* sec, u8* sec_tid)
{
    const uint32x4_t vm = vdupq_n_u32(opaprio), va = vdupq_n_u32(attr);
    const uint8x16_t vtid = vdupq_n_u8(tid);
    const uint16x8_t lane = {0, 1, 2, 3, 4, 5, 6, 7};
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint16x8_t p0 = vaddq_u16(lane, vdupq_n_u16((u16)i)), p1 = vaddq_u16(p0, vdupq_n_u16(8));
        uint8x16_t m8 = Narrow4(vceqq_u32(vandq_u32(vld1q_u32(obj + i), vm), va), vceqq_u32(vandq_u32(vld1q_u32(obj + i + 4), vm), va),
                                vceqq_u32(vandq_u32(vld1q_u32(obj + i + 8), vm), va), vceqq_u32(vandq_u32(vld1q_u32(obj + i + 12), vm), va));
        if (win) m8 = vandq_u8(m8, vtstq_u8(vld1q_u8(winmask + i), vdupq_n_u8(0x10)));
        Merge16<second>(m8, p0, p1, vtid, top + i, top_tid + i, sec + i, sec_tid + i);
    }
}

// The 3D layer: alpha (bits 24-31) non-zero, window bit 0x01. The value is the pixel position.
template <bool second, bool win>
inline void Select3D(const u32* l3d, const u8* winmask, u8 tid, u16* top, u8* top_tid, u16* sec, u8* sec_tid)
{
    const uint32x4_t va = vdupq_n_u32(0xFF000000);
    const uint8x16_t vtid = vdupq_n_u8(tid);
    const uint16x8_t lane = {0, 1, 2, 3, 4, 5, 6, 7};
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint16x8_t p0 = vaddq_u16(lane, vdupq_n_u16((u16)i)), p1 = vaddq_u16(p0, vdupq_n_u16(8));
        uint8x16_t m8 = Narrow4(vtstq_u32(vld1q_u32(l3d + i), va), vtstq_u32(vld1q_u32(l3d + i + 4), va),
                                vtstq_u32(vld1q_u32(l3d + i + 8), va), vtstq_u32(vld1q_u32(l3d + i + 12), va));
        if (win) m8 = vandq_u8(m8, vtstq_u8(vld1q_u8(winmask + i), vdupq_n_u8(0x01)));
        Merge16<second>(m8, p0, p1, vtid, top + i, top_tid + i, sec + i, sec_tid + i);
    }
}

// ---- line colour effects ------------------------------------------------------------------------
// The colour effects of a line whose second layer can't matter (no second-target hit possible):
// BLDCNT brightness up (effect 2, bias 8) / down (effect 3, bias 7) where the top layer is a first
// target (OBJ flag 0x80 -> bit 4, 3D flag 0x40 -> bit 0) and the window's colour-effect bit is set.
// The records and the alpha byte come out as ColorCompositeLine's (changed lanes alpha 0xFF).
// Inspired by DSperate's composite_line_fade.
inline void FadeLine(const u32* top, const u8* win, u32 bc, u32 evy, u32* out)
{
    const bool up = ((bc >> 6) & 3) == 2;
    const uint8x16_t bcLo = vdupq_n_u8((u8)bc), c80 = vdupq_n_u8(0x80), c40 = vdupq_n_u8(0x40);
    const uint8x16_t c10 = vdupq_n_u8(0x10), c01 = vdupq_n_u8(0x01), c20 = vdupq_n_u8(0x20), c3F = vdupq_n_u8(0x3F);
    const uint8x8_t vevy = vdup_n_u8((u8)evy);
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint8x16x4_t A = vld4q_u8(reinterpret_cast<const u8*>(top + i));
        const uint8x16_t f1 = A.val[3];
        const uint8x16_t flag1b = vbslq_u8(vtstq_u8(f1, c80), c10, vbslq_u8(vtstq_u8(f1, c40), c01, f1));
        const uint8x16_t fx = vandq_u8(vtstq_u8(flag1b, bcLo), vtstq_u8(vld1q_u8(win + i), c20));
        if (!vmaxvq_u8(fx))
        {
            for (u32 k = 0; k < 16; k += 4) vst1q_u32(out + i + k, vld1q_u32(top + i + k));
            continue;
        }
        uint8x16x4_t o;
        for (int c = 0; c < 3; c++)
        {
            const uint8x16_t a6 = vandq_u8(A.val[c], c3F);
            o.val[c] = vbslq_u8(fx, up ? Brighten16(a6, vevy, 8) : Darken16(a6, vevy, 7), A.val[c]);
        }
        o.val[3] = vbslq_u8(fx, vdupq_n_u8(0xFF), f1);
        vst4q_u8(reinterpret_cast<u8*>(out + i), o);
    }
}

// ---- sprite rows (LITEV_SOFT2D_OBJROW) ---------------------------------------------------------
// One sprite's decoded row (cnt texel indices in screen order, 0 = transparent) plotted into the
// OBJLine words, 4 at a time, with SoftRenderer2D::DrawSpritePixel<false>'s rule: an opaque pixel
// wins over a transparent one or one of a lower priority; a transparent pixel over a transparent
// one stamps its sprite/mosaic/priority bits. Inspired by DSperate's obj_plot16.
// opaque = 1<<18, prio = 3<<16, sprite = 1<<19, mosaic = 1<<20 (SoftRenderer2D OBJ_* bits)
inline u32 PlotObjRow(const u8* row, u32 cnt, u32 attr, u32* line)
{
    const uint32x4_t vattr = vdupq_n_u32(attr), vop = vdupq_n_u32(1u << 18), vpm = vdupq_n_u32(3u << 16);
    const uint32x4_t vprio = vdupq_n_u32(attr & (3u << 16));
    const uint32x4_t vkeep = vdupq_n_u32(~((1u << 20) | (3u << 16)));
    const uint32x4_t vstamp = vdupq_n_u32(attr & ((1u << 19) | (1u << 20) | (3u << 16)));
    u32 j = 0;
    for (; j + 4 <= cnt; j += 4)
    {
        u32 w; memcpy(&w, row + j, 4);
        const uint32x4_t c = vmovl_u16(vget_low_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(w)))));
        const uint32x4_t newop = vtstq_u32(c, c);
        const uint32x4_t old = vld1q_u32(line + j);
        const uint32x4_t oldop = vtstq_u32(old, vop);
        const uint32x4_t lt = vcltq_u32(vprio, vandq_u32(old, vpm));
        const uint32x4_t win = vandq_u32(newop, vorrq_u32(vmvnq_u32(oldop), lt));
        const uint32x4_t stamp = vbicq_u32(vmvnq_u32(newop), oldop);
        uint32x4_t r = vbslq_u32(stamp, vorrq_u32(vandq_u32(old, vkeep), vstamp), old);
        r = vbslq_u32(win, vorrq_u32(c, vattr), r);
        vst1q_u32(line + j, r);
    }
    return j;   // the caller plots the tail
}

// ---- output stage (LITEV_SOFT2D_OUTNEON) --------------------------------------------------------
// A regular-display line to the framebuffer in one pass: master brightness, then 6 -> 8 bit
// expansion to 0xAARRGGBB (SoftRenderer::ApplyMasterBrightness + ExpandColor).
inline void OutputLine(const u32* src, u16 reg, u32* dst)
{
    const u32 mode = reg >> 14;
    u32 factor = reg & 0x1F;
    if (factor > 16) factor = 16;
    const uint8x8_t vf = vdup_n_u8((u8)factor);
    const uint8x16_t v63 = vdupq_n_u8(63), vff = vdupq_n_u8(0xFF);
    const bool bright = mode == 1 && factor, dark = mode == 2 && factor;
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint8x16x4_t p = vld4q_u8(reinterpret_cast<const u8*>(src + i));
        uint8x16_t c[3];
        for (u32 k = 0; k < 3; ++k)
        {
            c[k] = vandq_u8(p.val[k], v63);
            if (bright) c[k] = Brighten16(c[k], vf, 0);
            else if (dark) c[k] = Darken16(c[k], vf, 15);
            c[k] = vorrq_u8(vshlq_n_u8(c[k], 2), vshrq_n_u8(c[k], 4));
        }
        const uint8x16x4_t rec = {{c[2], c[1], c[0], vff}};
        vst4q_u8(reinterpret_cast<u8*>(dst + i), rec);
    }
}

// Display capture, source A only: 18-bit records (alpha byte non-zero = opaque) -> BGR555 + bit 15.
inline void CaptureA15(const u32* src, u32 n, u16* dst)
{
    const uint8x16_t v1f = vdupq_n_u8(0x1F);
    for (u32 i = 0; i < n; i += 16)
    {
        const uint8x16x4_t p = vld4q_u8(reinterpret_cast<const u8*>(src + i));
        const uint8x16_t r = vandq_u8(vshrq_n_u8(p.val[0], 1), v1f);
        const uint8x16_t g = vandq_u8(vshrq_n_u8(p.val[1], 1), v1f);
        const uint8x16_t b = vandq_u8(vshrq_n_u8(p.val[2], 1), v1f);
        const uint8x16_t a = vtstq_u8(p.val[3], p.val[3]);
        auto half = [&](u32 o, uint8x8_t r8, uint8x8_t g8, uint8x8_t b8, uint8x8_t a8) {
            uint16x8_t w = vmovl_u8(r8);
            w = vorrq_u16(w, vshlq_n_u16(vmovl_u8(g8), 5));
            w = vorrq_u16(w, vshlq_n_u16(vmovl_u8(b8), 10));
            w = vorrq_u16(w, vshlq_n_u16(vmovl_u8(a8), 15));
            vst1q_u16(dst + i + o, w);
        };
        half(0, vget_low_u8(r), vget_low_u8(g), vget_low_u8(b), vget_low_u8(a));
        half(8, vget_high_u8(r), vget_high_u8(g), vget_high_u8(b), vget_high_u8(a));
    }
}

// Capture blend: A as above, B a BGR555 line (alpha bit 15; null = all transparent). Per channel
// ((ca*aa*eva)+(cb*ab*evb)+8)>>4 clamped to 31; alpha (eva?aa:0)|(evb?ab:0). eva/evb in 0..16.
inline void CaptureBlend(const u32* srca, const u16* srcb, u32 n, u32 eva, u32 evb, u16* dst)
{
    const uint8x16_t v1f = vdupq_n_u8(0x1F);
    const uint8x8_t va8 = vdup_n_u8((u8)eva), vb8 = vdup_n_u8((u8)evb);
    const uint8x16_t ea = vdupq_n_u8(eva ? 0xFF : 0), eb = vdupq_n_u8(evb ? 0xFF : 0);
    const uint16x8_t bias = vdupq_n_u16(8), v31 = vdupq_n_u16(31), topbit = vdupq_n_u16(0x8000);
    for (u32 i = 0; i < n; i += 16)
    {
        const uint8x16x4_t p = vld4q_u8(reinterpret_cast<const u8*>(srca + i));
        const uint8x16_t aam = vtstq_u8(p.val[3], p.val[3]);
        const uint8x16_t ra = vandq_u8(vandq_u8(vshrq_n_u8(p.val[0], 1), v1f), aam);
        const uint8x16_t ga = vandq_u8(vandq_u8(vshrq_n_u8(p.val[1], 1), v1f), aam);
        const uint8x16_t ba = vandq_u8(vandq_u8(vshrq_n_u8(p.val[2], 1), v1f), aam);
        const uint16x8_t w0 = srcb ? vld1q_u16(srcb + i) : vdupq_n_u16(0), w1 = srcb ? vld1q_u16(srcb + i + 8) : vdupq_n_u16(0);
        const uint8x16_t abm = vcombine_u8(vmovn_u16(vtstq_u16(w0, topbit)), vmovn_u16(vtstq_u16(w1, topbit)));
        auto bnarrow = [&](uint16x8_t c0, uint16x8_t c1) {
            return vandq_u8(vandq_u8(vcombine_u8(vmovn_u16(c0), vmovn_u16(c1)), v1f), abm);
        };
        const uint8x16_t rb = bnarrow(w0, w1);
        const uint8x16_t gb = bnarrow(vshrq_n_u16(w0, 5), vshrq_n_u16(w1, 5));
        const uint8x16_t bb = bnarrow(vshrq_n_u16(w0, 10), vshrq_n_u16(w1, 10));
        const uint8x16_t ad = vorrq_u8(vandq_u8(aam, ea), vandq_u8(abm, eb));
        auto half = [&](u32 o, auto lane) {
            auto blend1 = [&](uint8x16_t ca, uint8x16_t cb) {
                uint16x8_t s = vmlal_u8(vmull_u8(lane(ca), va8), lane(cb), vb8);
                return vminq_u16(vshrq_n_u16(vaddq_u16(s, bias), 4), v31);
            };
            uint16x8_t w = blend1(ra, rb);
            w = vorrq_u16(w, vshlq_n_u16(blend1(ga, gb), 5));
            w = vorrq_u16(w, vshlq_n_u16(blend1(ba, bb), 10));
            w = vorrq_u16(w, vshlq_n_u16(vmovl_u8(lane(ad)), 15));
            vst1q_u16(dst + i + o, w);
        };
        half(0, [](uint8x16_t v) { return vget_low_u8(v); });
        half(8, [](uint8x16_t v) { return vget_high_u8(v); });
    }
}
} // namespace S2DPlanes


}
