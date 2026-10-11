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
#include <cmath>
#include "Platform.h"
#include "NDS.h"
#include "Mic.h"
#include "DSi.h"
#include "DSi_I2S.h"
#include "SPU.h"
#include "LiteProfile.h"

#include "blip-buf/blip_buf.h"

#ifdef LITEV_SPU_MIX_NEON
#include <arm_neon.h>
#endif

#define INTERNAL_SAMPLE_RATE 16756991.f

// LITEV_SPU_INLINE: the per-sample decoders and FIFO reads are inlined into SPUChannel::Run, which
// calls them once per sample per channel (~0.3M calls/s with PW's ADPCM music). Same code.
#ifdef LITEV_SPU_INLINE
#define LITEV_SPU_INL __attribute__((always_inline)) inline
#else
#define LITEV_SPU_INL
#endif

#include <cstdlib>
#include <chrono>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;


// SPU TODO
// * capture addition modes, overflow bugs
// * channel hold


const s8 SPUChannel::ADPCMIndexTable[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

const u16 SPUChannel::ADPCMTable[89] =
{
    0x0007, 0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E,
    0x0010, 0x0011, 0x0013, 0x0015, 0x0017, 0x0019, 0x001C, 0x001F,
    0x0022, 0x0025, 0x0029, 0x002D, 0x0032, 0x0037, 0x003C, 0x0042,
    0x0049, 0x0050, 0x0058, 0x0061, 0x006B, 0x0076, 0x0082, 0x008F,
    0x009D, 0x00AD, 0x00BE, 0x00D1, 0x00E6, 0x00FD, 0x0117, 0x0133,
    0x0151, 0x0173, 0x0198, 0x01C1, 0x01EE, 0x0220, 0x0256, 0x0292,
    0x02D4, 0x031C, 0x036C, 0x03C3, 0x0424, 0x048E, 0x0502, 0x0583,
    0x0610, 0x06AB, 0x0756, 0x0812, 0x08E0, 0x09C3, 0x0ABD, 0x0BD0,
    0x0CFF, 0x0E4C, 0x0FBA, 0x114C, 0x1307, 0x14EE, 0x1706, 0x1954,
    0x1BDC, 0x1EA5, 0x21B6, 0x2515, 0x28CA, 0x2CDF, 0x315B, 0x364B,
    0x3BB9, 0x41B2, 0x4844, 0x4F7E, 0x5771, 0x602F, 0x69CE, 0x7462,
    0x7FFF
};

const s16 SPUChannel::PSGTable[8][8] =
{
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF}
};

template <typename T>
constexpr T ipow(T num, unsigned int pow)
{
    T product = 1;
    for (int i = 0; i < pow; ++i)
    {
        product *= num;
    }

    return product;
}

template <typename T>
constexpr T factorial(T num)
{
    T product = 1;
    for (T i = 1; i <= num; ++i)
    {
        product *= i;
    }

    return product;
}

// We can't use std::cos in constexpr functions until C++26,
// so we need to compute the cosine ourselves with the Taylor series.
// Code adapted from https://prosepoetrycode.potterpcs.net/2015/07/a-simple-constexpr-power-function-c/
template <int Iterations = 10>
constexpr double cosine (double theta)
{
    return (ipow(-1, Iterations) * ipow(theta, 2 * Iterations)) /
            static_cast<double>(factorial(2ull * Iterations))
        + cosine<Iterations-1>(theta);
}

template <>
constexpr double cosine<0> (double theta)
{
    return 1.0;
}

// generate interpolation tables
// values are 1:1:14 fixed-point
constexpr std::array<s16, 0x100> InterpCos = []() constexpr {
    std::array<s16, 0x100> interp {};

    for (int i = 0; i < 0x100; i++)
    {
        float ratio = (i * M_PI) / 255.0f;
        ratio = 1.0f - cosine(ratio);

        interp[i] = (s16)(ratio * 0x2000);
    }

    return interp;
}();

constexpr array2d<s16, 0x100, 4> InterpCubic = []() constexpr {
    array2d<s16, 0x100, 4> interp {};

    for (int i = 0; i < 0x100; i++)
    {
        s32 i1 = i << 6;
        s32 i2 = (i * i) >> 2;
        s32 i3 = (i * i * i) >> 10;

        interp[i][0] = -i3 + 2*i2 - i1;
        interp[i][1] = i3 - 2*i2 + 0x4000;
        interp[i][2] = -i3 + i2 + i1;
        interp[i][3] = i3 - i2;
    }

    return interp;
}();

const std::array<s16, 0x200> InterpSNESGauss = {
    0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
    0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x002, 0x002,
    0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005,
    0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A,
    0x00B, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010, 0x011, 0x011,
    0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018, 0x018, 0x019, 0x01A, 0x01B, 0x01B,
    0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021, 0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028,
    0x029, 0x02A, 0x02B, 0x02C, 0x02D, 0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038,
    0x03A, 0x03B, 0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C, 0x04D,
    0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F, 0x061, 0x063, 0x064, 0x066,
    0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076, 0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084,
    0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091, 0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8,
    0x0AB, 0x0AD, 0x0AF, 0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
    0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8, 0x0FB, 0x0FE, 0x101,
    0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122, 0x125, 0x129, 0x12C, 0x130, 0x133, 0x137,
    0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150, 0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172,
    0x176, 0x17A, 0x17D, 0x181, 0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2,
    0x1B7, 0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF, 0x1F3, 0x1F8,
    0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A, 0x22F, 0x233, 0x238, 0x23D, 0x241,
    0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267, 0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E,
    0x293, 0x298, 0x29D, 0x2A2, 0x2A6, 0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC,
    0x2E1, 0x2E6, 0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326, 0x32B,
    0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366, 0x36B, 0x370, 0x374, 0x379,
    0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4, 0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5,
    0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0, 0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C,
    0x410, 0x415, 0x419, 0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
    0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E, 0x481, 0x485, 0x488,
    0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9, 0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA,
    0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD, 0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3,
    0x4E5, 0x4E7, 0x4E9, 0x4EB, 0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500,
    0x502, 0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511, 0x511, 0x512,
    0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518, 0x518, 0x518, 0x518, 0x519, 0x519
};

SPU::SPU(melonDS::NDS& nds, AudioBitDepth bitdepth, AudioInterpolation interpolation, double outputSampleRate) :
    NDS(nds),
    Channels {
        SPUChannel(0, nds, interpolation),
        SPUChannel(1, nds, interpolation),
        SPUChannel(2, nds, interpolation),
        SPUChannel(3, nds, interpolation),
        SPUChannel(4, nds, interpolation),
        SPUChannel(5, nds, interpolation),
        SPUChannel(6, nds, interpolation),
        SPUChannel(7, nds, interpolation),
        SPUChannel(8, nds, interpolation),
        SPUChannel(9, nds, interpolation),
        SPUChannel(10, nds, interpolation),
        SPUChannel(11, nds, interpolation),
        SPUChannel(12, nds, interpolation),
        SPUChannel(13, nds, interpolation),
        SPUChannel(14, nds, interpolation),
        SPUChannel(15, nds, interpolation),
    },
    Capture {
        SPUCaptureUnit(0, nds),
        SPUCaptureUnit(1, nds),
    },
    AudioLock(Platform::Mutex_Create()),
    Degrade10Bit(bitdepth == AudioBitDepth::_10Bit || (nds.ConsoleType == 1 && bitdepth == AudioBitDepth::Auto)),
    OutputSampleRate(outputSampleRate),
    OutputBuffer(nullptr)
{
    NDS.RegisterEventFuncs(Event_SPU, this, {MakeEventThunk(SPU, Mix)});

    ApplyBias = true;
    Degrade10Bit = false;

    BlipLeft = blip_new(512);
    BlipRight = blip_new(512);

    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;

    SetSampleRate(AudioSampleRate::_32KHz);
}

SPU::~SPU()
{
    if (OutputBuffer != nullptr)
    {
        free(OutputBuffer);
        OutputBuffer = nullptr;
    }

    Platform::Mutex_Free(AudioLock);
    AudioLock = nullptr;
    blip_delete(BlipLeft);
    blip_delete(BlipRight);

    NDS.UnregisterEventFuncs(Event_SPU);
}

void SPU::Reset()
{
    InitOutput();

    Cnt = 0;
    MasterVolume = 0;
    Bias = 0;
    Mute = true;

    for (int i = 0; i < 16; i++)
        Channels[i].Reset();

    Capture[0].Reset();
    Capture[1].Reset();

    NDS.ScheduleEvent(Event_SPU, false, 1024, 0, 0);
}

void SPU::Stop()
{
    Platform::Mutex_Lock(AudioLock);
    memset(OutputBuffer, 0, 2*OutputBufferSize*2);

    blip_clear(BlipLeft);
    blip_clear(BlipRight);
    BlipTimer = 0;

    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::DoSavestate(Savestate* file)
{
    file->Section("SPU.");

    file->Var16(&Cnt);
    file->Var8(&MasterVolume);
    file->Var16(&Bias);

    file->VarArray(OutputLastSamples, sizeof(OutputLastSamples));

    file->Var32(&MixInterval);

    file->Bool32(&Mute);

#ifdef LITEV_SPU_SILENT_LAZY
    if (file->Saving) MaterializeAll();
#endif
#ifdef LITEV_SPU_FIFO_TRACK
    if (file->Saving) for (SPUChannel& ch : Channels) { ch.FIFOCatchUp(); if (ch.FIFOStale) ch.RefillFIFO(); }
#endif
    for (SPUChannel& channel : Channels)
        channel.DoSavestate(file);
#ifdef LITEV_SPU_SILENT_LAZY
    if (!file->Saving)
        for (SPUChannel& ch : Channels) ch.Stale = ch.Tracked = false;
#endif
#ifdef LITEV_SPU_FIFO_TRACK
    if (!file->Saving) for (SPUChannel& ch : Channels) ch.FIFOStale = 0, ch.FIFOOwed = 0;
#endif
#ifdef LITEV_SPU_ADPCM_MEMO
    if (!file->Saving) { Memos.clear(); MemoBytes = 0; MemoGen++; }
#endif

    for (SPUCaptureUnit& capture : Capture)
        capture.DoSavestate(file);
}


void SPU::SetPowerCnt(u32 val)
{
    Mute = !(val & (1<<0));
}


void SPU::SetSampleRate(AudioSampleRate rate)
{
    if (rate == AudioSampleRate::_47KHz)
    {
        MixInterval = 704;
    }
    else
    {
        MixInterval = 1024;
    }

    memset(OutputLastSamples, 0, sizeof(OutputLastSamples));
}


void SPU::SetInterpolation(AudioInterpolation type)
{
#ifdef LITEV_SPU_SILENT_LAZY
    MaterializeAll();   // (the replay assumes one interpolation setting since Start)
    for (SPUChannel& ch : Channels) ch.Tracked = false;
#endif
    for (SPUChannel& channel : Channels)
        channel.InterpType = type;
}

void SPU::SetBias(u16 bias)
{
    Bias = bias;
}

void SPU::SetApplyBias(bool enable)
{
    ApplyBias = enable;
}

void SPU::SetDegrade10Bit(bool enable)
{
    Degrade10Bit = enable;
}

void SPU::SetDegrade10Bit(AudioBitDepth depth)
{
    switch (depth)
    {
    case AudioBitDepth::Auto:
        Degrade10Bit = (NDS.ConsoleType == 0);
        break;
    case AudioBitDepth::_10Bit:
        Degrade10Bit = true;
        break;
    case AudioBitDepth::_16Bit:
        Degrade10Bit = false;
        break;
    }
}

SPUChannel::SPUChannel(u32 num, melonDS::NDS& nds, AudioInterpolation interpolation) :
    NDS(nds),
    Num(num),
    InterpType(interpolation)
{
}

void SPUChannel::Reset()
{
    KeyOn = false;

    SetCnt(0);
    SrcAddr = 0;
    TimerReload = 0;
    LoopPos = 0;
    Length = 0;

    Timer = 0;

    Pos = 0;
    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOReadOffset = 0;
    FIFOLevel = 0;
#ifdef LITEV_SPU_SILENT_LAZY
    Stale = Tracked = false; Restarts = 0; Steps = 0;
#endif
#ifdef LITEV_SPU_FIFO_TRACK
    FIFOStale = 0; FIFOOwed = 0;
#endif
}

void SPUChannel::DoSavestate(Savestate* file)
{
    file->Var32(&Cnt);
    file->Var32(&SrcAddr);
    file->Var16(&TimerReload);
    file->Var32(&LoopPos);
    file->Var32(&Length);

    file->Var8(&Volume);
    file->Var8(&VolumeShift);
    file->Var8(&Pan);

    file->Var8((u8*)&KeyOn);
    file->Var32(&Timer);
    file->Var32((u32*)&Pos);
    file->VarArray(PrevSample, sizeof(PrevSample));
    file->Var16((u16*)&CurSample);
    file->Var16(&NoiseVal);

    file->Var32((u32*)&ADPCMVal);
    file->Var32((u32*)&ADPCMIndex);
    file->Var32((u32*)&ADPCMValLoop);
    file->Var32((u32*)&ADPCMIndexLoop);
    file->Var8(&ADPCMCurByte);

    file->Var32(&FIFOReadPos);
    file->Var32(&FIFOWritePos);
    file->Var32(&FIFOReadOffset);
    file->Var32(&FIFOLevel);
    file->VarArray(FIFO, sizeof(FIFO));
}

void SPUChannel::FIFO_BufferData()
{
    u32 totallen = LoopPos + Length;

    if (FIFOReadOffset >= totallen)
    {
        u32 repeatmode = (Cnt >> 27) & 0x3;
        if      (repeatmode & 1) FIFOReadOffset = LoopPos;
        else if (repeatmode & 2) return; // one-shot sound, we're done
    }

    u32 burstlen = 16;
    if ((FIFOReadOffset + 16) > totallen)
        burstlen = totallen - FIFOReadOffset;

    // sound DMA can't read from the ARM7 BIOS
    if ((SrcAddr + FIFOReadOffset) >= 0x00004000)
    {
        for (u32 i = 0; i < burstlen; i += 4)
        {
#ifdef LITEV_SPU_FAST_FETCH
            // sample data is almost always in main RAM: read it directly (what ARM7Read32 does
            // for 0x02000000-0x02FFFFFF) instead of through the full ARM7 bus switch
            const u32 a = (SrcAddr + FIFOReadOffset) & ~0x3;
            if ((a & 0xFF000000) == 0x02000000)
                FIFO[FIFOWritePos] = *(u32*)&NDS.MainRAM[a & NDS.MainRAMMask];
            else
#endif
            FIFO[FIFOWritePos] = NDS.ARM7Read32(SrcAddr + FIFOReadOffset);
#ifdef LITEV_SPU_FIFO_TRACK
            FIFOSrc[FIFOWritePos] = FIFOReadOffset;
            FIFOStale &= ~(1 << FIFOWritePos);
#endif
            FIFOReadOffset += 4;
            FIFOWritePos++;
            FIFOWritePos &= 0x7;
        }
    }
    else
    {
        for (u32 i = 0; i < burstlen; i += 4)
        {
            FIFO[FIFOWritePos] = 0;
#ifdef LITEV_SPU_FIFO_TRACK
            FIFOSrc[FIFOWritePos] = FIFOReadOffset;
            FIFOStale &= ~(1 << FIFOWritePos);
#endif
            FIFOReadOffset += 4;
            FIFOWritePos++;
            FIFOWritePos &= 0x7;
        }
    }

    FIFOLevel += burstlen;
}

template<typename T>
LITEV_SPU_INL T SPUChannel::FIFO_ReadData()
{
    T ret = *(T*)&((u8*)FIFO)[FIFOReadPos];

    FIFOReadPos += sizeof(T);
    FIFOReadPos &= 0x1F;
    FIFOLevel -= sizeof(T);

    if (FIFOLevel <= 16)
        FIFO_BufferData();

    return ret;
}

void SPUChannel::Start()
{
    Timer = TimerReload;

    if (((Cnt >> 29) & 0x3) == 3)
        Pos = -1;
    else
        Pos = -3;

    NoiseVal = 0x7FFF;
    PrevSample[0] = 0;
    PrevSample[1] = 0;
    PrevSample[2] = 0;
    CurSample = 0;

    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOReadOffset = 0;
    FIFOLevel = 0;
#ifdef LITEV_SPU_SILENT_LAZY
    Stale = false; Tracked = true; Restarts = 0; Steps = 0;
#endif
#ifdef LITEV_SPU_FIFO_TRACK
    FIFOStale = 0; FIFOOwed = 0;
#endif

    // when starting a channel, buffer data
    if (((Cnt >> 29) & 0x3) != 3)
    {
        FIFO_BufferData();
        FIFO_BufferData();
    }
}

LITEV_SPU_INL void SPUChannel::NextSample_PCM8()
{
    Pos++;
    if (Pos < 0) return;
    if (Pos >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos;
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }

    s8 val = FIFO_ReadData<s8>();
    CurSample = val << 8;
}

LITEV_SPU_INL void SPUChannel::NextSample_PCM16()
{
    Pos++;
    if (Pos < 0) return;
    if ((Pos<<1) >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos>>1;
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }

    s16 val = FIFO_ReadData<s16>();
    CurSample = val;
}

#ifdef LITEV_SPU_ADPCM_TABLE
static const struct ADPCMTables
{
    u16 Diff[89][8];
    u8 Next[89][8];

    ADPCMTables()
    {
        for (int i = 0; i < 89; i++)
            for (int n = 0; n < 8; n++)
            {
                const u16 val = SPUChannel::ADPCMTable[i];
                u16 diff = val >> 3;
                if (n & 0x1) diff += (val >> 2);
                if (n & 0x2) diff += (val >> 1);
                if (n & 0x4) diff += val;
                Diff[i][n] = diff;
                const int next = i + SPUChannel::ADPCMIndexTable[n];
                Next[i][n] = next < 0 ? 0 : next > 88 ? 88 : next;

            }
    }
} ADPCMTabs;
#endif

LITEV_SPU_INL void SPUChannel::NextSample_ADPCM()
{
    Pos++;
    if (Pos < 8)
    {
        if (Pos == 0)
        {
            // setup ADPCM
            u32 header = FIFO_ReadData<u32>();
            ADPCMVal = (s32)(s16)(header & 0xFFFF);
            ADPCMIndex = (header >> 16) & 0x7F;
            if (ADPCMIndex > 88) ADPCMIndex = 88;

            ADPCMValLoop = ADPCMVal;
            ADPCMIndexLoop = ADPCMIndex;
        }

        return;
    }

    if ((Pos>>1) >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos<<1;
            ADPCMVal = ADPCMValLoop;
            ADPCMIndex = ADPCMIndexLoop;
            ADPCMCurByte = FIFO_ReadData<u8>();
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }
    else
    {
        if (!(Pos & 0x1))
            ADPCMCurByte = FIFO_ReadData<u8>();
        else
            ADPCMCurByte >>= 4;

#ifdef LITEV_SPU_ADPCM_TABLE
        // the step x nibble difference and the next step index from tables (the same values the
        // code below computes): its per-bit branches were unpredictable on the in-order A55
        // (ADPCM decode: ~5% of the emu thread with PW's ADPCM music)
        {
            const u32 nib = ADPCMCurByte & 0x7;
            const s32 diff = ADPCMTabs.Diff[ADPCMIndex][nib];
            if (ADPCMCurByte & 0x8)
            {
                ADPCMVal -= diff;
                if (ADPCMVal < -0x7FFF) ADPCMVal = -0x7FFF;
            }
            else
            {
                ADPCMVal += diff;
                if (ADPCMVal > 0x7FFF) ADPCMVal = 0x7FFF;
            }
            ADPCMIndex = ADPCMTabs.Next[ADPCMIndex][nib];
        }
#else
        u16 val = ADPCMTable[ADPCMIndex];
        u16 diff = val >> 3;
        if (ADPCMCurByte & 0x1) diff += (val >> 2);
        if (ADPCMCurByte & 0x2) diff += (val >> 1);
        if (ADPCMCurByte & 0x4) diff += val;

        if (ADPCMCurByte & 0x8)
        {
            ADPCMVal -= diff;
            if (ADPCMVal < -0x7FFF) ADPCMVal = -0x7FFF;
        }
        else
        {
            ADPCMVal += diff;
            if (ADPCMVal > 0x7FFF) ADPCMVal = 0x7FFF;
        }

        ADPCMIndex += ADPCMIndexTable[ADPCMCurByte & 0x7];
        if      (ADPCMIndex < 0)  ADPCMIndex = 0;
        else if (ADPCMIndex > 88) ADPCMIndex = 88;
#endif

        if (Pos == (LoopPos<<1))
        {
            ADPCMValLoop = ADPCMVal;
            ADPCMIndexLoop = ADPCMIndex;
        }
    }

    CurSample = ADPCMVal;
}

#ifdef LITEV_SPU_BENCH
static int BenchDiv = 0, BenchMemo = -1;
#endif
// debug.litev.<prop> on Android, <env> elsewhere; read once
[[maybe_unused]] static int SPUProp(const char* prop, const char* env, int def)
{
#if defined(__ANDROID__)
    char b[PROP_VALUE_MAX] = {0};
    (void)env;
    return __system_property_get(prop, b) > 0 ? atoi(b) : def;
#else
    (void)prop;
    const char* e = getenv(env);
    return e ? atoi(e) : def;
#endif
}

#ifdef LITEV_SPU_RATE_DIV
// Quality-for-speed (debug.litev.spudiv / LITEV_SPUDIV = 1, 2 or 4, default 1): mix at 32768/div Hz.
// Each channel still steps every sample of its own (exact Timer/Pos/end/busy at the batch end, which
// is all the ARM cores can see), but its output value, the pan mix and the blip delta are computed
// once per div ticks. Only with the channel-major batch (no sound capture, someone listening).
static u32 SPURateDiv(u32 setting)
{
    static const int forced = SPUProp("debug.litev.spudiv", "LITEV_SPUDIV", 0);   // 0 = the setting
    const u32 v = forced > 0 ? (u32)forced : setting;
#ifdef LITEV_SPU_BENCH
    if (BenchDiv) return BenchDiv;
#endif
    return ((v == 2 || v == 4) && (LITEV_SPU_BATCH_N) % v == 0) ? v : 1;
}
#endif

#ifdef LITEV_SPU_FAST_ADPCM
// Game-first ADPCM (debug.litev.spufast, default on): the same decode, loop and end handling as
// NextSample_ADPCM + Run<2>, but over a batch with the channel state in locals and the sample
// bytes read straight from main RAM (the hardware streams them through a 32-byte FIFO; reading at
// decode time only differs if the game rewrites a sample while it plays). Audible output and the
// channel's busy/end timing are otherwise the same. Main-RAM samples with linear/no interpolation.
static bool SPUFastOn()
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.spufast", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_SPUFAST");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on;
}

#ifdef LITEV_SPU_ADPCM_MEMO
// ADPCM decode memo (debug.litev.spumemo / LITEV_SPUMEMO, default on). A looping ADPCM sample decodes
// to the same states on every pass (the loop restarts from the state saved at the loop point), and
// the same instrument sample is keyed on again and again, so the first pass records each position's
// decoded state and later passes look it up instead of decoding. Every 16-byte block is checked
// against the bytes it was decoded from before it is replayed (a game rewriting a sample, e.g. a
// streamed ring buffer, decodes it afresh). Same output as decoding; only the decode work changes.
static constexpr u32 MemoMaxBytes = 0x10000;       // per sample
static constexpr size_t MemoCapBytes = 8 << 20;     // all of them (then start over)

SPUChannel::ADPCMMemo* SPU::GetMemo(u32 src, u32 loop, u32 total)
{
    auto& m = Memos[(u64)src | ((u64)loop << 27) | ((u64)total << 44)];
    if (!m)
    {
        if (MemoBytes + 9 * total > MemoCapBytes)
        {
            Memos.clear(); MemoBytes = 0; MemoGen++;
            return GetMemo(src, loop, total);
        }
        m = std::make_unique<SPUChannel::ADPCMMemo>();
        m->St.resize(2 * total); m->Raw.resize(total);
        m->Src = src; m->Loop = loop; m->Total = total;
        MemoBytes += 9 * total;
    }
    return m.get();
}

static bool SPUMemoOn()
{
    // default OFF: on the RG DS (in-order A55) the memo measured slower than decoding (overworld audio
    // share 9.98% -> 11.08%: its table misses the caches); debug.litev.spumemo=1 turns it on
    static const bool on = SPUProp("debug.litev.spumemo", "LITEV_SPUMEMO", 0) != 0;
#ifdef LITEV_SPU_BENCH
    if (BenchMemo >= 0) return BenchMemo;
#endif
    return on;
}

// can positions from pos on be replayed? okUntil = end of the checked block
bool SPUChannel::MemoCheck(ADPCMMemo& m, s32 pos, s32& okUntil)
{
    const u32 b0 = (u32)(pos >> 1) & ~15u;
    const u32 e = std::min(b0 + 16, (u32)(m.Hi + 1) >> 1);
    const u32 a = (m.Src + b0) & NDS.MainRAMMask;
    if (a + (e - b0) <= NDS.MainRAMMask + 1 && !memcmp(&NDS.MainRAM[a], &m.Raw[b0], e - b0))
    {
        okUntil = std::min((s32)(b0 + 16) << 1, m.Hi);
        return true;
    }
    m.Hi = std::max(8, (s32)b0 << 1);   // this block changed: decode (and record) it again
    return false;
}
#endif

bool SPUChannel::RunADPCMFast(u32 cycles, s32 (*dst)[16], int col, int n)
{
    if (!SPUFastOn() || (SrcAddr >> 24) != 0x02) return false;
    if (!(Cnt & (1u<<31)) || (Length + LoopPos) < 16)
    {
        for (int b = 0; b < n; b++) dst[b][col] = 0;
        return true;
    }
    if (KeyOn) { Start(); KeyOn = false; }
#ifdef LITEV_SPU_SILENT_LAZY
    Tracked = false;    // steps not counted here
#endif

    const u8* ram = NDS.MainRAM;
    const u32 mask = NDS.MainRAMMask;
    const u32 src = SrcAddr;
    const u32 total = LoopPos + Length;
    const u32 repeat = (Cnt >> 27) & 0x3;
    const bool interp = InterpType != AudioInterpolation::None;
    const u32 reload = TimerReload;
    u32 timer = Timer;
    s32 pos = Pos, val = ADPCMVal, idx = ADPCMIndex, valLoop = ADPCMValLoop, idxLoop = ADPCMIndexLoop;
    u32 curByte = ADPCMCurByte;
    u32 owed = 0;   // FIFO bytes this batch consumes (the exact path's FIFO_ReadData calls)
    s32 cur = CurSample, p0 = PrevSample[0], p1 = PrevSample[1], p2 = PrevSample[2];
    bool on = true;
#ifdef LITEV_SPU_ADPCM_MEMO
    ADPCMMemo* m = nullptr;
    if (SPUMemoOn() && total <= MemoMaxBytes)
    {
        m = Memo;
        if (!m || MemoGen != NDS.SPU.MemoGen || m->Src != src || m->Loop != LoopPos || m->Total != total)
        {
            Memo = m = NDS.SPU.GetMemo(src, LoopPos, total);
            MemoGen = NDS.SPU.MemoGen;
        }
    }
    s32 okUntil = 0;        // positions below this are checked and replayed from the memo
    bool stale = false;     // curByte not loaded (replayed positions don't read the sample)
#endif

#ifdef LITEV_SPU_ADPCM_MEMO
    // each output tick steps the channel kmin or kmin+1 samples (timer stays in [reload, 0x10000))
    const u32 period = 0x10000 - reload, kmin = cycles / period, rem = cycles - kmin * period;
    const s32 loop2 = (s32)(LoopPos << 1);
#endif
    for (int b = 0; b < n; b++)
    {
        if (!on) { dst[b][col] = 0; continue; }
#ifdef LITEV_SPU_ADPCM_MEMO
        // all of this tick's samples checked and in the memo: jump to the last one
        if (m && pos >= 8 && timer >= reload)
        {
            u32 t = timer + rem, k = kmin;
            if (t >= 0x10000) { t -= period; k++; }
            const s32 np = pos + (s32)k;
            if (np >= okUntil && np < m->Hi)   // check the blocks up to np
                for (s32 q = std::max(okUntil, pos + 1); q <= np && MemoCheck(*m, q, okUntil); q = okUntil) {}
            if (np < okUntil)
            {
                if (k)
                {
                    owed += (u32)((np >> 1) - (pos >> 1));   // the bytes of the even positions passed
                    if (pos < loop2 && np >= loop2) { valLoop = (s16)m->St[loop2]; idxLoop = m->St[loop2] >> 16; }
                    const u32 st = m->St[np];
                    val = (s16)st; idx = st >> 16;
                    if (interp) { p2 = p1; p1 = p0; p0 = (s16)m->St[np - 1]; }
                    cur = val; pos = np; stale = true;
                }
                timer = t;
                goto stepped;
            }
        }
#endif
        timer += cycles;
        while (timer >> 16)
        {
            timer = reload + (timer - 0x10000);
            if (interp) { p2 = p1; p1 = p0; p0 = cur; }
            pos++;
            if (pos < 8)
            {
                if (pos == 0)
                {
                    const u32 header = *(const u32*)&ram[(src & ~3u) & mask];
                    owed += 4;   // FIFO bytes consumed (booked by FIFOCatchUp if the exact path takes over)
                    val = (s32)(s16)(header & 0xFFFF);
                    idx = (header >> 16) & 0x7F;
                    if (idx > 88) idx = 88;
                    valLoop = val; idxLoop = idx;
#ifdef LITEV_SPU_ADPCM_MEMO
                    if (m && (m->Hi < 8 || memcmp(&m->Raw[0], &header, 4)))
                    {
                        memcpy(&m->Raw[0], &header, 4);
                        m->Hi = 8;
                    }
#endif
                }
                continue;
            }
            if ((u32)(pos >> 1) >= total)
            {
                if (repeat & 1)
                {
                    pos = LoopPos << 1;
                    val = valLoop; idx = idxLoop;
                    curByte = ram[(src + LoopPos) & mask];
                    owed++;
#ifdef LITEV_SPU_ADPCM_MEMO
                    okUntil = 0; stale = false;
#endif
                }
                else if (repeat & 2)
                {
                    cur = 0;
                    Cnt &= ~(1u<<31);
                    on = false;
                    break;
                }
            }
            else
            {
                owed += !(pos & 1);
#ifdef LITEV_SPU_ADPCM_MEMO
                if (pos < okUntil || (m && pos < m->Hi && MemoCheck(*m, pos, okUntil)))
                {
                    const u32 st = m->St[pos];
                    val = (s16)st; idx = st >> 16;
                    stale = true;
                }
                else
                {
                if (!(pos & 1)) curByte = ram[(src + (pos >> 1)) & mask];
                else if (stale) curByte = ram[(src + (pos >> 1)) & mask] >> 4;
                else            curByte >>= 4;
                stale = false;
#else
                if (!(pos & 1)) curByte = ram[(src + (pos >> 1)) & mask];
                else            curByte >>= 4;
#endif
                // (a branch-free signed 16-entry table measured slower on the RG DS: 4.5% -> 6.4%)
                const u32 nib = curByte & 0x7;
                const s32 diff = ADPCMTabs.Diff[idx][nib];
                if (curByte & 0x8) { val -= diff; if (val < -0x7FFF) val = -0x7FFF; }
                else               { val += diff; if (val > 0x7FFF)  val = 0x7FFF; }
                idx = ADPCMTabs.Next[idx][nib];
#ifdef LITEV_SPU_ADPCM_MEMO
                if (m && pos == m->Hi)
                {
                    if (!(pos & 1)) m->Raw[pos >> 1] = (u8)curByte;
                    m->St[pos] = (u16)val | ((u32)idx << 16);
                    m->Hi++;
                }
                }
#endif
                if (pos == (s32)(LoopPos << 1)) { valLoop = val; idxLoop = idx; }
            }
            cur = val;
        }
#ifdef LITEV_SPU_ADPCM_MEMO
    stepped:
#endif
        if (!on || Volume == 0) { dst[b][col] = 0; continue; }
        s32 v = cur;
        if (interp)
        {
            const s32 frac = (timer >> 8) & 0xFF;
            v = ((v * frac) + (p0 * (0xFF - frac))) >> 8;
        }
        v <<= VolumeShift;
        dst[b][col] = v * Volume;
    }

#ifdef LITEV_SPU_ADPCM_MEMO
    if (stale) curByte = ram[(src + (pos >> 1)) & mask] >> (4 * (pos & 1));
#endif
    FIFOOwed += owed;
    Timer = timer; Pos = pos; ADPCMVal = val; ADPCMIndex = idx;
    ADPCMValLoop = valLoop; ADPCMIndexLoop = idxLoop; ADPCMCurByte = (u8)curByte;
    CurSample = (s16)cur; PrevSample[0] = (s16)p0; PrevSample[1] = (s16)p1; PrevSample[2] = (s16)p2;
    return true;
}
#endif

LITEV_SPU_INL void SPUChannel::NextSample_PSG()
{
    Pos++;
    CurSample = PSGTable[(Cnt >> 24) & 0x7][Pos & 0x7];
}

LITEV_SPU_INL void SPUChannel::NextSample_Noise()
{
    if (NoiseVal & 0x1)
    {
        NoiseVal = (NoiseVal >> 1) ^ 0x6000;
        CurSample = -0x7FFF;
    }
    else
    {
        NoiseVal >>= 1;
        CurSample = 0x7FFF;
    }
}

template<u32 type>
s32 SPUChannel::Run(u32 cycles, bool out)
{
    if (!(Cnt & (1<<31))) return 0;

    if ((type < 3) && ((Length+LoopPos) < 16)) return 0;

    if (KeyOn)
    {
        Start();
        KeyOn = false;
    }

    // 1 sample = 512 cycles at 16MHz
    // (or 352 cycles at 47KHz)
    Timer += cycles;

    while (Timer >> 16)
    {
        Timer = TimerReload + (Timer - 0x10000);

        // for optional interpolation: save previous samples
        // the interpolated audio will be delayed by a couple samples,
        // but it's easier to deal with this way
        if ((type < 3) && (InterpType != AudioInterpolation::None))
        {
            PrevSample[2] = PrevSample[1];
            PrevSample[1] = PrevSample[0];
            PrevSample[0] = CurSample;
        }

#ifdef LITEV_SPU_SILENT_LAZY
        const s32 oldPos = Pos;
#endif
        switch (type)
        {
        case 0: NextSample_PCM8(); break;
        case 1: NextSample_PCM16(); break;
        case 2: NextSample_ADPCM(); break;
        case 3: NextSample_PSG(); break;
        case 4: NextSample_Noise(); break;
        }
#ifdef LITEV_SPU_SILENT_LAZY
        if (type < 3) CountStep(oldPos);
#endif

        if (!(Cnt & (1<<31))) break;
    }

    if (!out) return 0;

    // Volume-0 skip: the channel already decoded (position/finish advanced above), but its output is
    // CurSample*Volume = 0, so the interpolation below is wasted work. Return 0 now. Bit-exact (the
    // original returns 0 too) -> no audio change, MP-safe.
    if (Volume == 0) return 0;

    s32 val = (s32)CurSample;

    // interpolation (emulation improvement, not a hardware feature)
#ifdef LITEV_SPU_FAST_INTERP
    // liteDS-v2 (M4): replace the cubic/cosine/Gaussian interpolation with a
    // cheap linear blend using the raw fractional timer position. Trades a
    // divide + table lookups for a single mul/add pair. Any InterpType other
    // than None collapses to this path.
    if ((type < 3) && (InterpType != AudioInterpolation::None))
    {
        s32 frac = (Timer >> 8) & 0xFF;
        val = ((val * frac) + ((s32)PrevSample[0] * (0xFF - frac))) >> 8;
    }
#else
    if ((type < 3) && (InterpType != AudioInterpolation::None))
    {
        s32 samplepos = ((Timer - TimerReload) * 0x100) / (0x10000 - TimerReload);
        if (samplepos > 0xFF) samplepos = 0xFF;

        switch (InterpType)
        {
        case AudioInterpolation::Linear:
            val = ((val           * samplepos) +
                   (PrevSample[0] * (0xFF-samplepos))) >> 8;
            break;

        case AudioInterpolation::Cosine:
            val = ((val           * InterpCos[samplepos]) +
                   (PrevSample[0] * InterpCos[0xFF-samplepos])) >> 14;
            break;

        case AudioInterpolation::Cubic:
            val = ((PrevSample[2] * InterpCubic[samplepos][0]) +
                   (PrevSample[1] * InterpCubic[samplepos][1]) +
                   (PrevSample[0] * InterpCubic[samplepos][2]) +
                   (val           * InterpCubic[samplepos][3])) >> 14;
            break;

        case AudioInterpolation::SNESGaussian: {
                // Avoid clipping (from fullsnes)
#define CLAMP(s) (std::clamp((s) >> 1, -0x3FFA, 0x3FF8))
                s32 out =    (InterpSNESGauss[0x0FF - samplepos] * CLAMP(PrevSample[2]) >> 10);
                out = out + ((InterpSNESGauss[0x1FF - samplepos] * CLAMP(PrevSample[1])) >> 10);
                out = out + ((InterpSNESGauss[0x100 + samplepos] * CLAMP(PrevSample[0])) >> 10);
                out = out + ((InterpSNESGauss[0x000 + samplepos] * CLAMP(val)) >> 10);
                val = std::clamp(out, -0x8000, 0x7FFF);
#undef CLAMP
                break;
            }

        default:
            break;
        }
    }
#endif // LITEV_SPU_FAST_INTERP

    val <<= VolumeShift;
    val *= Volume;
    return val;
}

#ifdef LITEV_SPU_FIFO_TRACK
// FIFO_BufferData without reading the sample: the same offsets, slots and level
void SPUChannel::FIFO_BufferTiming()
{
    const u32 totallen = LoopPos + Length;
    if (FIFOReadOffset >= totallen)
    {
        const u32 repeatmode = (Cnt >> 27) & 0x3;
        if      (repeatmode & 1) FIFOReadOffset = LoopPos;
        else if (repeatmode & 2) return;
    }
    u32 burstlen = 16;
    if ((FIFOReadOffset + 16) > totallen)
        burstlen = totallen - FIFOReadOffset;
    for (u32 i = 0; i < burstlen; i += 4)
    {
        FIFOSrc[FIFOWritePos] = FIFOReadOffset;
        FIFOStale |= 1 << FIFOWritePos;
        FIFOReadOffset += 4;
        FIFOWritePos = (FIFOWritePos + 1) & 0x7;
    }
    FIFOLevel += burstlen;
}

void SPUChannel::RefillFIFO()
{
    for (int i = 0; i < 8; i++)
        if (FIFOStale & (1 << i))
            FIFO[i] = *(u32*)&NDS.MainRAM[((SrcAddr + FIFOSrc[i]) & ~3u) & NDS.MainRAMMask];
    FIFOStale = 0;
}
#endif

#ifdef LITEV_SPU_SILENT_LAZY
// Run<type>(cycles, false) n times (PCM8/PCM16/ADPCM, sample in main RAM, looped or one-shot),
// stepping what the ARM cores can see: Timer, Pos, loop, end (busy bit), FIFO bookkeeping.
// The decode (and the interpolation history) is left stale; Materialize rebuilds it.
template<u32 type>
void SPUChannel::RunTiming(u32 cycles, u32 n)
{
    if (!(Cnt & (1u<<31))) return;
    if ((Length + LoopPos) < 16) return;
    if (KeyOn)
    {
        // Start() without the sample reads
        Timer = TimerReload;
        Pos = -3;
        NoiseVal = 0x7FFF;
        PrevSample[0] = PrevSample[1] = PrevSample[2] = 0;
        CurSample = 0;
        FIFOReadPos = FIFOWritePos = FIFOReadOffset = FIFOLevel = 0;
        FIFOStale = 0; FIFOOwed = 0;
        FIFO_BufferTiming();
        FIFO_BufferTiming();
        Tracked = true; Restarts = 0; Steps = 0;
        KeyOn = false;
    }
    Stale = true;
    const u32 total = LoopPos + Length, repeat = (Cnt >> 27) & 0x3;
    u32 timer = Timer;
    for (u32 b = 0; b < n; b++)
    {
        timer += cycles;
        while (timer >> 16)
        {
            timer = TimerReload + (timer - 0x10000);
            const s32 oldPos = Pos++;
            bool ended = false;
            if (type == 0)          // NextSample_PCM8
            {
                if (Pos >= 0)
                {
                    if ((u32)Pos >= total)
                    {
                        if (repeat & 1) Pos = LoopPos;
                        else if (repeat & 2) ended = true;
                    }
                    if (!ended) FIFO_Skip(1);
                }
            }
            else if (type == 1)     // NextSample_PCM16
            {
                if (Pos >= 0)
                {
                    if ((u32)(Pos << 1) >= total)
                    {
                        if (repeat & 1) Pos = LoopPos >> 1;
                        else if (repeat & 2) ended = true;
                    }
                    if (!ended) FIFO_Skip(2);
                }
            }
            else                    // NextSample_ADPCM
            {
                if (Pos < 8)
                {
                    if (Pos == 0) FIFO_Skip(4);
                }
                else if ((u32)(Pos >> 1) >= total)
                {
                    if (repeat & 1) { Pos = LoopPos << 1; FIFO_Skip(1); }
                    else if (repeat & 2) ended = true;
                }
                else if (!(Pos & 0x1)) FIFO_Skip(1);
            }
            CountStep(oldPos);
            if (ended)
            {
                CurSample = 0;
                Cnt &= ~(1u<<31);
                Timer = timer;
                return;
            }
        }
    }
    Timer = timer;
}

// Rebuild what RunTiming left stale: the FIFO words (read now from where they were buffered from)
// and, for a playing channel, the decode state, by replaying the sample's data path from Start on
// a scratch channel. Every loop pass from the second on decodes the same values (the loop restores
// the state saved in the first), so at most 6 restarts are replayed. Same values as stepping them
// in time unless the game rewrote the sample meanwhile (as with the FIFO's own read-ahead).
void SPUChannel::Materialize()
{
    if (!Stale) return;
    Stale = false;
    RefillFIFO();
    if (KeyOn || !(Cnt & (1u<<31))) return;     // stopped, or restarting: the rest is reset first

    SPUChannel r(Num, NDS, InterpType);
    r.Cnt = Cnt; r.SrcAddr = SrcAddr; r.TimerReload = TimerReload; r.LoopPos = LoopPos; r.Length = Length;
    r.ADPCMVal = ADPCMVal; r.ADPCMIndex = ADPCMIndex; r.ADPCMValLoop = ADPCMValLoop;
    r.ADPCMIndexLoop = ADPCMIndexLoop; r.ADPCMCurByte = ADPCMCurByte;
    r.Start();
    const u32 type = (Cnt >> 29) & 0x3;
    const bool interp = InterpType != AudioInterpolation::None;
    auto step = [&]() -> u32 {
        if (interp) { r.PrevSample[2] = r.PrevSample[1]; r.PrevSample[1] = r.PrevSample[0]; r.PrevSample[0] = r.CurSample; }
        const s32 old = r.Pos;
        if (type == 0) r.NextSample_PCM8();
        else if (type == 1) r.NextSample_PCM16();
        else r.NextSample_ADPCM();
        return r.Pos != old + 1;
    };
    for (u32 k = 0; k < Restarts; ) k += step();
    for (u32 k = 0; k < Steps; k++) step();
    if (r.Pos != Pos || !(r.Cnt & (1u<<31)))
        Log(LogLevel::Error, "SPU lazy: channel %u replay at pos %d, channel at %d\n", Num, r.Pos, Pos);
    CurSample = r.CurSample;
    PrevSample[0] = r.PrevSample[0]; PrevSample[1] = r.PrevSample[1]; PrevSample[2] = r.PrevSample[2];
    ADPCMVal = r.ADPCMVal; ADPCMIndex = r.ADPCMIndex;
    ADPCMValLoop = r.ADPCMValLoop; ADPCMIndexLoop = r.ADPCMIndexLoop; ADPCMCurByte = r.ADPCMCurByte;
}
#endif

void SPUChannel::PanOutput(s32 in, s32& left, s32& right)
{
    left += ((s64)in * (128-Pan)) >> 10;
    right += ((s64)in * Pan) >> 10;
}


SPUCaptureUnit::SPUCaptureUnit(u32 num, melonDS::NDS& nds) : NDS(nds), Num(num)
{
}

void SPUCaptureUnit::Reset()
{
    SetCnt(0);
    DstAddr = 0;
    TimerReload = 0;
    Length = 0;

    Timer = 0;

    Pos = 0;
    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOWriteOffset = 0;
    FIFOLevel = 0;
}

void SPUCaptureUnit::DoSavestate(Savestate* file)
{
    file->Var8(&Cnt);
    file->Var32(&DstAddr);
    file->Var16(&TimerReload);
    file->Var32(&Length);

    file->Var32(&Timer);
    file->Var32((u32*)&Pos);

    file->Var32(&FIFOReadPos);
    file->Var32(&FIFOWritePos);
    file->Var32(&FIFOWriteOffset);
    file->Var32(&FIFOLevel);
    file->VarArray(FIFO, 4*4);
}

void SPUCaptureUnit::FIFO_FlushData()
{
    for (u32 i = 0; i < 4; i++)
    {
        NDS.ARM7Write32(DstAddr + FIFOWriteOffset, FIFO[FIFOReadPos]);
        // Calls the NDS or DSi version, depending on the class

        FIFOReadPos++;
        FIFOReadPos &= 0x3;
        FIFOLevel -= 4;

        FIFOWriteOffset += 4;
        if (FIFOWriteOffset >= Length)
        {
            FIFOWriteOffset = 0;
            break;
        }
    }
}

template<typename T>
void SPUCaptureUnit::FIFO_WriteData(T val)
{
    *(T*)&((u8*)FIFO)[FIFOWritePos] = val;

    FIFOWritePos += sizeof(T);
    FIFOWritePos &= 0xF;
    FIFOLevel += sizeof(T);

    if (FIFOLevel >= 16)
        FIFO_FlushData();
}

void SPUCaptureUnit::Run(u32 cycles, s32 sample)
{
    Timer += cycles;

    if (Cnt & 0x08)
    {
        while (Timer >> 16)
        {
            Timer = TimerReload + (Timer - 0x10000);

            FIFO_WriteData<s8>((s8)(sample >> 8));
            Pos++;
            if (Pos >= Length)
            {
                if (FIFOLevel >= 4)
                    FIFO_FlushData();

                if (Cnt & 0x04)
                {
                    Cnt &= 0x7F;
                    return;
                }
                else
                    Pos = 0;
            }
        }
    }
    else
    {
        while (Timer >> 16)
        {
            Timer = TimerReload + (Timer - 0x10000);

            FIFO_WriteData<s16>((s16)sample);
            Pos += 2;
            if (Pos >= Length)
            {
                if (FIFOLevel >= 4)
                    FIFO_FlushData();

                if (Cnt & 0x04)
                {
                    Cnt &= 0x7F;
                    return;
                }
                else
                    Pos = 0;
            }
        }
    }
}



#ifdef LITEV_SPU_BENCH
// Mac measurement only (-DLITEV_SPU_BENCH, LITEV_SPU_BENCH=1): before each real batch, run it once per
// option set on a copy of the channel/mic/output state and time each, interleaved so the busy Mac's
// noise hits all of them alike. Prints the totals at exit.
struct SPUBenchVariant { const char* name; u32 div; int memo; blip_t* l; blip_t* r; int timer; s16 last[2]; double ns; };
static SPUBenchVariant BenchV[] = {
    {"current", 1, 0}, {"memo", 1, 1}, {"div2", 2, 0}, {"memo+div2", 2, 1}, {"memo+div4", 4, 1}, {"div4", 4, 0},
};
static constexpr int BenchN = sizeof(BenchV) / sizeof(BenchV[0]);
static u64 BenchCalls, BenchKept;
static struct BenchPrint { ~BenchPrint() {
    if (!BenchKept) return;
    fprintf(stderr, "SPU bench: %llu batches, %llu kept\n", (unsigned long long)BenchCalls, (unsigned long long)BenchKept);
    for (auto& v : BenchV)
        fprintf(stderr, "  %-10s %8.0f ns/batch  %5.1f%%\n", v.name, v.ns / BenchKept, 100.0 * v.ns / BenchV[0].ns);
} } BenchPrinter;

void SPU::Bench(u32 spucycles)
{
    static const bool on = getenv("LITEV_SPU_BENCH") != nullptr;
    if (!on || !(Cnt & (1<<15)) || ((Capture[0].Cnt | Capture[1].Cnt) & (1<<7)) || LITEV_HEADLESS(Silent)) return;
    static std::vector<u8> chs(sizeof(Channels)), mic(sizeof(NDS.Mic));
    static std::vector<s16> out(2 * OutputBufferSize);
    memcpy(chs.data(), (void*)&Channels, sizeof(Channels));
    memcpy(mic.data(), (void*)&NDS.Mic, sizeof(NDS.Mic));
    blip_t* const rl = BlipLeft; blip_t* const rr = BlipRight; s16* const rout = OutputBuffer;
    const int rtimer = BlipTimer; const s16 rlast0 = OutputLastSamples[0], rlast1 = OutputLastSamples[1];
    const u32 rrd = OutputBufferReadPos, rwr = OutputBufferWritePos;
    OutputBuffer = out.data();
    double t[BenchN];
    for (int k = 0; k < BenchN; k++)
    {
        SPUBenchVariant& v = BenchV[(k + BenchCalls) % BenchN];
        if (!v.l) { v.l = blip_new(512); v.r = blip_new(512);
                    blip_set_rates(v.l, INTERNAL_SAMPLE_RATE, OutputSampleRate); blip_set_rates(v.r, INTERNAL_SAMPLE_RATE, OutputSampleRate); }
        memcpy((void*)&Channels, chs.data(), sizeof(Channels));
        memcpy((void*)&NDS.Mic, mic.data(), sizeof(NDS.Mic));
        BlipLeft = v.l; BlipRight = v.r; BlipTimer = v.timer; OutputLastSamples[0] = v.last[0]; OutputLastSamples[1] = v.last[1];
        BenchDiv = v.div; BenchMemo = v.memo;
        const auto t0 = std::chrono::steady_clock::now();
        MixSamples(spucycles);
        t[(k + BenchCalls) % BenchN] = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count();
        v.timer = BlipTimer; v.last[0] = OutputLastSamples[0]; v.last[1] = OutputLastSamples[1];
    }
    BenchDiv = 0; BenchMemo = -1;
    memcpy((void*)&Channels, chs.data(), sizeof(Channels));
    memcpy((void*)&NDS.Mic, mic.data(), sizeof(NDS.Mic));
    BlipLeft = rl; BlipRight = rr; OutputBuffer = rout; BlipTimer = rtimer;
    OutputLastSamples[0] = rlast0; OutputLastSamples[1] = rlast1; OutputBufferReadPos = rrd; OutputBufferWritePos = rwr;
    BenchCalls++;
    double lo = t[0], hi = t[0];
    for (double x : t) { lo = std::min(lo, x); hi = std::max(hi, x); }
    if (hi > 8 * lo) return;   // preempted
    BenchKept++;
    for (int k = 0; k < BenchN; k++) BenchV[k].ns += t[k];
}
#endif

void SPU::Mix(u32 spucycles)
{
#ifdef LITEV_SPU_BENCH
    Bench(spucycles);
#endif
    MixSamples(spucycles);
#ifdef LITEV_SPU_BATCH
    NDS.ScheduleEvent(Event_SPU, true, MixInterval * (LITEV_SPU_BATCH_N), 0, MixInterval >> 1);
#else
    NDS.ScheduleEvent(Event_SPU, true, MixInterval, 0, MixInterval >> 1);
#endif
}

#if !defined(__ANDROID__) && defined(LITEV_SPU_FIFO_TRACK)
// Mac check (LITEV_SPULAZY_VERIFY=K): run an exact copy of every channel beside the lazy one;
// compare what the ARM cores see after every batch, and every K batches rebuild the stale values
// and compare those too. Prints mismatches to stderr.
struct SPULazyVerify
{
    std::vector<SPUChannel> Shadow;
    u64 Batches = 0, Rebuilds = 0, TimingBad = 0, DataBad = 0;
    bool Synced = false;
};
static void CopyChannel(SPUChannel& d, const SPUChannel& o)
{
    d.Cnt = o.Cnt; d.SrcAddr = o.SrcAddr; d.TimerReload = o.TimerReload; d.LoopPos = o.LoopPos; d.Length = o.Length;
    d.Volume = o.Volume; d.VolumeShift = o.VolumeShift; d.Pan = o.Pan; d.KeyOn = o.KeyOn; d.Timer = o.Timer; d.Pos = o.Pos;
    memcpy(d.PrevSample, o.PrevSample, sizeof(d.PrevSample)); d.CurSample = o.CurSample; d.NoiseVal = o.NoiseVal;
    d.ADPCMVal = o.ADPCMVal; d.ADPCMIndex = o.ADPCMIndex; d.ADPCMValLoop = o.ADPCMValLoop; d.ADPCMIndexLoop = o.ADPCMIndexLoop;
    d.ADPCMCurByte = o.ADPCMCurByte; memcpy(d.FIFO, o.FIFO, sizeof(d.FIFO)); d.FIFOReadPos = o.FIFOReadPos;
    d.FIFOWritePos = o.FIFOWritePos; d.FIFOReadOffset = o.FIFOReadOffset; d.FIFOLevel = o.FIFOLevel; d.InterpType = o.InterpType;
}
#endif
#if !defined(__ANDROID__) && defined(LITEV_SPU_FAST_ADPCM)
// Mac check (LITEV_SPUFAST_VERIFY=K): an exact copy of every channel runs beside the fast ADPCM
// batch; after each batch the positions/FIFO bookkeeping must match, and every K batches the FIFO
// words are refilled (as when sound capture switches to the exact path) and the decode compared.
static void SPUFastVerify(SPU& spu, melonDS::NDS& nds, std::array<SPUChannel, 16>& chs, bool before, u32 cycles, u32 n)
{
    static const int every = getenv("LITEV_SPUFAST_VERIFY") ? atoi(getenv("LITEV_SPUFAST_VERIFY")) : 0;
    if (every <= 0) return;
    static thread_local std::unordered_map<SPU*, std::unique_ptr<SPULazyVerify>> vs;
    auto& vp = vs[&spu];
    if (!vp) { vp = std::make_unique<SPULazyVerify>(); vp->Shadow.reserve(16); for (int i = 0; i < 16; i++) vp->Shadow.emplace_back(i, nds, chs[i].InterpType); }
    SPULazyVerify& v = *vp;
    if (before)
    {
        for (int i = 0; i < 16; i++)
        {
            SPUChannel& sh = v.Shadow[i]; SPUChannel& ch = chs[i];
            if (!v.Synced) { if (ch.FIFOStale) ch.RefillFIFO(); CopyChannel(sh, ch); continue; }
            sh.Cnt = ch.Cnt; sh.SrcAddr = ch.SrcAddr; sh.TimerReload = ch.TimerReload; sh.LoopPos = ch.LoopPos;
            sh.Length = ch.Length; sh.Volume = ch.Volume; sh.VolumeShift = ch.VolumeShift; sh.Pan = ch.Pan; sh.KeyOn = ch.KeyOn;
        }
        v.Synced = true;
        return;
    }
    for (SPUChannel& sh : v.Shadow) for (u32 b = 0; b < n; b++) sh.DoRun(cycles, true);
    for (SPUChannel& ch : chs) ch.FIFOCatchUp();   // (what leaving the fast path does)
    v.Batches++;
    const bool refill = (v.Batches % every) == 0;
    for (int i = 0; i < 16; i++)
    {
        SPUChannel& sh = v.Shadow[i]; SPUChannel& ch = chs[i];
        bool bad = sh.Cnt != ch.Cnt || sh.Pos != ch.Pos || sh.Timer != ch.Timer || sh.FIFOReadPos != ch.FIFOReadPos
                   || sh.FIFOWritePos != ch.FIFOWritePos || sh.FIFOReadOffset != ch.FIFOReadOffset || sh.FIFOLevel != ch.FIFOLevel;
        if (!bad && refill && (ch.Cnt & (1u<<31)) && ((ch.Cnt >> 29) & 3) == 2 && ch.Pos >= 8)
        {
            if (ch.FIFOStale) { ch.RefillFIFO(); v.Rebuilds++; }
            bad = memcmp(sh.FIFO, ch.FIFO, sizeof(sh.FIFO)) || sh.ADPCMVal != ch.ADPCMVal || sh.ADPCMIndex != ch.ADPCMIndex
                  || sh.ADPCMValLoop != ch.ADPCMValLoop || sh.ADPCMIndexLoop != ch.ADPCMIndexLoop || sh.CurSample != ch.CurSample;
        }
        if (bad)
        {
            if (v.TimingBad++ < 20)
                fprintf(stderr, "SPUFAST MISMATCH ch%d batch %llu fmt %u: pos %d/%d timer %x/%x fifo %u,%u,%u,%u/%u,%u,%u,%u val %d/%d idx %d/%d fifo0 %08x/%08x\n", i,
                        (unsigned long long)v.Batches, (ch.Cnt >> 29) & 3, sh.Pos, ch.Pos, sh.Timer, ch.Timer, sh.FIFOReadPos, sh.FIFOWritePos, sh.FIFOReadOffset,
                        sh.FIFOLevel, ch.FIFOReadPos, ch.FIFOWritePos, ch.FIFOReadOffset, ch.FIFOLevel, sh.ADPCMVal, ch.ADPCMVal, sh.ADPCMIndex, ch.ADPCMIndex, sh.FIFO[0], ch.FIFO[0]);
            if (ch.FIFOStale) ch.RefillFIFO();
            CopyChannel(sh, ch);
        }
    }
    if ((v.Batches % 20000) == 0)
        fprintf(stderr, "SPUFAST %p: %llu batches, %llu refills checked, mismatches %llu\n", (void*)&spu, (unsigned long long)v.Batches,
                (unsigned long long)v.Rebuilds, (unsigned long long)v.TimingBad);
}
#endif


#ifdef LITEV_SPU_SILENT_LAZY
// debug.litev.spulazy / LITEV_SPULAZY (default on; 0 = step a silent console's channels exactly)
static bool SPULazyOn()
{
    static const bool on = SPUProp("debug.litev.spulazy", "LITEV_SPULAZY", 1) != 0;
    return on;
}


void SPU::RunQuiet(u32 cycles, u32 n)
{
#ifndef __ANDROID__
    static const int verifyEvery = getenv("LITEV_SPULAZY_VERIFY") ? atoi(getenv("LITEV_SPULAZY_VERIFY")) : 0;
    static thread_local std::unordered_map<SPU*, std::unique_ptr<SPULazyVerify>> verifies;
    SPULazyVerify* v = nullptr;
    if (verifyEvery > 0)
    {
        auto& vp = verifies[this];
        if (!vp) vp = std::make_unique<SPULazyVerify>();
        v = vp.get();
        if (v->Shadow.empty())
        {
            if (const char* it = getenv("LITEV_SPULAZY_INTERP")) SetInterpolation((AudioInterpolation)atoi(it));
            v->Shadow.reserve(16);
            for (int i = 0; i < 16; i++) v->Shadow.emplace_back(i, NDS, Channels[i].InterpType);
        }
        for (int i = 0; i < 16; i++)
        {
            SPUChannel& sh = v->Shadow[i]; const SPUChannel& ch = Channels[i];
            if (!v->Synced) { MaterializeAll(); CopyChannel(sh, ch); continue; }
            // registers the guest may have written since the last batch
            sh.Cnt = ch.Cnt; sh.SrcAddr = ch.SrcAddr; sh.TimerReload = ch.TimerReload; sh.LoopPos = ch.LoopPos;
            sh.Length = ch.Length; sh.Volume = ch.Volume; sh.VolumeShift = ch.VolumeShift; sh.Pan = ch.Pan; sh.KeyOn = ch.KeyOn;
        }
        v->Synced = true;
        for (SPUChannel& sh : v->Shadow)
            for (u32 b = 0; b < n; b++) sh.DoRun(cycles, false);
    }
#endif
    for (SPUChannel& ch : Channels)
    {
        if (!(ch.Cnt & (1u<<31))) continue;
        if (ch.LazyOK()) { ch.DoRunTiming(cycles, n); AnyStale = true; }
        else for (u32 b = 0; b < n; b++) ch.DoRun(cycles, false);
    }
    for (u32 b = 0; b < n; b++) NDS.Mic.Advance(cycles << 1);
#ifndef __ANDROID__
    if (v)
    {
        v->Batches++;
        const bool rebuild = (v->Batches % verifyEvery) == 0;
        if (rebuild) { MaterializeAll(); v->Rebuilds++; }
        for (int i = 0; i < 16; i++)
        {
            const SPUChannel& sh = v->Shadow[i]; const SPUChannel& ch = Channels[i];
            if (sh.Cnt != ch.Cnt || sh.Pos != ch.Pos || sh.Timer != ch.Timer || sh.KeyOn != ch.KeyOn || sh.FIFOReadPos != ch.FIFOReadPos
                || sh.FIFOWritePos != ch.FIFOWritePos || sh.FIFOReadOffset != ch.FIFOReadOffset || sh.FIFOLevel != ch.FIFOLevel)
            {
                if (v->TimingBad++ < 20)
                    fprintf(stderr, "SPULAZY TIMING %p ch%d batch %llu: cnt %08x/%08x pos %d/%d timer %x/%x fifo %u,%u,%u,%u/%u,%u,%u,%u\n", (void*)this, i,
                            (unsigned long long)v->Batches, sh.Cnt, ch.Cnt, sh.Pos, ch.Pos, sh.Timer, ch.Timer, sh.FIFOReadPos, sh.FIFOWritePos,
                            sh.FIFOReadOffset, sh.FIFOLevel, ch.FIFOReadPos, ch.FIFOWritePos, ch.FIFOReadOffset, ch.FIFOLevel);
                CopyChannel(v->Shadow[i], ch);
                continue;
            }
            if (!rebuild || !(ch.Cnt & (1u<<31))) continue;
            const u32 fmt = (ch.Cnt >> 29) & 3;
            bool bad = sh.CurSample != ch.CurSample || memcmp(sh.PrevSample, ch.PrevSample, sizeof(sh.PrevSample)) || memcmp(sh.FIFO, ch.FIFO, sizeof(sh.FIFO));
            if (fmt == 2 && ch.Pos >= 8)
                bad |= sh.ADPCMVal != ch.ADPCMVal || sh.ADPCMIndex != ch.ADPCMIndex || sh.ADPCMValLoop != ch.ADPCMValLoop
                       || sh.ADPCMIndexLoop != ch.ADPCMIndexLoop || (sh.ADPCMCurByte != ch.ADPCMCurByte);
            if (bad)
            {
                if (v->DataBad++ < 20)
                    fprintf(stderr, "SPULAZY DATA %p ch%d batch %llu fmt %u pos %d restarts %u steps %u: cur %d/%d val %d/%d idx %d/%d byte %02x/%02x fifo0 %08x/%08x\n", (void*)this, i,
                            (unsigned long long)v->Batches, fmt, ch.Pos, ch.Restarts, ch.Steps, sh.CurSample, ch.CurSample, sh.ADPCMVal, ch.ADPCMVal,
                            sh.ADPCMIndex, ch.ADPCMIndex, sh.ADPCMCurByte, ch.ADPCMCurByte, sh.FIFO[0], ch.FIFO[0]);
                CopyChannel(v->Shadow[i], ch);
            }
        }
        if ((v->Batches % 20000) == 0)
            fprintf(stderr, "SPULAZY %p: %llu batches, %llu rebuilds, timing mismatches %llu, value mismatches %llu\n", (void*)this,
                    (unsigned long long)v->Batches, (unsigned long long)v->Rebuilds, (unsigned long long)v->TimingBad, (unsigned long long)v->DataBad);
    }
#endif
}
#endif

#ifdef LITEV_SPU_CAPTURE_CHMAJOR
// Can this batch run its channels before its captures? True unless a capture flush in the next n
// ticks (at most 2 x 16 bytes from the capture's write offset, or anywhere in a buffer it wraps)
// could overlap a channel's FIFO reads (at most 64 bytes from its read offset, plus the loop start).
u32 SPU::CaptureInterleaved() const
{
    // windows in main RAM (offsets within its 4 MB); anything else, or a window that wraps it: all
    // channels stay interleaved with the captures (0xFFFF)
    const u32 mask = NDS.MainRAMMask;
    auto win = [&](u32 addr, u32 len, u32& a, u32& b) {
        if ((addr >> 24) != 0x02) return false;
        a = addr & mask; b = a + len;
        return b <= mask + 1;
    };
    u32 w0[2], w1[2]; int nw = 0;
    for (const SPUCaptureUnit& c : Capture)
    {
        if (!(c.Cnt & 0x80)) continue;
        const u32 len = c.Length ? c.Length : 4;
        const bool ok = (c.FIFOWriteOffset + 32 > len) ? win(c.DstAddr, len, w0[nw], w1[nw])
                                                         : win(c.DstAddr + c.FIFOWriteOffset, 32, w0[nw], w1[nw]);
        if (!ok) return 0xFFFF;
        nw++;
    }
    u32 late = 0;
    for (const SPUChannel& ch : Channels)
    {
        if (!(ch.Cnt & (1u<<31))) continue;
        if (((ch.Cnt >> 29) & 0x3) == 3) continue;   // PSG/noise read nothing
        const u32 total = ch.LoopPos + ch.Length;
        u32 r0, r1;
        const bool ok = (ch.KeyOn || ch.FIFOReadOffset + 64 > total) ? win(ch.SrcAddr, total + 64, r0, r1)
                                                                    : win(ch.SrcAddr + ch.FIFOReadOffset, 64, r0, r1);
        bool hit = !ok;
        for (int j = 0; j < nw && !hit; j++) hit = r0 < w1[j] && w0[j] < r1;
        if (hit) late |= 1u << ch.Num;
    }
    return late;
}
#endif

void SPU::MixSamples(u32 spucycles)
{
    LITE_PROFILE_SCOPE(spuTimer, melonDS::LiteProfile::g_Frame.SPUMixNs);
#ifdef LITEV_SPU_SILENT_LAZY
    {
        const bool quiet = LITEV_HEADLESS(Silent) && !((Capture[0].Cnt | Capture[1].Cnt) & (1<<7)) && NDS.ConsoleType == 0;
        if (quiet && (Cnt & (1<<15)) && SPULazyOn())
        {
            // the batch below with every channel's output unused: channel-major, timing only where possible
#ifdef LITEV_SPU_BATCH
            RunQuiet(spucycles, (u32)(LITEV_SPU_BATCH_N));
#else
            RunQuiet(spucycles, 1);
#endif
            return;
        }
        if (!quiet && AnyStale) MaterializeAll();
    }
#endif


#ifdef LITEV_SPU_BATCH
    // DraStic batched audio ring (teardown doc 11): generate LITEV_SPU_BATCH_N
    // samples per scheduled Event_SPU instead of one, cutting the SPU event
    // flood ~N x (547/frame -> ~547/N). Each loop iteration is bit-identical to
    // a standalone Mix() at param=spucycles (same channel advance, same blip
    // deltas, same BufferAudio thresholds); only the scheduler granularity is
    // coarsened. The ARM cores do not run between the batched samples, so SPU
    // IRQs / sound-capture writeback land up to (N-1) samples late -- a
    // deliberate FPS-first timing relaxation (flag default OFF).
#ifdef LITEV_SPU_CHMAJOR
    // Channel-major batch: run each enabled channel over the whole batch, then mix sample by sample.
    // Channels don't read each other or the mix, and nothing else runs between batched samples, so
    // each channel advances exactly as in the interleaved order (a disabled channel's Run returns 0
    // before touching anything). Not with sound capture on (it writes the mix to RAM a channel may
    // play) nor for a console nobody hears (its own path below).
    s32 chmajor[LITEV_SPU_BATCH_N][16];
    const bool chMajor = (Cnt & (1<<15)) && !((Capture[0].Cnt | Capture[1].Cnt) & (1<<7))
                         && !LITEV_HEADLESS(Silent);
#ifdef LITEV_SPU_RATE_DIV
    const u32 rateDiv = chMajor ? SPURateDiv(RateDiv) : 1;
    const u32 nticks = (u32)(LITEV_SPU_BATCH_N) / rateDiv;
    const u32 mixcyc = spucycles * rateDiv;
#else
    const u32 nticks = (u32)(LITEV_SPU_BATCH_N);
    const u32 mixcyc = spucycles;
#endif
#ifdef LITEV_SPU_FAST_ADPCM
    // leaving the fast ADPCM path (sound capture on): the exact path reads the FIFO words it skipped
    if (!chMajor)
        for (SPUChannel& ch : Channels)
            if (ch.FIFOOwed | ch.FIFOStale) { ch.FIFOCatchUp(); ch.RefillFIFO(); }
#endif
#if !defined(__ANDROID__) && defined(LITEV_SPU_FAST_ADPCM)
    if (chMajor) SPUFastVerify(*this, NDS, Channels, true, mixcyc, nticks);
#endif
#ifdef LITEV_SPU_CAPTURE_CHMAJOR
    // Sound capture on: the channels still run channel-major (exact per-sample decoding, no fast
    // ADPCM) when no capture write of this batch can land where a channel reads during it (then the
    // capture writes that come after the channel reads here changed nothing they read).
    // channels whose reads may meet a capture write in this batch stay interleaved with the captures
    const bool capMajor = !chMajor && (Cnt & (1<<15)) && NDS.ConsoleType == 0 && ((Capture[0].Cnt | Capture[1].Cnt) & 0x80);
    const u32 capLate = capMajor ? CaptureInterleaved() : 0;
#else
    constexpr bool capMajor = false;
    constexpr u32 capLate = 0;
#endif
    if (chMajor || capMajor)
    {
        for (int i = 0; i < 16; i++)
        {
            SPUChannel& ch = Channels[i];
            if (!(ch.Cnt & (1u<<31)) || (capLate & (1u << i)))
            {
                for (u32 b = 0; b < nticks; b++) chmajor[b][i] = 0;
                continue;
            }
            ch.DoRunN(mixcyc, chmajor, i, nticks, !capMajor);
        }
    }
#if !defined(__ANDROID__) && defined(LITEV_SPU_FAST_ADPCM)
    if (chMajor) SPUFastVerify(*this, NDS, Channels, false, mixcyc, nticks);
#endif
#endif
#ifndef LITEV_SPU_CHMAJOR
    const u32 nticks = (u32)(LITEV_SPU_BATCH_N);
    const u32 mixcyc = spucycles;
#endif
    for (u32 _spubatch = 0; _spubatch < nticks; _spubatch++)
    {
#else
    const u32 mixcyc = spucycles;
#endif

    s32 left = 0, right = 0;
    s32 leftoutput = 0, rightoutput = 0;

    if (Cnt & (1<<15))
    {
        // Decode all 16 channels first (sequential; each advances its own position/finish exactly),
        // then accumulate the pan mix. Channel order is unchanged (0..15).
        // A console nobody hears (Netplay's other players): the mix only feeds the speakers,
        // unless sound capture records it into memory, so skip it (and each channel's output value).
        const bool quiet = LITEV_HEADLESS(Silent) && !((Capture[0].Cnt | Capture[1].Cnt) & (1<<7)) && NDS.ConsoleType == 0;
#ifdef LITEV_ACCESS_STATS
        { extern u64 LitevAccess[6][0x10000]; for (int i = 0; i < 16; i++) if (Channels[i].Cnt & (1u<<31)) LitevAccess[5][0xD000 | ((((Channels[i].Cnt >> 29) & 3) << 10)) | ((0x10000 - Channels[i].TimerReload) >> 6 & 0x3FF)]++; }
#endif
        s32 cv[16];
#if defined(LITEV_SPU_CHMAJOR) && defined(LITEV_SPU_BATCH)
        if (chMajor || capMajor)
        {
            memcpy(cv, chmajor[_spubatch], sizeof(cv));
            for (u32 late = capLate; late; late &= late - 1)
            {
                const int i = __builtin_ctz(late);
                cv[i] = Channels[i].DoRun(mixcyc, !quiet);   // (in step with the captures, as before)
            }
        }
        else
#endif
        for (int i = 0; i < 16; i++) cv[i] = Channels[i].DoRun(mixcyc, !quiet);
        const s32 ch1 = cv[1], ch3 = cv[3];   // raw values for the routing switch below

        if (quiet)
        {
            NDS.Mic.Advance(mixcyc << 1);
            goto mixed;
        }

        // Channels 1 and 3 are conditionally muted from the MAIN mix (Cnt bits 12/13). Zero their
        // mix contribution (PanOutput of 0 == skipping it); the routing switch still uses raw ch1/ch3.
        s32 mv[16];
        for (int i = 0; i < 16; i++) mv[i] = cv[i];
        if (Cnt & (1<<12)) mv[1] = 0;
        if (Cnt & (1<<13)) mv[3] = 0;

#ifdef LITEV_SPU_MIX_NEON
        // Vectorized 16-channel pan-accumulate. Per channel PanOutput is
        //   left  += ((s64)in * (128-pan)) >> 10;   right += ((s64)in * pan) >> 10;
        // Integer NEON, BIT-EXACT: widen s32*s32->s64 (vmull_s32), shift each term >>10, sum. DS
        // channel values (|in| < ~7e7) and 16-term sums stay < 2^31, so s64 accumulation narrowed
        // once == the scalar per-term s32 accumulation (no overflow, no wrap).
        s32 pn[16];
        for (int i = 0; i < 16; i++) pn[i] = Channels[i].Pan;
        int64x2_t lacc = vdupq_n_s64(0), racc = vdupq_n_s64(0);
        for (int i = 0; i < 16; i += 4)
        {
            int32x4_t v    = vld1q_s32(&mv[i]);
            int32x4_t p    = vld1q_s32(&pn[i]);
            int32x4_t invp = vsubq_s32(vdupq_n_s32(128), p);
            lacc = vaddq_s64(lacc, vshrq_n_s64(vmull_s32(vget_low_s32(v),  vget_low_s32(invp)),  10));
            lacc = vaddq_s64(lacc, vshrq_n_s64(vmull_s32(vget_high_s32(v), vget_high_s32(invp)), 10));
            racc = vaddq_s64(racc, vshrq_n_s64(vmull_s32(vget_low_s32(v),  vget_low_s32(p)),  10));
            racc = vaddq_s64(racc, vshrq_n_s64(vmull_s32(vget_high_s32(v), vget_high_s32(p)), 10));
        }
        left  = (s32)(vgetq_lane_s64(lacc, 0) + vgetq_lane_s64(lacc, 1));
        right = (s32)(vgetq_lane_s64(racc, 0) + vgetq_lane_s64(racc, 1));
#else
        for (int i = 0; i < 16; i++)
        {
            const s32 pan = Channels[i].Pan;
            left  += ((s64)mv[i] * (128 - pan)) >> 10;
            right += ((s64)mv[i] * pan) >> 10;
        }
#endif

        // sound capture
        // TODO: other sound capture sources, along with their bugs

        if (Capture[0].Cnt & (1<<7))
        {
            s32 val = left;

            val >>= 8;
            if      (val < -0x8000) val = -0x8000;
            else if (val > 0x7FFF)  val = 0x7FFF;

            Capture[0].Run(spucycles, val);
        }

        if (Capture[1].Cnt & (1<<7))
        {
            s32 val = right;

            val >>= 8;
            if      (val < -0x8000) val = -0x8000;
            else if (val > 0x7FFF)  val = 0x7FFF;

            Capture[1].Run(spucycles, val);
        }

        // final output

        switch (Cnt & 0x0300)
        {
        case 0x0000: // left mixer
            leftoutput = left;
            break;
        case 0x0100: // channel 1
            {
                s32 pan = 128 - Channels[1].Pan;
                leftoutput = ((s64)ch1 * pan) >> 10;
            }
            break;
        case 0x0200: // channel 3
            {
                s32 pan = 128 - Channels[3].Pan;
                leftoutput = ((s64)ch3 * pan) >> 10;
            }
            break;
        case 0x0300: // channel 1+3
            {
                s32 pan1 = 128 - Channels[1].Pan;
                s32 pan3 = 128 - Channels[3].Pan;
                leftoutput = (((s64)ch1 * pan1) >> 10) + (((s64)ch3 * pan3) >> 10);
            }
            break;
        }

        switch (Cnt & 0x0C00)
        {
        case 0x0000: // right mixer
            rightoutput = right;
            break;
        case 0x0400: // channel 1
            {
                s32 pan = Channels[1].Pan;
                rightoutput = ((s64)ch1 * pan) >> 10;
            }
            break;
        case 0x0800: // channel 3
            {
                s32 pan = Channels[3].Pan;
                rightoutput = ((s64)ch3 * pan) >> 10;
            }
            break;
        case 0x0C00: // channel 1+3
            {
                s32 pan1 = Channels[1].Pan;
                s32 pan3 = Channels[3].Pan;
                rightoutput = (((s64)ch1 * pan1) >> 10) + (((s64)ch3 * pan3) >> 10);
            }
            break;
        }
    }

    leftoutput = ((s64)leftoutput * MasterVolume) >> 7;
    rightoutput = ((s64)rightoutput * MasterVolume) >> 7;

    leftoutput >>= 8;
    rightoutput >>= 8;

    // Add SOUNDBIAS value
    // The value used by all commercial games is 0x200, so we subtract that so it won't offset the final sound output.
    if (ApplyBias)
    {
        leftoutput += (Bias << 6) - 0x8000;
        rightoutput += (Bias << 6) - 0x8000;
    }

    s16 output[2];
    if (Mute)
    {
        // on the DSi, POWCNT2 bit 0 only disables NITRO mixer output
        output[0] = 0;
        output[1] = 0;
    }
    else
    {
        output[0] = (s16)std::clamp(leftoutput, -0x8000, 0x7FFF);
        output[1] = (s16)std::clamp(rightoutput, -0x8000, 0x7FFF);
    }

    NDS.Mic.Advance(mixcyc << 1);

    if (NDS.ConsoleType == 1)
    {
        // for the DSi, we run the I2S interface here, so it can mix in DSP audio
        // this isn't the cleanest, but it's the easiest, since the audio output apparatus is here
        ((DSi&)NDS).I2S.SampleClock(output);
    }

    // The original DS and DS lite degrade the output from 16 to 10 bit before output
    if (Degrade10Bit)
    {
        output[0] &= 0xFFC0;
        output[1] &= 0xFFC0;
    }

    BlipTimer += mixcyc;

    if (output[0] != OutputLastSamples[0])
        blip_add_delta(BlipLeft, BlipTimer, (int) output[0] - OutputLastSamples[0]);
    if (output[1] != OutputLastSamples[1])
        blip_add_delta(BlipRight, BlipTimer, (int) output[1] - OutputLastSamples[1]);

    OutputLastSamples[0] = output[0];
    OutputLastSamples[1] = output[1];

    if (BlipTimer >= 512 * 128)
        BufferAudio();
mixed:;

#ifdef LITEV_SPU_BATCH
    }
#endif
}

void SPU::BufferAudio()
{
    blip_end_frame(BlipLeft, BlipTimer);
    blip_end_frame(BlipRight, BlipTimer);
    BlipTimer = 0;

    int avail = blip_samples_avail(BlipLeft);
    s16 temp[avail * 2];
    blip_read_samples(BlipLeft, temp, avail, true);
    blip_read_samples(BlipRight, temp + 1, avail, true);

    Platform::Mutex_Lock(AudioLock);
    for (int i = 0; i < avail * 2; i += 2)
    {
        OutputBuffer[OutputBufferWritePos++] = temp[i];
        OutputBuffer[OutputBufferWritePos++] = temp[i+1];

        OutputBufferWritePos &= ((2*OutputBufferSize)-1);

        if (OutputBufferWritePos == OutputBufferReadPos)
        {
            // advance the read position too, to avoid losing the entire FIFO
            OutputBufferReadPos += 2;
            OutputBufferReadPos &= ((2*OutputBufferSize)-1);
        }
    }
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::TrimOutput()
{
    Platform::Mutex_Lock(AudioLock);
    const int halflimit = (OutputBufferSize / 2);

    int readpos = OutputBufferWritePos - (halflimit*2);
    if (readpos < 0) readpos += (OutputBufferSize*2);

    OutputBufferReadPos = readpos;
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::DrainOutput()
{
    Platform::Mutex_Lock(AudioLock);
    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::InitOutput()
{
    Platform::Mutex_Lock(AudioLock);

    blip_set_rates(BlipLeft, INTERNAL_SAMPLE_RATE * OutputSkew, OutputSampleRate);
    blip_set_rates(BlipRight, INTERNAL_SAMPLE_RATE * OutputSkew, OutputSampleRate);

    u32 needSamples = (u32) ceil(INTERNAL_SAMPLE_RATE / 60 / INTERNAL_SAMPLE_RATE * OutputSampleRate);
    u32 newBufferSize = 512;
    while (newBufferSize < needSamples)
        newBufferSize <<= 1;
    newBufferSize <<= 1;

    if (newBufferSize != OutputBufferSize)
    {
        if (OutputBuffer != nullptr)
            free(OutputBuffer);
        OutputBuffer = (s16*) malloc(2 * newBufferSize * 2);
        OutputBufferSize = newBufferSize;
    }

    memset(OutputBuffer, 0, 2*OutputBufferSize*2);
    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;
    Platform::Mutex_Unlock(AudioLock);
}

int SPU::GetOutputSize() const
{
    Platform::Mutex_Lock(AudioLock);

    int ret;
    if (OutputBufferWritePos >= OutputBufferReadPos)
        ret = OutputBufferWritePos - OutputBufferReadPos;
    else
        ret = (OutputBufferSize*2) - OutputBufferReadPos + OutputBufferWritePos;

    ret >>= 1;

    Platform::Mutex_Unlock(AudioLock);
    return ret;
}

void SPU::Sync(bool wait)
{
    // this function is currently not used anywhere
    // depending on the usage context the thread safety measures could be made
    // a lot faster

    // sync to audio output in case the core is running too fast
    // * wait=true: wait until enough audio data has been played
    // * wait=false: merely skip some audio data to avoid a FIFO overflow

    const int halflimit = (OutputBufferSize / 2);

    if (wait)
    {
        // TODO: less CPU-intensive wait?
        while (GetOutputSize() > halflimit);
    }
    else if (GetOutputSize() > halflimit)
    {
        Platform::Mutex_Lock(AudioLock);

        int readpos = OutputBufferWritePos - (halflimit*2);
        if (readpos < 0) readpos += (OutputBufferSize*2);

        OutputBufferReadPos = readpos;

        Platform::Mutex_Unlock(AudioLock);
    }
}

int SPU::ReadOutput(s16* data, int samples)
{
    Platform::Mutex_Lock(AudioLock);
    if (OutputBufferReadPos == OutputBufferWritePos)
    {
        Platform::Mutex_Unlock(AudioLock);
        return 0;
    }

    for (int i = 0; i < samples; i++)
    {
        *data++ = OutputBuffer[OutputBufferReadPos++];
        *data++ = OutputBuffer[OutputBufferReadPos++];
        OutputBufferReadPos &= ((2*OutputBufferSize)-1);

        if (OutputBufferWritePos == OutputBufferReadPos)
        {
            Platform::Mutex_Unlock(AudioLock);
            return i+1;
        }
    }

    Platform::Mutex_Unlock(AudioLock);
    return samples;
}

void SPU::SetOutputSampleRate(double rate)
{
    OutputSampleRate = rate;
    InitOutput();
}

void SPU::SetOutputSkew(double skew)
{
    blip_set_rates(BlipLeft, INTERNAL_SAMPLE_RATE * skew, OutputSampleRate);
    blip_set_rates(BlipRight, INTERNAL_SAMPLE_RATE * skew, OutputSampleRate);
    OutputSkew = skew;
}


u8 SPU::Read8(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt & 0xFF;
        case 0x1: return (chan->Cnt >> 8) & 0xFF;
        case 0x2: return (chan->Cnt >> 16) & 0xFF;
        case 0x3: return chan->Cnt >> 24;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt & 0x7F;
        case 0x04000501: return Cnt >> 8;

        case 0x04000508: return Capture[0].Cnt;
        case 0x04000509: return Capture[1].Cnt;
        }
    }

    Log(LogLevel::Warn, "unknown SPU read8 %08X\n", addr);
    return 0;
}

u16 SPU::Read16(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt & 0xFFFF;
        case 0x2: return chan->Cnt >> 16;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt;
        case 0x04000504: return Bias;

        case 0x04000508: return Capture[0].Cnt | (Capture[1].Cnt << 8);
        }
    }

    Log(LogLevel::Warn, "unknown SPU read16 %08X\n", addr);
    return 0;
}

u32 SPU::Read32(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt;
        case 0x04000504: return Bias;

        case 0x04000508: return Capture[0].Cnt | (Capture[1].Cnt << 8);

        case 0x04000510: return Capture[0].DstAddr;
        case 0x04000518: return Capture[1].DstAddr;
        }
    }

    Log(LogLevel::Warn, "unknown SPU read32 %08X\n", addr);
    return 0;
}

#ifdef LITEV_SPU_FIFO_TRACK
// A write that changes what a playing channel plays (format, repeat, source, loop start, length):
// first bring what was left for later up to date with the old registers (SILENT_LAZY: rebuild the
// stale values, then step the channel exactly until its next start; FAST_ADPCM: book the FIFO
// bytes the fast path consumed). Volume, pan and pitch writes change neither.
struct SPUChannelWriteGuard
{
    SPUChannel& Ch;
    const u32 Cnt, Src, Loop, Len;
    const bool Playing;
    explicit SPUChannelWriteGuard(SPUChannel& ch) : Ch(ch), Cnt(ch.Cnt), Src(ch.SrcAddr), Loop(ch.LoopPos), Len(ch.Length),
        Playing((ch.Cnt & (1u<<31)) && !ch.KeyOn) {}
    ~SPUChannelWriteGuard()
    {
        if (!Playing || !(Ch.Cnt & (1u<<31))) return;
        if (!((Ch.Cnt ^ Cnt) & 0x78000000) && Ch.SrcAddr == Src && Ch.LoopPos == Loop && Ch.Length == Len) return;
        const u32 c = Ch.Cnt, s = Ch.SrcAddr, l = Ch.LoopPos, n = Ch.Length;
        Ch.Cnt = Cnt; Ch.SrcAddr = Src; Ch.LoopPos = Loop; Ch.Length = Len;
#ifdef LITEV_SPU_FAST_ADPCM
        Ch.FIFOCatchUp();
#endif
#ifdef LITEV_SPU_SILENT_LAZY
        if (Ch.Stale || Ch.Tracked) { Ch.Materialize(); Ch.Tracked = false; }
#endif
        Ch.Cnt = c; Ch.SrcAddr = s; Ch.LoopPos = l; Ch.Length = n;
    }
};
#define LITEV_SPU_WRITE_GUARD(chan) SPUChannelWriteGuard lazyGuard(*chan)
#else
#define LITEV_SPU_WRITE_GUARD(chan)
#endif

void SPU::Write8(u32 addr, u8 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];
        LITEV_SPU_WRITE_GUARD(chan);

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt((chan->Cnt & 0xFFFFFF00) | val); return;
        case 0x1: chan->SetCnt((chan->Cnt & 0xFFFF00FF) | (val << 8)); return;
        case 0x2: chan->SetCnt((chan->Cnt & 0xFF00FFFF) | (val << 16)); return;
        case 0x3: chan->SetCnt((chan->Cnt & 0x00FFFFFF) | (val << 24)); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = (Cnt & 0xBF00) | (val & 0x7F);
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;
        case 0x04000501:
            Cnt = (Cnt & 0x007F) | ((val & 0xBF) << 8);
            return;

        case 0x04000508:
            Capture[0].SetCnt(val);
            if (val & 0x03) Log(LogLevel::Warn, "!! UNSUPPORTED SPU CAPTURE MODE %02X\n", val);
            return;
        case 0x04000509:
            Capture[1].SetCnt(val);
            if (val & 0x03) Log(LogLevel::Warn, "!! UNSUPPORTED SPU CAPTURE MODE %02X\n", val);
            return;
        }
    }

    Log(LogLevel::Warn, "unknown SPU write8 %08X %02X\n", addr, val);
}

void SPU::Write16(u32 addr, u16 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];
        LITEV_SPU_WRITE_GUARD(chan);

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt((chan->Cnt & 0xFFFF0000) | val); return;
        case 0x2: chan->SetCnt((chan->Cnt & 0x0000FFFF) | (val << 16)); return;
        case 0x8:
            chan->SetTimerReload(val);
            if      ((addr & 0xF0) == 0x10) Capture[0].SetTimerReload(val);
            else if ((addr & 0xF0) == 0x30) Capture[1].SetTimerReload(val);
            return;
        case 0xA: chan->SetLoopPos(val); return;

        case 0xC: chan->SetLength(((chan->Length >> 2) & 0xFFFF0000) | val); return;
        case 0xE: chan->SetLength(((chan->Length >> 2) & 0x0000FFFF) | (val << 16)); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = val & 0xBF7F;
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;

        case 0x04000504:
            Bias = val & 0x3FF;
            return;

        case 0x04000508:
            Capture[0].SetCnt(val & 0xFF);
            Capture[1].SetCnt(val >> 8);
            if (val & 0x0303) Log(LogLevel::Warn, "!! UNSUPPORTED SPU CAPTURE MODE %04X\n", val);
            return;

        case 0x04000514: Capture[0].SetLength(val); return;
        case 0x0400051C: Capture[1].SetLength(val); return;
        }
    }

    Log(LogLevel::Warn, "unknown SPU write16 %08X %04X\n", addr, val);
}

void SPU::Write32(u32 addr, u32 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];
        LITEV_SPU_WRITE_GUARD(chan);

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt(val); return;
        case 0x4: chan->SetSrcAddr(val); return;
        case 0x8:
            chan->SetLoopPos(val >> 16);
            val &= 0xFFFF;
            chan->SetTimerReload(val);
            if      ((addr & 0xF0) == 0x10) Capture[0].SetTimerReload(val);
            else if ((addr & 0xF0) == 0x30) Capture[1].SetTimerReload(val);
            return;
        case 0xC: chan->SetLength(val); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = val & 0xBF7F;
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;

        case 0x04000504:
            Bias = val & 0x3FF;
            return;

        case 0x04000508:
            Capture[0].SetCnt(val & 0xFF);
            Capture[1].SetCnt(val >> 8);
            if (val & 0x0303) Log(LogLevel::Warn, "!! UNSUPPORTED SPU CAPTURE MODE %04X\n", val);
            return;

        case 0x04000510: Capture[0].SetDstAddr(val); return;
        case 0x04000514: Capture[0].SetLength(val & 0xFFFF); return;
        case 0x04000518: Capture[1].SetDstAddr(val); return;
        case 0x0400051C: Capture[1].SetLength(val & 0xFFFF); return;
        }
    }
}

}
