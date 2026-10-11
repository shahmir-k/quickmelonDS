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

#ifndef SPU_H
#define SPU_H

#include "Savestate.h"
#include <memory>
#include <vector>
#include <unordered_map>
#include "Platform.h"

struct blip_t;

// FIFO bookkeeping without the sample reads (FIFO_Skip), for channel paths that read the sample
// elsewhere (LITEV_SPU_FAST_ADPCM) or not at all (LITEV_SPU_SILENT_LAZY); RefillFIFO reads the words.
#if defined(LITEV_SPU_SILENT_LAZY) || defined(LITEV_SPU_FAST_ADPCM)
#define LITEV_SPU_FIFO_TRACK 1
#endif

namespace melonDS
{

class NDS;
class SPU;

enum class AudioSampleRate
{
    _32KHz = 0,
    _47KHz
};

enum class AudioBitDepth
{
    Auto,
    _10Bit,
    _16Bit,
};

enum class AudioInterpolation
{
    None,
    Linear,
    Cosine,
    Cubic,
    SNESGaussian
};

class SPUChannel
{
public:
    SPUChannel(u32 num, melonDS::NDS& nds, AudioInterpolation interpolation);
    void Reset();
    void DoSavestate(Savestate* file);

    static const s8 ADPCMIndexTable[8];
    static const u16 ADPCMTable[89];
    static const s16 PSGTable[8][8];

    // audio interpolation is an improvement upon the original hardware
    // (which performs no interpolation)
    AudioInterpolation InterpType = AudioInterpolation::None;

    const u32 Num;

    u32 Cnt = 0;
    u32 SrcAddr = 0;
    u16 TimerReload = 0;
    u32 LoopPos = 0;
    u32 Length = 0;

    u8 Volume = 0;
    u8 VolumeShift = 0;
    u8 Pan = 0;

    bool KeyOn = false;
    u32 Timer = 0;
    s32 Pos = 0;
    s16 PrevSample[3] {};
    s16 CurSample = 0;
    u16 NoiseVal = 0;

    s32 ADPCMVal = 0;
    s32 ADPCMIndex = 0;
    s32 ADPCMValLoop = 0;
    s32 ADPCMIndexLoop = 0;
    u8 ADPCMCurByte = 0;

    u32 FIFO[8] {};
    u32 FIFOReadPos = 0;
    u32 FIFOWritePos = 0;
    u32 FIFOReadOffset = 0;
    u32 FIFOLevel = 0;

    void FIFO_BufferData();
    template<typename T> T FIFO_ReadData();

    void SetCnt(u32 val)
    {
        u32 oldcnt = Cnt;
        Cnt = val & 0xFF7F837F;

        Volume = Cnt & 0x7F;
        if (Volume == 127) Volume++;

        const u8 volshift[4] = {4, 3, 2, 0};
        VolumeShift = volshift[(Cnt >> 8) & 0x3];

        Pan = (Cnt >> 16) & 0x7F;
        if (Pan == 127) Pan++;

        if ((val & (1<<31)) && !(oldcnt & (1<<31)))
        {
            KeyOn = true;
        }
    }

    void SetSrcAddr(u32 val) { SrcAddr = val & 0x07FFFFFC; }
    void SetTimerReload(u32 val) { TimerReload = val & 0xFFFF; }
    void SetLoopPos(u32 val) { LoopPos = (val & 0xFFFF) << 2; }
    void SetLength(u32 val) { Length = (val & 0x001FFFFF) << 2; }

    void Start();

    void NextSample_PCM8();
    void NextSample_PCM16();
    void NextSample_ADPCM();
    void NextSample_PSG();
    void NextSample_Noise();

    // out=false: advance and decode exactly, but skip the output value (nobody hears it)
    template<u32 type> s32 Run(u32 cycles, bool out = true);

#ifdef LITEV_SPU_ADPCM_MEMO
    // decoded ADPCM states of the channel's current sample, by position (see RunADPCMFast)
    struct ADPCMMemo
    {
        std::vector<u32> St;    // pos -> (u16)val | idx << 16, valid for 8 <= pos < Hi
        std::vector<u8> Raw;    // the sample bytes they were decoded from
        u32 Src = 0, Loop = 0, Total = 0;
        s32 Hi = 0;
    };
    ADPCMMemo* Memo = nullptr;  // the SPU's memo of the current sample (shared by all channels)
    u32 MemoGen = 0;
    bool MemoCheck(ADPCMMemo& m, s32 pos, s32& okUntil);
#endif
#ifdef LITEV_SPU_FAST_ADPCM
    // approximate ADPCM channel over n output ticks: state in locals, sample bytes read straight
    // from main RAM instead of through the 32-byte channel FIFO. false = not applicable
    bool RunADPCMFast(u32 cycles, s32 (*dst)[16], int col, int n);
#endif
#ifdef LITEV_SPU_CHMAJOR
    // n samples of this channel into dst[0..n)[col], the type switch hoisted out of the loop
    template<u32 type> void RunN(u32 cycles, s32 (*dst)[16], int col, int n)
    {
        for (int b = 0; b < n; b++) dst[b][col] = Run<type>(cycles, true);
    }
    void DoRunN(u32 cycles, s32 (*dst)[16], int col, int n, bool fast = true)
    {
        switch ((Cnt >> 29) & 0x3)
        {
        case 0: RunN<0>(cycles, dst, col, n); return;
        case 1: RunN<1>(cycles, dst, col, n); return;
        case 2:
#ifdef LITEV_SPU_FAST_ADPCM
            if (fast && RunADPCMFast(cycles, dst, col, n)) return;
#endif
            (void)fast;
            RunN<2>(cycles, dst, col, n); return;
        case 3:
            if (Num >= 14) { RunN<4>(cycles, dst, col, n); return; }
            if (Num >= 8)  { RunN<3>(cycles, dst, col, n); return; }
            [[fallthrough]];
        default:
            for (int b = 0; b < n; b++) dst[b][col] = 0;
        }
    }
#endif
    s32 DoRun(u32 cycles, bool out = true)
    {
        switch ((Cnt >> 29) & 0x3)
        {
        case 0: return Run<0>(cycles, out); break;
        case 1: return Run<1>(cycles, out); break;
        case 2: return Run<2>(cycles, out); break;
        case 3:
            if (Num >= 14)
            {
                return Run<4>(cycles, out);
                break;
            }
            else if (Num >= 8)
            {
                return Run<3>(cycles, out);
                break;
            }
            [[fallthrough]];
        default:
            return 0;
        }
    }

    void PanOutput(s32 in, s32& left, s32& right);

#ifdef LITEV_SPU_SILENT_LAZY
    // A console nobody hears (SPU::Silent, no sound capture) steps its main-RAM PCM/ADPCM channels'
    // timing only: position, loop, end (the busy bit the ARM7 reads), FIFO bookkeeping. The sample
    // values (decode state, FIFO words) only reach the guest through sound capture, so they are
    // rebuilt (Materialize) before capture, a savestate or a register change can see them.
    bool Stale = false;     // decoded values and the FIFO words in FIFOStale are not current
    bool Tracked = false;   // Restarts/Steps are valid since Start (every step since went through Run/RunTiming)
    u8 Restarts = 0;        // loop restarts since Start (capped: passes from the 2nd on are identical)
    u32 Steps = 0;          // samples since Start or the last loop restart
    bool LazyOK() const
    {
        if (Stale) return true;
        const u32 fmt = (Cnt >> 29) & 0x3;
        return fmt < 3 && ((Cnt >> 27) & 0x3) && (Tracked || KeyOn)
            && (SrcAddr >> 24) == 0x02 && SrcAddr + LoopPos + Length <= 0x03000000;
    }
    void CountStep(s32 oldPos)
    {
        if (Pos == oldPos + 1) Steps++;
        else { Steps = 0; if (Restarts < 6) Restarts++; }
    }
    template<u32 type> void RunTiming(u32 cycles, u32 n);
    void DoRunTiming(u32 cycles, u32 n)
    {
        switch ((Cnt >> 29) & 0x3)
        {
        case 0: RunTiming<0>(cycles, n); return;
        case 1: RunTiming<1>(cycles, n); return;
        default: RunTiming<2>(cycles, n); return;
        }
    }
    void Materialize();
#endif
#ifdef LITEV_SPU_FIFO_TRACK
    u8 FIFOStale = 0;       // FIFO slots buffered without reading the sample
    u32 FIFOSrc[8] {};      // sample offset each FIFO slot holds
    void FIFO_BufferTiming();
    void FIFO_Skip(u32 size)
    {
        FIFOReadPos = (FIFOReadPos + size) & 0x1F;
        FIFOLevel -= size;
        if (FIFOLevel <= 16) FIFO_BufferTiming();
    }
    void RefillFIFO();      // main-RAM samples only (the paths above require it)
    u32 FIFOOwed = 0;       // FIFO bytes the fast ADPCM path consumed and has not booked yet
    void FIFOCatchUp() { for (; FIFOOwed; FIFOOwed--) FIFO_Skip(1); }   // (byte by byte = the same end state)
#endif

private:
    melonDS::NDS& NDS;
};

class SPUCaptureUnit
{
public:
    SPUCaptureUnit(u32 num, melonDS::NDS&);
    void Reset();
    void DoSavestate(Savestate* file);

    const u32 Num;

    u8 Cnt = 0;
    u32 DstAddr = 0;
    u16 TimerReload = 0;
    u32 Length = 0;

    u32 Timer = 0;
    s32 Pos = 0;

    u32 FIFO[4] {};
    u32 FIFOReadPos = 0;
    u32 FIFOWritePos = 0;
    u32 FIFOWriteOffset = 0;
    u32 FIFOLevel = 0;

    void FIFO_FlushData();
    template<typename T> void FIFO_WriteData(T val);

    void SetCnt(u8 val)
    {
        if ((val & 0x80) && !(Cnt & 0x80))
            Start();

        val &= 0x8F;
        if (!(val & 0x80)) val &= ~0x01;
        Cnt = val;
    }

    void SetDstAddr(u32 val) { DstAddr = val & 0x07FFFFFC; }
    void SetTimerReload(u32 val) { TimerReload = val & 0xFFFF; }
    void SetLength(u32 val) { Length = val << 2; if (Length == 0) Length = 4; }

    void Start()
    {
        Timer = TimerReload;
        Pos = 0;
        FIFOReadPos = 0;
        FIFOWritePos = 0;
        FIFOWriteOffset = 0;
        FIFOLevel = 0;
    }

    void Run(u32 cycles, s32 sample);

private:
    melonDS::NDS& NDS;
};

class SPU
{
public:
    explicit SPU(melonDS::NDS& nds, AudioBitDepth bitdepth, AudioInterpolation interpolation, double outputSampleRate);
#ifdef LITEV_SPU_ADPCM_MEMO
    std::unordered_map<u64, std::unique_ptr<SPUChannel::ADPCMMemo>> Memos;   // by (src, loop, total)
    size_t MemoBytes = 0;
    u32 MemoGen = 1;    // bumped when Memos is cleared (channels drop their pointers)
    SPUChannel::ADPCMMemo* GetMemo(u32 src, u32 loop, u32 total);
#endif
    ~SPU();
    void Reset();
    void DoSavestate(Savestate* file);

    void Stop();

    void SetPowerCnt(u32 val);

    void SetSampleRate(AudioSampleRate rate);

    // 0=none 1=linear 2=cosine 3=cubic
    void SetInterpolation(AudioInterpolation type);

    void SetBias(u16 bias);
    void SetDegrade10Bit(bool enable);
    // Netplay: nobody hears this console; Mix skips the output path (state unchanged)
    bool Silent = false;
    // LITEV_SPU_RATE_DIV: the app's Audio quality (1 full, 2 balanced, 4 performance): mix at
    // 32768/RateDiv Hz. debug.litev.spudiv overrides it for testing.
    u32 RateDiv = 1;
    void SetDegrade10Bit(AudioBitDepth depth);
    void SetApplyBias(bool enable);

    void Mix(u32 spucycles);
    void MixSamples(u32 spucycles);
#ifdef LITEV_SPU_BENCH
    void Bench(u32 spucycles);
#endif
    void BufferAudio();

    void TrimOutput();
    void DrainOutput();
    void InitOutput();
    int GetOutputSize() const;
    void Sync(bool wait);
    int ReadOutput(s16* data, int samples);
    void SetOutputSampleRate(double rate);
    void SetOutputSkew(double skew);

    u8 Read8(u32 addr);
    u16 Read16(u32 addr);
    u32 Read32(u32 addr);
    void Write8(u32 addr, u8 val);
    void Write16(u32 addr, u16 val);
    void Write32(u32 addr, u32 val);

private:
    u32 OutputBufferSize = 0;
    double OutputSampleRate;
    double OutputSkew = 1.0;
    melonDS::NDS& NDS;

    blip_t* BlipLeft;
    blip_t* BlipRight;
    int BlipTimer = 0;

    s16* OutputBuffer;
    u32 OutputBufferWritePos = 0;
    u32 OutputBufferReadPos = 0;
    s16 OutputLastSamples[2];

    u32 MixInterval;

    Platform::Mutex* AudioLock;

    u16 Cnt = 0;
    u8 MasterVolume = 0;
    u16 Bias = 0;
    bool ApplyBias = true;
    bool Degrade10Bit = false;
    bool Mute;

    std::array<SPUChannel, 16> Channels;
    std::array<SPUCaptureUnit, 2> Capture;
#ifdef LITEV_SPU_SILENT_LAZY
    bool AnyStale = false;
    void MaterializeAll() { for (SPUChannel& ch : Channels) ch.Materialize(); AnyStale = false; }
    void RunQuiet(u32 cycles, u32 n);
#endif
#ifdef LITEV_SPU_CAPTURE_CHMAJOR
    u32 CaptureInterleaved() const;   // channels that must step in turn with sound capture
#endif
};

}
#endif // SPU_H
