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

#ifndef GPU_H
#define GPU_H

#include <memory>

#include "GPU2D.h"
#include "GPU3D.h"
#include "NonStupidBitfield.h"

namespace melonDS
{
class GPU3D;
class ARMJIT;

static constexpr u32 VRAMDirtyGranularity = 512;
class GPU;

template <u32 Size, u32 MappingGranularity>
struct VRAMTrackingSet
{
    u16 Mapping[Size / MappingGranularity];

    const u32 VRAMBitsPerMapping = MappingGranularity / VRAMDirtyGranularity;

    void Reset()
    {
        for (u32 i = 0; i < Size / MappingGranularity; i++)
        {
            // this is not a real VRAM bank
            // so it will always be a mismatch => the bank will be completely invalidated
            Mapping[i] = 0x8000;
        }
    }
    NonStupidBitField<Size/VRAMDirtyGranularity> DeriveState(const u32* currentMappings, GPU& gpu);
};

class Renderer;

class GPU
{
public:
    explicit GPU(melonDS::NDS& nds, std::unique_ptr<Renderer>&& renderer = nullptr) noexcept;
    ~GPU() noexcept;
    void Reset() noexcept;
    void Stop() noexcept;

    void DoSavestate(Savestate* file) noexcept;

    void SetRenderer(std::unique_ptr<Renderer>&& renderer) noexcept;
    const Renderer& GetRenderer() const noexcept { return *Rend; }
    Renderer& GetRenderer() noexcept { return *Rend; }

    // return value for GetFramebuffers:
    // true -> pointers to RAM framebuffers are returned via the parameters
    // false -> this renderer doesn't use RAM framebuffers
    //          - values are renderer-specific (ie. OpenGL texture handle)
    bool GetFramebuffers(void** top, void** bottom);

    u8* GetUniqueBankPtr(u32 mask, u32 offset) noexcept;
    const u8* GetUniqueBankPtr(u32 mask, u32 offset) const noexcept;

    u8 Read8(u32 addr);
    u16 Read16(u32 addr);
    u32 Read32(u32 addr);
    void Write8(u32 addr, u8 val);
    void Write16(u32 addr, u16 val);
    void Write32(u32 addr, u32 val);

    void MapVRAM_AB(u32 bank, u8 cnt) noexcept;
    void MapVRAM_CD(u32 bank, u8 cnt) noexcept;
    void MapVRAM_E(u32 bank, u8 cnt) noexcept;
    void MapVRAM_FG(u32 bank, u8 cnt) noexcept;
    void MapVRAM_H(u32 bank, u8 cnt) noexcept;
    void MapVRAM_I(u32 bank, u8 cnt) noexcept;

    /*
        VRAM syncing code for display capture blocks

        The software renderer will write display captures straight to VRAM, making this unnecessary.
        However, hardware-accelerated renderers may want to keep display captures in GPU memory unless
        it is necessary to read them back. This syncing system assists with that.

        Those checks are limited to banks A..D, since those are the only ones that can be used for
        display capture.

        TODO: make checks more efficient
    */

    void SyncVRAM_LCDC(u32 addr, bool write)
    {
        u32 bank = (addr >> 17) & 0x7;
        if (bank >= 4) return;

        if (VRAMMap_LCDC & (1<<bank))
            SyncVRAMCaptureBlock((addr >> 15) & 0xF, write);
    }

    void SyncVRAM_ABG(u32 addr, bool write)
    {
        u32 mask = VRAMMap_ABG[(addr >> 14) & 0x1F];
        addr = (addr >> 15) & 0x3;
        if (mask & (1<<0)) SyncVRAMCaptureBlock((0<<2) | addr, write);
        if (mask & (1<<1)) SyncVRAMCaptureBlock((1<<2) | addr, write);
        if (mask & (1<<2)) SyncVRAMCaptureBlock((2<<2) | addr, write);
        if (mask & (1<<3)) SyncVRAMCaptureBlock((3<<2) | addr, write);
    }

    void SyncVRAM_AOBJ(u32 addr, bool write)
    {
        u32 mask = VRAMMap_AOBJ[(addr >> 14) & 0xF];
        addr = (addr >> 15) & 0x3;
        if (mask & (1<<0)) SyncVRAMCaptureBlock((0<<2) | addr, write);
        if (mask & (1<<1)) SyncVRAMCaptureBlock((1<<2) | addr, write);
    }

    void SyncVRAM_BBG(u32 addr, bool write)
    {
        u32 mask = VRAMMap_BBG[(addr >> 14) & 0x7];
        addr = (addr >> 15) & 0x3;
        if (mask & (1<<2)) SyncVRAMCaptureBlock((2<<2) | addr, write);
    }

    void SyncVRAM_BOBJ(u32 addr, bool write)
    {
        u32 mask = VRAMMap_BOBJ[(addr >> 14) & 0x7];
        addr = (addr >> 15) & 0x3;
        if (mask & (1<<3)) SyncVRAMCaptureBlock((3<<2) | addr, write);
    }

    int GetCaptureBlock_LCDC(u32 offset);

    void GetCaptureInfo_ABG(int* info);
    void GetCaptureInfo_AOBJ(int* info);
    void GetCaptureInfo_BBG(int* info);
    void GetCaptureInfo_BOBJ(int* info);
    void GetCaptureInfo_Texture(int* info);

    template<typename T>
    T ReadVRAM_LCDC(u32 addr) const noexcept
    {
        int bank;

        switch (addr & 0xFF8FC000)
        {
        case 0x06800000: case 0x06804000: case 0x06808000: case 0x0680C000:
        case 0x06810000: case 0x06814000: case 0x06818000: case 0x0681C000:
            bank = 0;
            addr &= 0x1FFFF;
            break;

        case 0x06820000: case 0x06824000: case 0x06828000: case 0x0682C000:
        case 0x06830000: case 0x06834000: case 0x06838000: case 0x0683C000:
            bank = 1;
            addr &= 0x1FFFF;
            break;

        case 0x06840000: case 0x06844000: case 0x06848000: case 0x0684C000:
        case 0x06850000: case 0x06854000: case 0x06858000: case 0x0685C000:
            bank = 2;
            addr &= 0x1FFFF;
            break;

        case 0x06860000: case 0x06864000: case 0x06868000: case 0x0686C000:
        case 0x06870000: case 0x06874000: case 0x06878000: case 0x0687C000:
            bank = 3;
            addr &= 0x1FFFF;
            break;

        case 0x06880000: case 0x06884000: case 0x06888000: case 0x0688C000:
            bank = 4;
            addr &= 0xFFFF;
            break;

        case 0x06890000:
            bank = 5;
            addr &= 0x3FFF;
            break;

        case 0x06894000:
            bank = 6;
            addr &= 0x3FFF;
            break;

        case 0x06898000:
        case 0x0689C000:
            bank = 7;
            addr &= 0x7FFF;
            break;

        case 0x068A0000:
            bank = 8;
            addr &= 0x3FFF;
            break;

        default: return 0;
        }

        if (VRAMMap_LCDC & (1<<bank)) return *(T*)&VRAM[bank][addr];

        return 0;
    }

    template<typename T>
    void WriteVRAM_LCDC(u32 addr, T val)
    {
        int bank;

        switch (addr & 0xFF8FC000)
        {
        case 0x06800000: case 0x06804000: case 0x06808000: case 0x0680C000:
        case 0x06810000: case 0x06814000: case 0x06818000: case 0x0681C000:
            bank = 0;
            addr &= 0x1FFFF;
            break;

        case 0x06820000: case 0x06824000: case 0x06828000: case 0x0682C000:
        case 0x06830000: case 0x06834000: case 0x06838000: case 0x0683C000:
            bank = 1;
            addr &= 0x1FFFF;
            break;

        case 0x06840000: case 0x06844000: case 0x06848000: case 0x0684C000:
        case 0x06850000: case 0x06854000: case 0x06858000: case 0x0685C000:
            bank = 2;
            addr &= 0x1FFFF;
            break;

        case 0x06860000: case 0x06864000: case 0x06868000: case 0x0686C000:
        case 0x06870000: case 0x06874000: case 0x06878000: case 0x0687C000:
            bank = 3;
            addr &= 0x1FFFF;
            break;

        case 0x06880000: case 0x06884000: case 0x06888000: case 0x0688C000:
            bank = 4;
            addr &= 0xFFFF;
            break;

        case 0x06890000:
            bank = 5;
            addr &= 0x3FFF;
            break;

        case 0x06894000:
            bank = 6;
            addr &= 0x3FFF;
            break;

        case 0x06898000:
        case 0x0689C000:
            bank = 7;
            addr &= 0x7FFF;
            break;

        case 0x068A0000:
            bank = 8;
            addr &= 0x3FFF;
            break;

        default: return;
        }

        if (VRAMMap_LCDC & (1<<bank))
        {
            *(T*)&VRAM[bank][addr] = val;
            VRAMDirty[bank][addr / VRAMDirtyGranularity] = true;
        }
    }


    template<typename T>
    T ReadVRAM_ABG(u32 addr) const noexcept
    {
        u8* ptr = VRAMPtr_ABG[(addr >> 14) & 0x1F];
        if (ptr) return *(T*)&ptr[addr & 0x3FFF];

        T ret = 0;
        u32 mask = VRAMMap_ABG[(addr >> 14) & 0x1F];

        if (mask & (1<<0)) ret |= *(T*)&VRAM_A[addr & 0x1FFFF];
        if (mask & (1<<1)) ret |= *(T*)&VRAM_B[addr & 0x1FFFF];
        if (mask & (1<<2)) ret |= *(T*)&VRAM_C[addr & 0x1FFFF];
        if (mask & (1<<3)) ret |= *(T*)&VRAM_D[addr & 0x1FFFF];
        if (mask & (1<<4)) ret |= *(T*)&VRAM_E[addr & 0xFFFF];
        if (mask & (1<<5)) ret |= *(T*)&VRAM_F[addr & 0x3FFF];
        if (mask & (1<<6)) ret |= *(T*)&VRAM_G[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    void WriteVRAM_ABG(u32 addr, T val)
    {
        u32 mask = VRAMMap_ABG[(addr >> 14) & 0x1F];

        if (mask & (1<<0))
        {
            VRAMDirty[0][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_A[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<1))
        {
            VRAMDirty[1][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_B[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<2))
        {
            VRAMDirty[2][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_C[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<3))
        {
            VRAMDirty[3][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_D[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<4))
        {
            VRAMDirty[4][(addr & 0xFFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_E[addr & 0xFFFF] = val;
        }
        if (mask & (1<<5))
        {
            VRAMDirty[5][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_F[addr & 0x3FFF] = val;
        }
        if (mask & (1<<6))
        {
            VRAMDirty[6][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_G[addr & 0x3FFF] = val;
        }
    }


    template<typename T>
    T ReadVRAM_AOBJ(u32 addr) const noexcept
    {
        u8* ptr = VRAMPtr_AOBJ[(addr >> 14) & 0xF];
        if (ptr) return *(T*)&ptr[addr & 0x3FFF];

        T ret = 0;
        u32 mask = VRAMMap_AOBJ[(addr >> 14) & 0xF];

        if (mask & (1<<0)) ret |= *(T*)&VRAM_A[addr & 0x1FFFF];
        if (mask & (1<<1)) ret |= *(T*)&VRAM_B[addr & 0x1FFFF];
        if (mask & (1<<4)) ret |= *(T*)&VRAM_E[addr & 0xFFFF];
        if (mask & (1<<5)) ret |= *(T*)&VRAM_F[addr & 0x3FFF];
        if (mask & (1<<6)) ret |= *(T*)&VRAM_G[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    void WriteVRAM_AOBJ(u32 addr, T val)
    {
        u32 mask = VRAMMap_AOBJ[(addr >> 14) & 0xF];

        if (mask & (1<<0))
        {
            VRAMDirty[0][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_A[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<1))
        {
            VRAMDirty[1][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_B[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<4))
        {
            VRAMDirty[4][(addr & 0xFFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_E[addr & 0xFFFF] = val;
        }
        if (mask & (1<<5))
        {
            VRAMDirty[5][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_F[addr & 0x3FFF] = val;
        }
        if (mask & (1<<6))
        {
            VRAMDirty[6][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_G[addr & 0x3FFF] = val;
        }
    }


    template<typename T>
    T ReadVRAM_BBG(u32 addr) const noexcept
    {
        u8* ptr = VRAMPtr_BBG[(addr >> 14) & 0x7];
        if (ptr) return *(T*)&ptr[addr & 0x3FFF];

        T ret = 0;
        u32 mask = VRAMMap_BBG[(addr >> 14) & 0x7];

        if (mask & (1<<2)) ret |= *(T*)&VRAM_C[addr & 0x1FFFF];
        if (mask & (1<<7)) ret |= *(T*)&VRAM_H[addr & 0x7FFF];
        if (mask & (1<<8)) ret |= *(T*)&VRAM_I[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    void WriteVRAM_BBG(u32 addr, T val)
    {
        u32 mask = VRAMMap_BBG[(addr >> 14) & 0x7];

        if (mask & (1<<2))
        {
            VRAMDirty[2][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_C[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<7))
        {
            VRAMDirty[7][(addr & 0x7FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_H[addr & 0x7FFF] = val;
        }
        if (mask & (1<<8))
        {
            VRAMDirty[8][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_I[addr & 0x3FFF] = val;
        }
    }


    template<typename T>
    T ReadVRAM_BOBJ(u32 addr) const noexcept
    {
        u8* ptr = VRAMPtr_BOBJ[(addr >> 14) & 0x7];
        if (ptr) return *(T*)&ptr[addr & 0x3FFF];

        T ret = 0;
        u32 mask = VRAMMap_BOBJ[(addr >> 14) & 0x7];

        if (mask & (1<<3)) ret |= *(T*)&VRAM_D[addr & 0x1FFFF];
        if (mask & (1<<8)) ret |= *(T*)&VRAM_I[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    void WriteVRAM_BOBJ(u32 addr, T val)
    {
        u32 mask = VRAMMap_BOBJ[(addr >> 14) & 0x7];

        if (mask & (1<<3))
        {
            VRAMDirty[3][(addr & 0x1FFFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_D[addr & 0x1FFFF] = val;
        }
        if (mask & (1<<8))
        {
            VRAMDirty[8][(addr & 0x3FFF) / VRAMDirtyGranularity] = true;
            *(T*)&VRAM_I[addr & 0x3FFF] = val;
        }
    }

    template<typename T>
    T ReadVRAM_ARM7(u32 addr) const noexcept
    {
        T ret = 0;
        u32 mask = VRAMMap_ARM7[(addr >> 17) & 0x1];

        if (mask & (1<<2)) ret |= *(T*)&VRAM_C[addr & 0x1FFFF];
        if (mask & (1<<3)) ret |= *(T*)&VRAM_D[addr & 0x1FFFF];

        return ret;
    }

    template<typename T>
    void WriteVRAM_ARM7(u32 addr, T val)
    {
        u32 mask = VRAMMap_ARM7[(addr >> 17) & 0x1];

        if (mask & (1<<2)) *(T*)&VRAM_C[addr & 0x1FFFF] = val;
        if (mask & (1<<3)) *(T*)&VRAM_D[addr & 0x1FFFF] = val;
    }


    template<typename T>
    T ReadVRAM_BG(u32 addr) const noexcept
    {
        if ((addr & 0xFFE00000) == 0x06000000)
            return ReadVRAM_ABG<T>(addr);
        else
            return ReadVRAM_BBG<T>(addr);
    }

    template<typename T>
    T ReadVRAM_OBJ(u32 addr) const noexcept
    {
        if ((addr & 0xFFE00000) == 0x06400000)
            return ReadVRAM_AOBJ<T>(addr);
        else
            return ReadVRAM_BOBJ<T>(addr);
    }


    template<typename T>
    T ReadVRAM_Texture(u32 addr) const noexcept
    {
        T ret = 0;
        u32 mask = VRAMMap_Texture[(addr >> 17) & 0x3];

        if (mask & (1<<0)) ret |= *(T*)&VRAM_A[addr & 0x1FFFF];
        if (mask & (1<<1)) ret |= *(T*)&VRAM_B[addr & 0x1FFFF];
        if (mask & (1<<2)) ret |= *(T*)&VRAM_C[addr & 0x1FFFF];
        if (mask & (1<<3)) ret |= *(T*)&VRAM_D[addr & 0x1FFFF];

        return ret;
    }

    template<typename T>
    T ReadVRAM_TexPal(u32 addr) const noexcept
    {
        T ret = 0;
        u32 mask = VRAMMap_TexPal[(addr >> 14) & 0x7];

        if (mask & (1<<4)) ret |= *(T*)&VRAM_E[addr & 0xFFFF];
        if (mask & (1<<5)) ret |= *(T*)&VRAM_F[addr & 0x3FFF];
        if (mask & (1<<6)) ret |= *(T*)&VRAM_G[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    T ReadPalette(u32 addr) const noexcept
    {
        return *(T*)&Palette[addr & 0x7FF];
    }

    template<typename T>
    void WritePalette(u32 addr, T val)
    {
        addr &= 0x7FF;

        *(T*)&Palette[addr] = val;
        if (addr & 0x3FE)
            PaletteDirty |= 1 << (addr / VRAMDirtyGranularity);
        else
            PaletteDirty |= 0x10 << (addr / VRAMDirtyGranularity);
    }

    template<typename T>
    T ReadOAM(u32 addr) const noexcept
    {
        return *(T*)&OAM[addr & 0x7FF];
    }

    template<typename T>
    void WriteOAM(u32 addr, T val)
    {
        addr &= 0x7FF;

        *(T*)&OAM[addr] = val;
        OAMDirty |= 1 << (addr / 1024);
    }

    template <typename T>
    inline T ReadVRAMFlat_Texture(u32 addr) const
    {
        return *(T*)&VRAMFlat_Texture[addr & 0x7FFFF];
    }
    template <typename T>
    inline T ReadVRAMFlat_TexPal(u32 addr) const
    {
        return *(T*)&VRAMFlat_TexPal[addr & 0x1FFFF];
    }

    void SetPowerCnt(u32 val) noexcept;

    void StartFrame() noexcept;
    void FinishFrame(u32 lines) noexcept;
    void BlankFrame() noexcept;
    void StartScanline(u32 line) noexcept;
    void StartHBlank(u32 line) noexcept;

    void Restart3DFrame() noexcept;

    void DisplayFIFO(u32 x) noexcept;

    void SetDispStat(u32 cpu, u16 val, u16 mask) noexcept;
    void SetVCount(u16 val, u16 mask) noexcept;

    bool MakeVRAMFlat_ABGCoherent(NonStupidBitField<512*1024/VRAMDirtyGranularity>& dirty) noexcept;
    bool MakeVRAMFlat_BBGCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty) noexcept;

    bool MakeVRAMFlat_AOBJCoherent(NonStupidBitField<256*1024/VRAMDirtyGranularity>& dirty) noexcept;
    bool MakeVRAMFlat_BOBJCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty) noexcept;

    bool MakeVRAMFlat_ABGExtPalCoherent(NonStupidBitField<32*1024/VRAMDirtyGranularity>& dirty) noexcept;
    bool MakeVRAMFlat_BBGExtPalCoherent(NonStupidBitField<32*1024/VRAMDirtyGranularity>& dirty) noexcept;

    bool MakeVRAMFlat_AOBJExtPalCoherent(NonStupidBitField<8*1024/VRAMDirtyGranularity>& dirty) noexcept;
    bool MakeVRAMFlat_BOBJExtPalCoherent(NonStupidBitField<8*1024/VRAMDirtyGranularity>& dirty) noexcept;

    // dst: write the chunks there instead of VRAMFlat_* (the hybrid's emu-side mirror)
    bool MakeVRAMFlat_TextureCoherent(NonStupidBitField<512*1024/VRAMDirtyGranularity>& dirty, u8* dst = nullptr) noexcept;
    bool MakeVRAMFlat_TexPalCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty, u8* dst = nullptr) noexcept;

    melonDS::NDS& NDS;

    bool ScreensEnabled = false;
    bool ScreenSwap = false;

    u16 VCount = 0;
    u16 TotalScanlines = 0;

#ifdef LITEV_AGGRESSIVE_SKIP
    // liteDS-v2 (M4): per-frame 2D/3D rasterization skip gate. Renders 1 frame,
    // then skips FrameskipTarget frames' worth of rasterization. CPU/DMA/timers/
    // WiFi always run; only the (soft) renderer draw calls are gated.
    int FrameskipTarget = 0;
    int FrameskipCounter = 0;
    bool SkipThisFrame = false;
    static constexpr int LITEV_FRAMESKIP_MAX = 9;

    // Netplay: a console nobody looks at (another player's) draws nothing. Rendering does not
    // feed back into emulation, except through display capture, so once the game has used
    // capture it renders fully from then on.
    // ponytail: the very first capture's 3D layer is the stale one (3D for a frame is drawn
    // during the previous one); render 3D every frame for that if a game desyncs on it.
    bool Headless = false;
    bool CaptureSeen = false;
    // Record mode: skipping a frame's drawing must not change guest state, but a display capture
    // writes the drawn picture into VRAM. Once the game has captured, draw every frame (fast-forward
    // and auto frameskip then only drop the sleep), so a recording replays exactly.
    // ponytail: a first capture right after a skipped frame can still see stale 3D; the replay's
    // hash check reports it.
    bool KeepCaptures = false;
    bool KeepCapturesSeen = false;
#ifdef LITEV_NETPLAY_CAPTURE
    // LITEV_NETPLAY_CAPTURE: captured pixels only matter to emulation if the CPU reads them back
    // (games use capture for display effects: motion blur, a frozen 3D frame shown as a BG). So
    // CaptureSeen is set when an ARM9/DMA access reads a VRAM block a capture wrote (or such a
    // bank is handed to the ARM7), not when capture is merely used. CaptureTaint = 32 KB blocks
    // of banks A-D a capture has written (bit = bank*4 + block), never cleared.
    // ponytail: sticky taint over-reports after the game overwrites a captured block (fallback =
    // the slow everything-software path, still correct); clear per block on full overwrite if a
    // game trips it. A read that does fire may already differ between devices (the local console's
    // GPU capture is not bit-exact with software), so it is logged as a desync risk.
    u16 CaptureTaint = 0;
#endif
    int DiagNoDraw = 0;         // TEMP diagnostic: bit0 skip 2D drawing, bit1 skip 3D rendering   // the game has used display capture (Netplay then renders identically everywhere)

#ifdef LITEV_SKIP_REPEAT_FRAMES
    // Display-only: while a game draws 3D at 30 Hz (new 3D every other frame), skip the 2D
    // drawing and the present of each frame whose 3D repeats the previous frame's; the screen
    // keeps the previous frame (only 2D changes on that frame show one frame late). Never on a
    // headless console or once the game has used display capture (drawn pixels then reach VRAM).
    bool SkipRepeatEnabled = false;
    bool SkipRepeat = false;         // this frame (valid until the next frame starts)
    bool NextSkipRepeat = false;     // decided at line 262, before the next frame's line-0 sprites
    bool LastRepeat3D = false;
#endif

    void SetFrameskipTarget(int target) noexcept
    {
        if (target < 0) target = 0;
        if (target > LITEV_FRAMESKIP_MAX) target = LITEV_FRAMESKIP_MAX;
        FrameskipTarget = target;
        FrameskipCounter = 0;
        SkipThisFrame = false;
    }
#endif // LITEV_AGGRESSIVE_SKIP

    u16 DispStat[2] {};
    u8 VRAMCNT[9] {};
    u8 VRAMSTAT = 0;

    u16 MasterBrightnessA;
    u16 MasterBrightnessB;
#ifdef LITEV_MASTERBRIGHT_LATCH
    // Display only (not guest state): the brightness the renderers draw a line with. A game that
    // changes MASTER_BRIGHT once mid-frame (a fade step landing late, after the game missed
    // VBlank) would tear that frame, so a single change takes effect from the next frame; a
    // second change in the same frame is a deliberate raster effect and is followed line by line.
    u16 DrawBrightness[2] {}, BrightLatch[2] {}, BrightLast[2] {};
    u8 BrightChanges[2] {};
    void UpdateDrawBrightness() noexcept
    {
        const u16 cur[2] = { MasterBrightnessA, MasterBrightnessB };
        for (int s = 0; s < 2; s++)
        {
            if (VCount == 0) { BrightLatch[s] = BrightLast[s] = cur[s]; BrightChanges[s] = 0; }
            else if (cur[s] != BrightLast[s]) { BrightLast[s] = cur[s]; BrightChanges[s]++; }
            DrawBrightness[s] = BrightChanges[s] <= 1 ? BrightLatch[s] : cur[s];
        }
    }
    u16 DrawBrightnessA() const noexcept { return DrawBrightness[0]; }
    u16 DrawBrightnessB() const noexcept { return DrawBrightness[1]; }
#else
    u16 DrawBrightnessA() const noexcept { return MasterBrightnessA; }
    u16 DrawBrightnessB() const noexcept { return MasterBrightnessB; }
#endif

    u16 DispFIFO[16];
    u8 DispFIFOReadPtr;
    u8 DispFIFOWritePtr;
    alignas(8) u16 DispFIFOBuffer[256];

    u32 CaptureCnt;
    bool CaptureEnable;
    u32 CaptureCount = 0; // diagnostics only: display captures started (not serialized)

    alignas(u64) u8 Palette[2*1024] {};
    alignas(u64) u8 OAM[2*1024] {};

    alignas(u64) u8 VRAM_A[128*1024] {};
    alignas(u64) u8 VRAM_B[128*1024] {};
    alignas(u64) u8 VRAM_C[128*1024] {};
    alignas(u64) u8 VRAM_D[128*1024] {};
    alignas(u64) u8 VRAM_E[ 64*1024] {};
    alignas(u64) u8 VRAM_F[ 16*1024] {};
    alignas(u64) u8 VRAM_G[ 16*1024] {};
    alignas(u64) u8 VRAM_H[ 32*1024] {};
    alignas(u64) u8 VRAM_I[ 16*1024] {};

    u8* const VRAM[9]     = {VRAM_A,  VRAM_B,  VRAM_C,  VRAM_D,  VRAM_E, VRAM_F, VRAM_G, VRAM_H, VRAM_I};
    u32 const VRAMMask[9] = {0x1FFFF, 0x1FFFF, 0x1FFFF, 0x1FFFF, 0xFFFF, 0x3FFF, 0x3FFF, 0x7FFF, 0x3FFF};

    u32 VRAMMap_LCDC = 0;
    u32 VRAMMap_ABG[0x20] {};
    u32 VRAMMap_AOBJ[0x10] {};
    u32 VRAMMap_BBG[0x8] {};
    u32 VRAMMap_BOBJ[0x8] {};
    u32 VRAMMap_ABGExtPal[4] {};
    u32 VRAMMap_AOBJExtPal {};
    u32 VRAMMap_BBGExtPal[4] {};
    u32 VRAMMap_BOBJExtPal {};
    u32 VRAMMap_Texture[4] {};
    u32 VRAMMap_TexPal[8] {};
    u32 VRAMMap_ARM7[2] {};

    u8* VRAMPtr_ABG[0x20] {};
    u8* VRAMPtr_AOBJ[0x10] {};
    u8* VRAMPtr_BBG[0x8] {};
    u8* VRAMPtr_BOBJ[0x8] {};

    melonDS::GPU2D GPU2D_A;
    melonDS::GPU2D GPU2D_B;
    melonDS::GPU3D GPU3D;

    NonStupidBitField<128*1024/VRAMDirtyGranularity> VRAMDirty[9] {};
    VRAMTrackingSet<512*1024, 16*1024> VRAMDirty_ABG {};
    VRAMTrackingSet<256*1024, 16*1024> VRAMDirty_AOBJ {};
    VRAMTrackingSet<128*1024, 16*1024> VRAMDirty_BBG {};
    VRAMTrackingSet<128*1024, 16*1024> VRAMDirty_BOBJ {};

    VRAMTrackingSet<32*1024, 8*1024> VRAMDirty_ABGExtPal {};
    VRAMTrackingSet<32*1024, 8*1024> VRAMDirty_BBGExtPal {};
    VRAMTrackingSet<8*1024, 8*1024> VRAMDirty_AOBJExtPal {};
    VRAMTrackingSet<8*1024, 8*1024> VRAMDirty_BOBJExtPal {};

    VRAMTrackingSet<512*1024, 128*1024> VRAMDirty_Texture {};
    VRAMTrackingSet<128*1024, 16*1024> VRAMDirty_TexPal {};

    u8 VRAMFlat_ABG[512*1024] {};
    u8 VRAMFlat_BBG[128*1024] {};
    u8 VRAMFlat_AOBJ[256*1024] {};
    u8 VRAMFlat_BOBJ[128*1024] {};

    alignas(u16) u8 VRAMFlat_ABGExtPal[32*1024] {};
    alignas(u16) u8 VRAMFlat_BBGExtPal[32*1024] {};

    alignas(u16) u8 VRAMFlat_AOBJExtPal[8*1024] {};
    alignas(u16) u8 VRAMFlat_BOBJExtPal[8*1024] {};

    alignas(u64) u8 VRAMFlat_Texture[512*1024] {};
    alignas(u64) u8 VRAMFlat_TexPal[128*1024] {};

#ifdef LITEV_SNAP_DIRTY
    // P2-dirty (2026-07-17): incremental shadow snapshots. The flat mirrors are only ever
    // mutated by the Make*Coherent dirty-driven copies (CopyLinearVRAM applies exactly the
    // chunks in the caller-derived dirty set; mapping changes arrive pre-marked by
    // DeriveState). Accumulating those same 512B-chunk dirty bits into a per-shadow-bank
    // pending mask lets Snapshot*Shadow copy ONLY chunks that changed since that bank's
    // last snapshot, instead of a blind full memcpy (tex 640KB/frame, BGOBJ 1.1MB/frame —
    // measured 2026-07-17 as the #1 emu-thread backend-stall source, ~18% of stalls in
    // __memcpy). Bit-exact by construction: shadow bytes == full-copy bytes whenever
    // pending ⊇ chunks-changed-since-this-bank's-last-copy, which the accumulation
    // guarantees; ResetVRAMCache re-arms a full copy. Storage: bitmasks only (~4.5KB).
    template<size_t BYTES> struct SnapPend
    {
        static constexpr size_t NBITS  = BYTES / 512;
        static constexpr size_t NWORDS = (NBITS + 63) / 64;
        u64 bits[2][NWORDS] {};
        void arm() noexcept { memset(bits, 0xFF, sizeof(bits)); }   // force full copy next snapshot
        template<u32 N> void add(const NonStupidBitField<N>& d) noexcept
        {
            static_assert(N == NBITS, "dirty granularity mismatch");
            for (size_t i = 0; i < NWORDS; i++) { bits[0][i] |= d.Data[i]; bits[1][i] |= d.Data[i]; }
        }
        void copy(int bank, u8* dst, const u8* src) noexcept
        {
            for (size_t w = 0; w < NWORDS; w++)
            {
                u64 m = bits[bank][w]; bits[bank][w] = 0;
                while (m)
                {
                    const int b = __builtin_ctzll(m); m &= m - 1;
                    const size_t off = ((w * 64) + (size_t)b) * 512;
                    if (off < BYTES) memcpy(dst + off, src + off, (BYTES - off) < 512 ? (BYTES - off) : 512);
                }
            }
        }
    };
#endif


#if defined(LITEV_SOFT2D_DEPTH2)
    // Part 1b (flat-VRAM parity snapshot, 2D side): A/B parity shadow of the flat BG/OBJ/
    // ext-pal VRAM, mirroring the R4 texture shadow above. The 2D raster reads these via
    // GetBGVRAM/GetOBJVRAM/GetBGExtPal/GetOBJExtPal, redirected through the *Read pointers to
    // the 2D CONSUMER's frame bank (SetBGOBJReadShadow, on the async 2D thread) -- INDEPENDENT
    // of the 3D texture read pointer (which the render thread points at the 3D render bank).
    // SnapshotBGOBJShadow copies ~1.1 MB/frame (~0.3 ms on the A55); +~2.2 MB storage.
    // Byte-identical at depth-1 (shadow == live coherent data). Under depth-2 the emu's next-
    // frame snapshot writes the OTHER bank -> cannot corrupt the in-flight raster's bytes.
    u8 VRAMFlat_ABGShadow[2][512*1024] {};
    u8 VRAMFlat_BBGShadow[2][128*1024] {};
    u8 VRAMFlat_AOBJShadow[2][256*1024] {};
    u8 VRAMFlat_BOBJShadow[2][128*1024] {};
    alignas(u16) u8 VRAMFlat_ABGExtPalShadow[2][32*1024] {};
    alignas(u16) u8 VRAMFlat_BBGExtPalShadow[2][32*1024] {};
    alignas(u16) u8 VRAMFlat_AOBJExtPalShadow[2][8*1024] {};
    alignas(u16) u8 VRAMFlat_BOBJExtPalShadow[2][8*1024] {};
    u8* VRAMFlat_ABGRead = VRAMFlat_ABG;
    u8* VRAMFlat_BBGRead = VRAMFlat_BBG;
    u8* VRAMFlat_AOBJRead = VRAMFlat_AOBJ;
    u8* VRAMFlat_BOBJRead = VRAMFlat_BOBJ;
    u8* VRAMFlat_ABGExtPalRead = VRAMFlat_ABGExtPal;
    u8* VRAMFlat_BBGExtPalRead = VRAMFlat_BBGExtPal;
    u8* VRAMFlat_AOBJExtPalRead = VRAMFlat_AOBJExtPal;
    u8* VRAMFlat_BOBJExtPalRead = VRAMFlat_BOBJExtPal;
#ifdef LITEV_SNAP_DIRTY
    SnapPend<512*1024> SnapPendABG;
    SnapPend<128*1024> SnapPendBBG;
    SnapPend<256*1024> SnapPendAOBJ;
    SnapPend<128*1024> SnapPendBOBJ;
    SnapPend<32*1024>  SnapPendABGExtPal;
    SnapPend<32*1024>  SnapPendBBGExtPal;
    SnapPend<8*1024>   SnapPendAOBJExtPal;
    SnapPend<8*1024>   SnapPendBOBJExtPal;
#endif
    void SnapshotBGOBJShadow(int bank) noexcept;
    void SetBGOBJReadShadow(bool on, int bank) noexcept
    {
        VRAMFlat_ABGRead = on ? VRAMFlat_ABGShadow[bank] : VRAMFlat_ABG;
        VRAMFlat_BBGRead = on ? VRAMFlat_BBGShadow[bank] : VRAMFlat_BBG;
        VRAMFlat_AOBJRead = on ? VRAMFlat_AOBJShadow[bank] : VRAMFlat_AOBJ;
        VRAMFlat_BOBJRead = on ? VRAMFlat_BOBJShadow[bank] : VRAMFlat_BOBJ;
        VRAMFlat_ABGExtPalRead = on ? VRAMFlat_ABGExtPalShadow[bank] : VRAMFlat_ABGExtPal;
        VRAMFlat_BBGExtPalRead = on ? VRAMFlat_BBGExtPalShadow[bank] : VRAMFlat_BBGExtPal;
        VRAMFlat_AOBJExtPalRead = on ? VRAMFlat_AOBJExtPalShadow[bank] : VRAMFlat_AOBJExtPal;
        VRAMFlat_BOBJExtPalRead = on ? VRAMFlat_BOBJExtPalShadow[bank] : VRAMFlat_BOBJExtPal;
    }
#endif

    u32 OAMDirty = 0;
    u32 PaletteDirty = 0;

private:
    void ResetVRAMCache() noexcept;

    template<typename T>
    T ReadVRAM_ABGExtPal(u32 addr) const noexcept
    {
        u32 mask = VRAMMap_ABGExtPal[(addr >> 13) & 0x3];

        T ret = 0;
        if (mask & (1<<4)) ret |= *(T*)&VRAM_E[addr & 0x7FFF];
        if (mask & (1<<5)) ret |= *(T*)&VRAM_F[addr & 0x3FFF];
        if (mask & (1<<6)) ret |= *(T*)&VRAM_G[addr & 0x3FFF];

        return ret;
    }

    template<typename T>
    T ReadVRAM_BBGExtPal(u32 addr) const noexcept
    {
        u32 mask = VRAMMap_BBGExtPal[(addr >> 13) & 0x3];

        T ret = 0;
        if (mask & (1<<7)) ret |= *(T*)&VRAM_H[addr & 0x7FFF];

        return ret;
    }

    template<typename T>
    T ReadVRAM_AOBJExtPal(u32 addr) const noexcept
    {
        u32 mask = VRAMMap_AOBJExtPal;

        T ret = 0;
        if (mask & (1<<4)) ret |= *(T*)&VRAM_F[addr & 0x1FFF];
        if (mask & (1<<5)) ret |= *(T*)&VRAM_G[addr & 0x1FFF];

        return ret;
    }

    template<typename T>
    T ReadVRAM_BOBJExtPal(u32 addr) const noexcept
    {
        u32 mask = VRAMMap_BOBJExtPal;

        T ret = 0;
        if (mask & (1<<8)) ret |= *(T*)&VRAM_I[addr & 0x1FFF];

        return ret;
    }

    template <u32 MappingGranularity, u32 Size>
    constexpr bool CopyLinearVRAM(u8* flat, const u32* mappings, NonStupidBitField<Size>& dirty, u64 (GPU::* const slowAccess)(u32) const noexcept) noexcept
    {
        const u32 VRAMBitsPerMapping = MappingGranularity / VRAMDirtyGranularity;

        bool change = false;

        typename NonStupidBitField<Size>::Iterator it = dirty.Begin();
        while (it != dirty.End())
        {
            u32 offset = *it * VRAMDirtyGranularity;
            u8* dst = flat + offset;
            u8* fastAccess = GetUniqueBankPtr(mappings[*it / VRAMBitsPerMapping], offset);
            if (fastAccess)
            {
                memcpy(dst, fastAccess, VRAMDirtyGranularity);
            }
            else
            {
                for (u32 i = 0; i < VRAMDirtyGranularity; i += 8)
                    *(u64*)&dst[i] = (this->*slowAccess)(offset + i);
            }
            change = true;
            it++;
        }
        return change;
    }

    u16* GetUniqueBankCBF(u32 mask, u32 offset);
    void VRAMCBFlagsSet(u32 bank, u32 block, u16 val);
    void VRAMCBFlagsClear(u32 bank, u32 block);
    void VRAMCBFlagsOr(u32 bank, u32 block, u16 val);
    void CheckCaptureStart();
    void CheckCaptureEnd();
    void SyncVRAMCaptureBlock(u32 block, bool write);
    void SyncAllVRAMCaptures();
    void GetCaptureInfo(int* info, u16** cbf, int len);

    void SetDispStatIRQ(int cpu, int num);

    bool UsesDisplayFIFO();
    void SampleDisplayFIFO(u32 offset, u32 num);

    bool VCountOverride = false;
    u16 NextVCount = 0;

    bool RunFIFO = false;

    u16 VMatch[2] {};

    std::unique_ptr<Renderer> Rend = nullptr;

    u16 VRAMCaptureBlockFlags[16];

    u16* VRAMCBF_ABG[0x20] {};
    u16* VRAMCBF_AOBJ[0x10] {};
    u16* VRAMCBF_BBG[0x8] {};
    u16* VRAMCBF_BOBJ[0x8] {};
};


struct RendererSettings
{
    // scale factor, for renderers that support upscaling
    int ScaleFactor;

    // whether to use separate threads for rendering
    bool Threaded;

    // whether to use hi-res vertex coordinates when applying upscaling
    bool HiresCoordinates;

    // "improved polygon splitting" (regular OpenGL renderer)
    bool BetterPolygons;

    // software renderer only: false = fast tile 3D (TileRenderer3D, when compiled in),
    // true = melonDS's accurate SoftRenderer3D. Other renderers ignore it.
    bool Accurate3D;
};

class Renderer
{
public:
    explicit Renderer(melonDS::GPU& gpu) : GPU(gpu), BackBuffer(0) {}
    virtual ~Renderer() {}
    virtual bool Init() = 0;
    virtual void Reset() = 0;
    virtual void Stop() = 0;

    virtual void PreSavestate() {}
    virtual void PostSavestate() {}

    virtual void SetRenderSettings(RendererSettings& settings) = 0;

    virtual void DrawScanline(u32 line) = 0;
    virtual void DrawSprites(u32 line) = 0;

    virtual void Start3DRendering() { Rend3D->RenderFrame(); }
    virtual void Finish3DRendering() { Rend3D->FinishRendering(); }
    virtual void Restart3DRendering() { Rend3D->RestartFrame(); }

    virtual void VBlank() = 0;
    virtual void VBlankEnd() = 0;

    virtual void AllocCapture(u32 bank, u32 start, u32 len) = 0;
    virtual void SyncVRAMCapture(u32 bank, u32 start, u32 len, bool complete) = 0;

    // a renderer may render to RAM buffers, or to something else (ie. OpenGL)
    // if the renderer uses RAM buffers, they should be 32-bit BGRA, 256x192 for each screen
    virtual bool GetFramebuffers(void** top, void** bottom) = 0;
    virtual void SwapBuffers() { BackBuffer ^= 1; }

    virtual bool NeedsShaderCompile() { return false; }
    virtual void ShaderCompileStep(int& current, int& count) {}

protected:
    melonDS::GPU& GPU;

    int BackBuffer;

    std::unique_ptr<Renderer2D> Rend2D_A;
    std::unique_ptr<Renderer2D> Rend2D_B;
    std::unique_ptr<Renderer3D> Rend3D;
};

}

#endif
