// A cross-build live handoff may skip a JIT cache payload while loading a
// checkpoint. Finishing that read must not rewrite the transmitted bytes.
#include "Platform.h"
#include "Savestate.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

namespace melonDS::Platform
{
void Log(LogLevel, const char*, ...) {}
}

int main()
{
    using namespace melonDS;

    std::array<u8, 256> storage{};
    Savestate writer(storage.data(), static_cast<u32>(storage.size()), true);
    writer.Section("NDSG");
    u32 gameValue = 0x12345678;
    writer.Var32(&gameValue);
    writer.Section("JITS");
    u64 build = 0x0123456789abcdef;
    u32 config = 0x3201e;
    u32 valid = 1;
    writer.Var64(&build);
    writer.Var32(&config);
    writer.Var32(&valid);
    std::array<u8, 32> cachePayload{};
    cachePayload.fill(0xa5);
    writer.VarArray(cachePayload.data(), static_cast<u32>(cachePayload.size()));
    writer.Finish();
    assert(!writer.Error);

    const u32 length = writer.Length();
    std::vector<u8> checkpoint(storage.begin(), storage.begin() + length);
    u32 storedLength = 0;
    memcpy(&storedLength, checkpoint.data() + 8, sizeof(storedLength));
    assert(storedLength == length);
    const std::vector<u8> original = checkpoint;

    Savestate reader(checkpoint.data(), static_cast<u32>(checkpoint.size()), false);
    assert(!reader.Error);
    reader.Section("NDSG");
    u32 loadedGameValue = 0;
    reader.Var32(&loadedGameValue);
    assert(loadedGameValue == gameValue);
    reader.Section("JITS");
    u64 loadedBuild = 0;
    u32 loadedConfig = 0;
    u32 loadedValid = 0;
    reader.Var64(&loadedBuild);
    reader.Var32(&loadedConfig);
    reader.Var32(&loadedValid);
    assert(loadedBuild == build && loadedConfig == config && loadedValid == valid);
    assert(reader.Length() < checkpoint.size()); // another build skips the JIT cache
    reader.Finish();
    assert(!reader.Error);
    assert(checkpoint == original);

    std::puts("savestate-readonly-test: PASS");
}
