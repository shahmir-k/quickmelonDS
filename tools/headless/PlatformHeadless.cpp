/*
    liteDS-v2 headless harness - implementation of the melonDS Platform
    interface (src/Platform.h) for a headless POSIX host.

    Scope for Unit 0:
      - Real: file IO (stdio), time, threads/mutex/semaphores (std::thread etc),
        NDS/GBA save + firmware write-back into a --data-dir.
      - Stubs (noted in the report): camera, mic, LAN (Net_*), local
        multiplayer (MP_*), LED/rumble/motion addons, AAC decoding,
        dynamic-library loading.

    Cribbed from src/frontend/qt_sdl/Platform.cpp but with no Qt dependency.
*/

#include "Platform.h"
#include "PlatformHeadless.h"
#include "MPInterface.h"

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <sys/stat.h>
#include <sys/types.h>
#include <dlfcn.h>

namespace HeadlessHost
{
static std::string s_DataDir = "./headless-data";

static void EnsureDir(const std::string& dir)
{
    // best-effort recursive mkdir
    std::string path;
    for (size_t i = 0; i <= dir.size(); i++)
    {
        if (i == dir.size() || dir[i] == '/')
        {
            if (!path.empty())
                mkdir(path.c_str(), 0755);
        }
        if (i < dir.size())
            path += dir[i];
    }
}

void SetDataDir(const std::string& dir)
{
    s_DataDir = dir;
    EnsureDir(s_DataDir);
}

const std::string& GetDataDir()
{
    return s_DataDir;
}
}

namespace melonDS::Platform
{

void SignalStop(StopReason reason, void* userdata)
{
    (void)reason; (void)userdata;
}

// ---------------------------------------------------------------------------
// File IO (stdio-backed; FileHandle* is an opaque FILE*)
// ---------------------------------------------------------------------------

static FILE* ToFile(FileHandle* f) { return reinterpret_cast<FILE*>(f); }

std::string GetLocalFilePath(const std::string& filename)
{
    // Absolute paths pass through; relative ones resolve under the data dir.
    if (!filename.empty() && filename[0] == '/')
        return filename;
    return HeadlessHost::GetDataDir() + "/" + filename;
}

static const char* ModeString(FileMode mode)
{
    if (mode & FileMode::Append)
        return (mode & FileMode::Read) ? "a+b" : "ab";

    bool read = mode & FileMode::Read;
    bool write = mode & FileMode::Write;

    if (read && write)
    {
        // Preserve => don't truncate an existing file.
        if (mode & FileMode::Preserve)
            return "r+b";
        return "w+b";
    }
    if (write)
        return "wb";
    return "rb";
}

FileHandle* OpenFile(const std::string& path, FileMode mode)
{
    if ((mode & (FileMode::Read | FileMode::Write)) == 0)
        return nullptr;

    // NoCreate/Preserve for write: only open if it already exists.
    if ((mode & FileMode::Write) && (mode & FileMode::NoCreate))
    {
        FILE* chk = fopen(path.c_str(), "rb");
        if (!chk) return nullptr;
        fclose(chk);
    }

    const char* m = ModeString(mode);
    FILE* f = fopen(path.c_str(), m);

    // "r+b" fails if the file doesn't exist; for ReadWrite without NoCreate,
    // fall back to creating it.
    if (!f && (mode & FileMode::Write) && (mode & FileMode::Read) &&
        (mode & FileMode::Preserve) && !(mode & FileMode::NoCreate))
    {
        f = fopen(path.c_str(), "w+b");
    }

    return reinterpret_cast<FileHandle*>(f);
}

FileHandle* OpenLocalFile(const std::string& path, FileMode mode)
{
    return OpenFile(GetLocalFilePath(path), mode);
}

bool FileExists(const std::string& name)
{
    FILE* f = fopen(name.c_str(), "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

bool LocalFileExists(const std::string& name)
{
    return FileExists(GetLocalFilePath(name));
}

bool CheckFileWritable(const std::string& filepath)
{
    FILE* f = fopen(filepath.c_str(), "abe");
    if (!f) return false;
    fclose(f);
    return true;
}

bool CheckLocalFileWritable(const std::string& name)
{
    return CheckFileWritable(GetLocalFilePath(name));
}

bool CloseFile(FileHandle* file)
{
    if (!file) return false;
    return fclose(ToFile(file)) == 0;
}

bool IsEndOfFile(FileHandle* file)
{
    return feof(ToFile(file)) != 0;
}

bool FileReadLine(char* str, int count, FileHandle* file)
{
    return fgets(str, count, ToFile(file)) != nullptr;
}

u64 FilePosition(FileHandle* file)
{
    return (u64)ftell(ToFile(file));
}

bool FileSeek(FileHandle* file, s64 offset, FileSeekOrigin origin)
{
    int whence = SEEK_SET;
    switch (origin)
    {
    case FileSeekOrigin::Start:   whence = SEEK_SET; break;
    case FileSeekOrigin::Current: whence = SEEK_CUR; break;
    case FileSeekOrigin::End:     whence = SEEK_END; break;
    }
    return fseek(ToFile(file), (long)offset, whence) == 0;
}

void FileRewind(FileHandle* file)
{
    rewind(ToFile(file));
}

u64 FileRead(void* data, u64 size, u64 count, FileHandle* file)
{
    return (u64)fread(data, size, count, ToFile(file));
}

bool FileFlush(FileHandle* file)
{
    return fflush(ToFile(file)) == 0;
}

u64 FileWrite(const void* data, u64 size, u64 count, FileHandle* file)
{
    return (u64)fwrite(data, size, count, ToFile(file));
}

u64 FileWriteFormatted(FileHandle* file, const char* fmt, ...)
{
    if (!fmt) return 0;
    va_list args;
    va_start(args, fmt);
    int ret = vfprintf(ToFile(file), fmt, args);
    va_end(args);
    return ret < 0 ? 0 : (u64)ret;
}

u64 FileLength(FileHandle* file)
{
    FILE* f = ToFile(file);
    long pos = ftell(f);
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, pos, SEEK_SET);
    return len < 0 ? 0 : (u64)len;
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

void Log(LogLevel level, const char* fmt, ...)
{
    if (!fmt) return;
    // Suppress Debug-level spam unless LITEDS_VERBOSE is set.
    if (level == LogLevel::Debug && !getenv("LITEDS_VERBOSE"))
        return;
    static const char* tags[] = { "[D] ", "[I] ", "[W] ", "[E] " };
    fputs(tags[level], stderr);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
}

// ---------------------------------------------------------------------------
// Threads / mutexes / semaphores (std:: backed)
// ---------------------------------------------------------------------------

struct Thread { std::thread t; };

Thread* Thread_Create(std::function<void()> func)
{
    Thread* th = new Thread();
    th->t = std::thread(std::move(func));
    return th;
}

void Thread_Free(Thread* thread)
{
    if (!thread) return;
    if (thread->t.joinable())
        thread->t.detach();
    delete thread;
}

void Thread_Wait(Thread* thread)
{
    if (thread && thread->t.joinable())
        thread->t.join();
}

struct Semaphore
{
    std::mutex mtx;
    std::condition_variable cv;
    int count = 0;
};

Semaphore* Semaphore_Create() { return new Semaphore(); }

void Semaphore_Free(Semaphore* sema) { delete sema; }

void Semaphore_Reset(Semaphore* sema)
{
    std::lock_guard<std::mutex> lk(sema->mtx);
    sema->count = 0;
}

void Semaphore_Wait(Semaphore* sema)
{
    std::unique_lock<std::mutex> lk(sema->mtx);
    sema->cv.wait(lk, [&]{ return sema->count > 0; });
    sema->count--;
}

bool Semaphore_TryWait(Semaphore* sema, int timeout_ms)
{
    std::unique_lock<std::mutex> lk(sema->mtx);
    if (timeout_ms <= 0)
    {
        if (sema->count <= 0) return false;
    }
    else
    {
        if (!sema->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                               [&]{ return sema->count > 0; }))
            return false;
    }
    sema->count--;
    return true;
}

void Semaphore_Post(Semaphore* sema, int count)
{
    std::lock_guard<std::mutex> lk(sema->mtx);
    sema->count += count;
    sema->cv.notify_all();
}

struct Mutex { std::mutex m; };

Mutex* Mutex_Create() { return new Mutex(); }
void Mutex_Free(Mutex* mutex) { delete mutex; }
void Mutex_Lock(Mutex* mutex) { mutex->m.lock(); }
void Mutex_Unlock(Mutex* mutex) { mutex->m.unlock(); }
bool Mutex_TryLock(Mutex* mutex) { return mutex->m.try_lock(); }

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

static std::chrono::steady_clock::time_point s_Start = std::chrono::steady_clock::now();

void Sleep(u64 usecs)
{
    std::this_thread::sleep_for(std::chrono::microseconds(usecs));
}

u64 GetMSCount()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - s_Start).count();
}

u64 GetUSCount()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - s_Start).count();
}

// ---------------------------------------------------------------------------
// Save / firmware write-back (into the data dir)
// ---------------------------------------------------------------------------

static void WriteBufferToLocal(const std::string& name, const u8* data, u32 len)
{
    std::string path = GetLocalFilePath(name);
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return;
    fwrite(data, 1, len, f);
    fclose(f);
}

// Resolve the per-instance save stem from the NDS userdata pointer. Falls back
// to "headless" when no InstanceUserData was supplied (benchmark path).
static std::string SavePrefix(void* userdata)
{
    if (userdata)
        return reinterpret_cast<HeadlessHost::InstanceUserData*>(userdata)->savePrefix;
    return "headless";
}

void WriteNDSSave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)writeoffset; (void)writelen;
    WriteBufferToLocal(SavePrefix(userdata) + ".sav", savedata, savelen);
}

void WriteGBASave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)writeoffset; (void)writelen;
    WriteBufferToLocal(SavePrefix(userdata) + "-gba.sav", savedata, savelen);
}

void WriteFirmware(const Firmware& firmware, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)firmware; (void)writeoffset; (void)writelen; (void)userdata;
    // Generated firmware is transient in headless mode; nothing to persist.
}

void WriteDateTime(int year, int month, int day, int hour, int minute, int second, void* userdata)
{
    (void)year; (void)month; (void)day; (void)hour; (void)minute; (void)second; (void)userdata;
}

// ---------------------------------------------------------------------------
// Local multiplayer (MP_*) - route to the shared in-process MPInterface
// (a LocalMP, installed by the MP harness via MPInterface::Set(Local)). The
// instance id comes from the per-NDS InstanceUserData; a null userdata (the
// single-instance benchmark path) is instance 0. Until the harness installs a
// LocalMP, MPInterface::Current is a no-op dummy, so normal single-instance runs
// behave exactly as before. Mirrors src/frontend/qt_sdl/Platform.cpp:460-512.
// ---------------------------------------------------------------------------

static int MPInst(void* userdata)
{
    return userdata ? reinterpret_cast<HeadlessHost::InstanceUserData*>(userdata)->instanceID : 0;
}

void MP_Begin(void* u) { MPInterface::Get().Begin(MPInst(u)); }
void MP_End(void* u)   { MPInterface::Get().End(MPInst(u)); }
int MP_SendPacket(u8* d, int l, u64 t, void* u) { return MPInterface::Get().SendPacket(MPInst(u), d, l, t); }
int MP_RecvPacket(u8* d, u64* t, void* u)       { return MPInterface::Get().RecvPacket(MPInst(u), d, t); }
int MP_SendCmd(u8* d, int l, u64 t, void* u)    { return MPInterface::Get().SendCmd(MPInst(u), d, l, t); }
int MP_SendReply(u8* d, int l, u64 t, u16 aid, void* u) { return MPInterface::Get().SendReply(MPInst(u), d, l, t, aid); }
int MP_SendAck(u8* d, int l, u64 t, void* u)    { return MPInterface::Get().SendAck(MPInst(u), d, l, t); }
int MP_RecvHostPacket(u8* d, u64* t, void* u)   { return MPInterface::Get().RecvHostPacket(MPInst(u), d, t); }
u16 MP_RecvReplies(u8* d, u64 t, u16 aidmask, void* u) { return MPInterface::Get().RecvReplies(MPInst(u), d, t, aidmask); }

// ---------------------------------------------------------------------------
// Networking (Net_*) - STUBS
// ---------------------------------------------------------------------------

int Net_SendPacket(u8*, int, void*) { return 0; }
int Net_RecvPacket(u8*, void*) { return 0; }

// ---------------------------------------------------------------------------
// Camera - STUBS
// ---------------------------------------------------------------------------

void Camera_Start(int, void*) {}
void Camera_Stop(int, void*) {}
void Camera_CaptureFrame(int, u32* frame, int width, int height, bool, void*)
{
    if (frame)
        memset(frame, 0, (size_t)width * height * sizeof(u32));
}

// ---------------------------------------------------------------------------
// Microphone - STUBS (returns silence)
// ---------------------------------------------------------------------------

void Mic_Start(void*) {}
void Mic_Stop(void*) {}
int Mic_ReadInput(s16*, int, void*) { return 0; }

// ---------------------------------------------------------------------------
// AAC decoding (DSi DSP HLE) - STUBS
// ---------------------------------------------------------------------------

AACDecoder* AAC_Init() { return nullptr; }
void AAC_DeInit(AACDecoder*) {}
bool AAC_Configure(AACDecoder*, int, int) { return false; }
bool AAC_DecodeFrame(AACDecoder*, const void*, int, void*, int) { return false; }

// ---------------------------------------------------------------------------
// Addon inputs (guitar grip / rumble / motion) - STUBS
// ---------------------------------------------------------------------------

bool Addon_KeyDown(KeyType, void*) { return false; }
void Addon_RumbleStart(u32, void*) {}
void Addon_RumbleStop(void*) {}
float Addon_MotionQuery(MotionQueryType, void*) { return 0.0f; }

// ---------------------------------------------------------------------------
// Dynamic library loading (dlopen wrappers)
// ---------------------------------------------------------------------------

struct DynamicLibrary;

DynamicLibrary* DynamicLibrary_Load(const char* lib)
{
    return reinterpret_cast<DynamicLibrary*>(dlopen(lib, RTLD_NOW | RTLD_LOCAL));
}

void DynamicLibrary_Unload(DynamicLibrary* lib)
{
    if (lib) dlclose(reinterpret_cast<void*>(lib));
}

void* DynamicLibrary_LoadFunction(DynamicLibrary* lib, const char* name)
{
    if (!lib) return nullptr;
    return dlsym(reinterpret_cast<void*>(lib), name);
}

} // namespace melonDS::Platform
