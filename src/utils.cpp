
#ifdef _WIN32
    #include "win32.hpp"

#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
    // https://lwn.net/Articles/592652/
    // https://man7.org/linux/man-pages/man3/fseeko.3.html
    // note: musl does not make ftello available without a feature macro,
    // even if _FILE_OFFSET_BITS=64 is defined
    #define _GNU_SOURCE 1
    #define _FILE_OFFSET_BITS 64
    
    #include <unistd.h>
    #if defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
        #define HAS_POSIX_2001 1
    #endif
#endif

#include <cstdarg>
#include <array>
#include <debugbreak.h>

#include "utils.hpp"

#ifdef HAS_POSIX_2001
static bool posix_has_term_colors()
{
    if (!isatty(STDOUT_FILENO) || !isatty(STDERR_FILENO)) {
        return false;
    }
    const char* term = getenv("TERM");
    if (!term) {
        return false;
    }
    return std::strcmp(term, "dumb") != 0;
}
#endif

struct global_logger
{
    file_ptr file;
    bool has_colors;
};
static global_logger LOG{ {nullptr, nullptr}, false };

bool log_init(const char* logfile)
{
    // note: binary mode
    LOG.file = SAFE_FOPENA(logfile, "wb");
    if (!LOG.file) {
        // can't log an error, show a message box
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
            "Error", "Could not create log file", NULL);
        return false;
    }

#ifdef _WIN32
    LOG.has_colors = win32_console_enable_colors();  
    if (!win32_console_enable_utf8()) {
        logWARNING("Failed to enable UTF-8");
    };
#elif defined(HAS_POSIX_2001)
    LOG.has_colors = posix_has_term_colors();
#endif
    return true;
}

std::int64_t log_size()
{
#if defined(_WIN32)
    return _ftelli64(LOG.file.get());
#elif defined(HAS_POSIX_2001)
    return ::ftello(LOG.file.get());
#else
    return std::ftell(LOG.file.get());
#endif
}

static void log_to_stream(
    std::FILE* stream, const char* fmt, std::va_list vlist,
    const char* prefix = nullptr, const char* color_seq = nullptr, 
    bool flush = false)
{
PUSH_WARNINGS
IGNORE_WFORMAT_SECURITY
    if (color_seq) {
        std::fputs(color_seq, stream);
    }
    if (prefix) {
        std::fputs(prefix, stream);
    }
    std::vfprintf(stream, fmt, vlist);

    if (color_seq) {
        std::fputs("\033[0m\n", stream);
    } else {
        std::fputs("\n", stream);
    }
    if (flush) {
        std::fflush(stream);
    }
POP_WARNINGS
}

#define GEN_LOG(stream, fmt, prefix, color_seq)                                             \
do {                                                                                        \
    std::va_list vlist;                                                                     \
    va_start(vlist, fmt);                                                                   \
    log_to_stream(LOG.file.get(), fmt, vlist, prefix, nullptr, true);                       \
    log_to_stream(stream, fmt, vlist, prefix, LOG.has_colors ? color_seq : nullptr, false); \
    va_end(vlist);                                                                          \
} while(0)

void logERROR(const char* fmt, ...) {
    GEN_LOG(stderr, fmt, "Error: ", "\033[1;31m");
}

void logWARNING(const char* fmt, ...) {
    GEN_LOG(stderr, fmt, "Warning: ", "\033[1;33m");
}

void logMESSAGE(const char* fmt, ...) {
    GEN_LOG(stdout, fmt, nullptr, nullptr);
}

void log_colorMESSAGE(const char* color_seq, const char* fmt, ...) {
    GEN_LOG(stdout, fmt, nullptr, color_seq);
}

#ifndef NDEBUG
void do_assert_msg(const char* expr, const char* file, int line, const char* fmt, ...) 
{
    auto print_assert = [](const char* fmt, ...) {
        GEN_LOG(stderr, fmt, "Assert failed: ", "\033[1;31m");
    };
    print_assert("%s, %s (line %d)", expr, file, line);
    // additional message
    GEN_LOG(stderr, fmt, nullptr, nullptr);

    debug_break();
}
#endif

bool read_file(const fs::path& path, std::unique_ptr<char[]>& out_data, size_t& out_size)
{
    file_ptr file = SAFE_FOPEN(path.c_str(), "rb");
    if (!file) {
        logERROR("Could not open file %s", path.string().c_str());
        return false;
    }

    uintmax_t filesize = fs::file_size(path);
    if (!std::in_range<size_t>(filesize)) {
        logERROR("File %s is too large", path.string().c_str());
        return false;
    }

    auto bufsize = size_t(filesize);
    auto buf = std::make_unique<char[]>(bufsize);

    if (std::fread(buf.get(), 1, bufsize, file.get()) != bufsize) {
        logERROR("Could not read from file %s", path.string().c_str());
        return false;
    }
    out_data = std::move(buf);
    out_size = bufsize;
    return true;
}

bool read_file(const fs::path& path, buffer<char>& out_data)
{
    return read_file(path, out_data.ptr, out_data.size);
}

static std::string_view trim(std::string_view str)
{
    const char* beg = str.data();
    const char* end = str.data() + str.length();

    while (beg != end && is_ws(*beg)) { ++beg; }
    while (end != beg && is_ws(*(end - 1))) { --end; }

    return { beg, end };
}

static bool extract_str(std::string_view& src, std::string_view& str, char delim)
{
    if (src.size() == 0) { return false; }

    size_t i = src.find(delim);
    size_t endpos = (i != src.npos) ? i : src.size();
    size_t nread = (i != src.npos) ? i + 1 : src.size(); // skip delim

    str = trim({ src.data(), endpos });
    src.remove_prefix(nread);
    return true;
}

static bool getline_sv(std::string_view& src, std::string_view& line)
{
    return extract_str(src, line, '\n');
}

inireader::inireader(const fs::path& path) :
    m_pathstr(path.string()), m_ok(false)
{
    size_t filesize;
    if (!read_file(path, m_filedata, filesize)) {
        return;
    }

    int lineno = 1;
    std::string_view line, section;
    std::string_view filedata_sv(m_filedata.get(), filesize);

    while (getline_sv(filedata_sv, line))
    {
        if (line.empty()) {
            continue;
        }
        if (line.starts_with('[') && line.ends_with(']')) {
            section = line.substr(1, line.length() - 2);
        }
        else {
            std::string_view key, value;
            if (!extract_str(line, key, '=') || key.empty() ||
                !extract_str(line, value, '\n') || value.empty()) {
                logERROR("%s: Invalid entry on line %d", path_cstr(), lineno);
                return;
            }

            m_map[section][key] = value;
        }
        lineno++;
    }

    m_ok = true;
}
