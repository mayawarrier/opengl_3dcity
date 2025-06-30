
#include <cstdarg>
#include <array>
#include <debugbreak.h>

#ifdef _WIN32
    #include "win32.hpp"
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
    #include <unistd.h>
    #if defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
        #define HAS_POSIX_2001 1
    #endif
#endif

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

static file_ptr LOGFILE(nullptr, nullptr);
static bool LOG_COLOR_CONSOLE = false;

bool log_init(const char* logfile)
{
    LOGFILE = SAFE_FOPENA(logfile, "w");
    if (!LOGFILE) {
        // can't log an error, show a message box
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
            "Error", "Could not create log file", NULL);
        return false;
    }

#ifdef _WIN32
    LOG_COLOR_CONSOLE = win32_console_enable_colors();
    
    if (!win32_console_enable_utf8()) {
        logWARNING("Failed to enable UTF-8");
    };

#elif defined(HAS_POSIX_2001)
    LOG_COLOR_CONSOLE = posix_has_term_colors();
#endif
    return true;
}

static void log_to_stream(std::FILE* stream, const char* prefix, 
    const char* fmt, std::va_list vlist, bool flush = false)
{
PUSH_WARNINGS
IGNORE_WFORMAT_SECURITY
    if (prefix) {
        std::fputs(prefix, stream);
    }
    std::vfprintf(stream, fmt, vlist);
    std::fputs("\n", stream);

    if (flush) {
        std::fflush(stream);
    }
POP_WARNINGS
}

static inline void log(std::FILE* stream,
    const char* prefix, const char* prefix_color, 
    const char* fmt, std::va_list vlist)
{
    log_to_stream(LOGFILE.get(), prefix, fmt, vlist, true);
    log_to_stream(stream, LOG_COLOR_CONSOLE ? prefix_color : prefix, fmt, vlist);
}

#define GEN_LOG(stream, fmt, prefix, prefix_color)  \
do {                                                \
    std::va_list vlist;                             \
    va_start(vlist, fmt);                           \
    log(stream, prefix, prefix_color, fmt, vlist);  \
    va_end(vlist);                                  \
} while(0)


void logERROR(const char* fmt, ...) {
    GEN_LOG(stderr, fmt, "Error: ", "\033[1;31mError:\033[0m ");
}

void logWARNING(const char* fmt, ...) {
    GEN_LOG(stderr, fmt, "Warning: ", "\033[1;33mWarning:\033[0m ");
}

void logMESSAGE(const char* fmt, ...) {
    GEN_LOG(stdout, fmt, nullptr, nullptr);
}

#ifndef NDEBUG
void do_assert_msg(const char* expr, const char* file, int line, const char* fmt, ...) 
{
    auto print_assert = [](const char* fmt, ...) {
        GEN_LOG(stderr, fmt, "Assert failed: ", "\033[1;31mAssert failed:\033[0m ");
    };
    print_assert("%s, %s (line %d)", expr, file, line);
    // additional message
    GEN_LOG(stderr, fmt, nullptr, nullptr);

    debug_break();
}
#endif

bool read_file(const fs::path& path, std::unique_ptr<char[]>& filedata, size_t& filesize)
{
    file_ptr file = SAFE_FOPEN(path.c_str(), "rb");
    if (!file) {
        logERROR("Could not open file %s", path.string().c_str());
        return false;
    }
    filesize = size_t(fs::file_size(path));
    filedata = std::make_unique<char[]>(filesize);

    if (std::fread(filedata.get(), 1, filesize, file.get()) != filesize) {
        logERROR("Could not read from file %s", path.string().c_str());
        return false;
    }
    return true;
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
