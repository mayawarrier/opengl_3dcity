
#ifndef UTILS_HPP
#define UTILS_HPP

#include <cstdio>
#include <utility>
#include <algorithm>
#include <filesystem>
#include <charconv>
#include <chrono>
#include <span>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <optional>
#include <initializer_list>
#include <type_traits>

#include <SDL.h>

#define CONCAT(x, y) x##y
#define STR(a) #a
#define XSTR(a) STR(a)

#define NS_PER_MS 1000000

// Bogus warnings
#ifdef __clang__
#define PUSH_WARNINGS _Pragma("clang diagnostic push")
#define POP_WARNINGS  _Pragma("clang diagnostic pop")
#define IGNORE_WFORMAT_SECURITY \
    _Pragma("clang diagnostic ignored \"-Wformat-security\"")
#elif defined(__GNUC__)
#define PUSH_WARNINGS _Pragma("GCC diagnostic push")
#define POP_WARNINGS  _Pragma("GCC diagnostic pop")
#define IGNORE_WFORMAT_SECURITY \
    _Pragma("GCC diagnostic ignored \"-Wformat-security\"")
#else
#define PUSH_WARNINGS
#define POP_WARNINGS
#define IGNORE_WFORMAT_SECURITY
#endif

namespace fs = std::filesystem;
namespace tim = std::chrono;

using clk = tim::steady_clock;
using uint = unsigned int;

struct size
{
    int width, height;
};

// Defer static_asserts until instantiation time
template <typename...>
struct deferred_false : std::false_type {};

#define MOVE_ONLY_CLASS(classname, handlename, handlenull)           \
    classname(const classname&) = delete;                            \
    classname& operator=(const classname&) = delete;                 \
                                                                     \
    classname(classname&& rhs) noexcept :                            \
        handlename(std::exchange(rhs.handlename, handlenull))        \
    {}                                                               \
    classname& operator=(classname&& rhs) noexcept {                 \
        if (this != &rhs) {                                          \
            handlename = std::exchange(rhs.handlename, handlenull);  \
        }                                                            \
        return *this;                                                \
    }                                                                \
    bool ok() const noexcept { return handlename != handlenull; }


// this only works if NDEBUG is defined in Release mode
// (default for CMake)
constexpr bool is_debug()
{
#ifdef NDEBUG
    return false;
#else
    return true;
#endif
}

using file_ptr = std::unique_ptr<std::FILE, int(*)(std::FILE*)>;

#define SAFE_FOPENA(fname, mode) file_ptr(std::fopen(fname, mode), std::fclose)

#if defined(_MSC_VER) || defined(__MINGW32__)
#define SAFE_FOPEN(fname, mode) file_ptr(::_wfopen(fname, CONCAT(L, mode)), std::fclose)
#else
#define SAFE_FOPEN(fname, mode) SAFE_FOPENA(fname, mode)
#endif


bool log_init(const char* logfile);

void logERROR(const char* fmt, ...);
void logWARNING(const char* fmt, ...);
void logMESSAGE(const char* fmt, ...);

#ifdef NDEBUG
#define assert_msg(cond, fmt, ...) ((void)0)
#else
void do_assert_msg(const char* expr, const char* file, int line, const char* fmt, ...);

#define assert_msg(cond, fmt, ...)                                            \
    do {                                                                      \
        if (!(cond)) {                                                        \
            do_assert_msg(XSTR(cond), __FILE__, __LINE__, fmt, __VA_ARGS__);  \
        }                                                                     \
    } while (0)
#endif


template <typename T>
struct dynarray
{
    std::unique_ptr<T[]> ptr;
    size_t size;

    dynarray() : ptr(nullptr), size(0) {}

    dynarray(size_t size) : 
        ptr(std::make_unique<T[]>(size)), size(size)
    {}
    
    dynarray(std::unique_ptr<T[]>&& ptr, size_t size) : 
        ptr(std::move(ptr)), size(size)
    {}

    std::span<T> span() const noexcept { 
        return { ptr.get(), size }; 
    }
};

// Read file contents.
bool read_file(const fs::path& path, std::unique_ptr<char[]>& out_data, size_t& out_size);
bool read_file(const fs::path& path, dynarray<char>& out_data);


// this has good codegen
template <typename T>
inline void set_bit(T* ptr, int bit, bool val)
{
    *ptr = (*ptr & ~(0x1 << bit)) | (val << bit);
}

template <typename T>
inline bool get_bit(T word, int bit)
{
    return (word & (0x1 << bit)) != 0;
}

constexpr SDL_Point sdl_ptadd(SDL_Point a, SDL_Point b)
{
    return { a.x + b.x, a.y + b.y };
}
constexpr SDL_Point sdl_ptsub(SDL_Point a, SDL_Point b)
{
    return { a.x - b.x, a.y - b.y };
}

constexpr float wrap_angle(float angle)
{
    if (angle >= 360.f) {
        angle -= 360.f;
    }
    else if (angle <= 0) {
        angle += 360.f;
    }
    return angle;
}

template <typename = void>
struct ws_lut {
    static constexpr uint8_t lut[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
};

template <typename T>
constexpr uint8_t ws_lut<T>::lut[];

// Fast whitespace check, C-locale
constexpr bool is_ws(char c) {
    return ws_lut<>::lut[static_cast<uint8_t>(c)];
}

template <typename T>
bool parse_num(std::string_view str, T& val)
{
    const char* beg = str.data();
    const char* end = str.data() + str.length();
    // skip whitespace
    while (beg != end && is_ws(*beg)) { beg++; }

    auto res = std::from_chars(beg, end, val);
    return res.ec == std::errc();
}

// sorta replacement for C++26 operator+(string, string_view) 
inline std::string concat_sv(
    std::initializer_list<std::string_view> list)
{
    std::string ret;
    for (auto elem : list) {
        ret += elem;
    }
    return ret;
}

struct inireader
{
    inireader(const fs::path& path);

    bool ok() const { return m_ok; }

    const char* path_cstr() const { return m_pathstr.c_str(); }

    std::optional<std::string> get_string(std::string_view section, std::string_view key)
    {
        auto str = get_value_sv(section, key);
        if (!str.data()) {
            return {};
        }
        return std::string(str);
    }

    template <typename T> requires std::is_arithmetic_v<T>
    std::optional<T> get_num(std::string_view section, std::string_view key)
    {
        T value;
        auto str = get_value_sv(section, key);
        if (!str.data() || !parse_num(str, value)) {
            return {};
        }
        return value;
    }

private:
    // not null-terminated!
    std::string_view get_value_sv(std::string_view section, std::string_view key)
    {
        auto sectitr = m_map.find(section);
        if (sectitr != m_map.end())
        {
            auto& entries = sectitr->second;
            auto valueitr = entries.find(key);
            if (valueitr != entries.end()) {
                return valueitr->second;
            }
        }
        return {};
    }

private:
    using section_t = std::unordered_map<std::string_view, std::string_view>;
    using map_t = std::unordered_map<std::string_view, section_t>;

    map_t m_map;
    std::string m_pathstr;
    std::unique_ptr<char[]> m_filedata;
    bool m_ok;
};

struct iniwriter
{
    iniwriter(const fs::path& path) :
        m_file(SAFE_FOPEN(path.c_str(), "wb")),
        m_pathstr(path.string()),
        m_ok(false)
    {
        if (!m_file) {
            logERROR("Could not open file %s", m_pathstr.c_str());
            return;
        }
        m_ok = true;
    }

    bool ok() const { return m_ok; }

    bool write_section(std::string_view name)
    {
        return write_str(concat_sv({ "[", name, "]\n" }));
    }

    bool write_keyvalue(std::string_view key, std::string_view value)
    {
        return write_str(concat_sv({ key, " = ", value, "\n" }));
    }

    bool flush() { return std::fflush(m_file.get()) == 0; }

private:
    bool write_str(const std::string& str)
    {
        if (std::fwrite(str.c_str(), 1, str.length(), m_file.get()) != str.length()) {
            logERROR("Could not write to file %s", m_pathstr.c_str());
            return false;
        }
        return true;
    }

private:
    file_ptr m_file;
    std::string m_pathstr;
    bool m_ok;
};

#endif
