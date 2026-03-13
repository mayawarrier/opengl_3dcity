
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

// Disable copy and enable move semantics for a class with a single handle member.
#define HANDLE_CLASS(classname, handlename, handlenull)              \
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

// Disable copy semantics for a class.
#define DISABLE_COPY(classname)                      \
    classname(const classname&) = delete;            \
    classname& operator=(const classname&) = delete;

// Default move semantics for a class.
#define DEFAULT_MOVE(classname)                      \
    classname(classname&&) = default;                \
    classname& operator=(classname&&) = default;


namespace fs = std::filesystem;
namespace tim = std::chrono;

using clk = tim::steady_clock;
using uint = unsigned int;



template <typename T>
struct size
{
    T width, height;
};

using file_ptr = std::unique_ptr<std::FILE, int(*)(std::FILE*)>;

#define SAFE_FOPENA(fname, mode) file_ptr(std::fopen(fname, mode), std::fclose)

#if defined(_MSC_VER) || defined(__MINGW32__)
#define SAFE_FOPEN(fname, mode) file_ptr(::_wfopen(fname, CONCAT(L, mode)), std::fclose)
#else
#define SAFE_FOPEN(fname, mode) SAFE_FOPENA(fname, mode)
#endif

enum log_type
{
    LOG_ERROR,
    LOG_WARNING,
    LOG_MESSAGE
};

bool log_init(const char* logfile);
void logERROR(const char* fmt, ...);
void logWARNING(const char* fmt, ...);
void logMESSAGE(const char* fmt, ...);

#ifdef NDEBUG
#define logDEBUG(type, fmt, ...) ((void)0)
#define assert_msg(cond, fmt, ...) ((void)0)
#else
#define logDEBUG(type, fmt, ...)          \
    do {                                  \
        if (type == LOG_ERROR) {          \
            logERROR(fmt, __VA_ARGS__);   \
        } else if (type == LOG_WARNING) { \
            logWARNING(fmt, __VA_ARGS__); \
        } else if (type == LOG_MESSAGE) { \
            logMESSAGE(fmt, __VA_ARGS__); \
        }                                 \
    } while (0)

void do_assert_msg(const char* expr, const char* file, int line, const char* fmt, ...);
#define assert_msg(cond, fmt, ...)                                            \
    do {                                                                      \
        if (!(cond)) {                                                        \
            do_assert_msg(XSTR(cond), __FILE__, __LINE__, fmt, __VA_ARGS__);  \
        }                                                                     \
    } while (0)
#endif


struct buffer_overwrite_t {
    explicit buffer_overwrite_t() = default;
};
inline constexpr buffer_overwrite_t buffer_overwrite{};

template <typename T>
struct buffer
{
    std::unique_ptr<T[]> ptr;
    size_t size;

    buffer() : ptr(nullptr), size(0) {}

    buffer(size_t size) : 
        ptr(std::make_unique<T[]>(size)), size(size)
    {}

    buffer(size_t size, buffer_overwrite_t) :
        ptr(std::make_unique_for_overwrite<T[]>(size)), size(size)
    {}
    
    buffer(std::unique_ptr<T[]> ptr, size_t size) : 
        ptr(std::move(ptr)), size(size)
    {}

    std::span<T> span() const noexcept { 
        return { ptr.get(), size }; 
    }
};

// Read file contents.
bool read_file(const fs::path& path, std::unique_ptr<char[]>& out_data, size_t& out_size);
bool read_file(const fs::path& path, buffer<char>& out_data);


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
    if (!str.data()) {
        return false;
    }
    const char* beg = str.data();
    const char* end = str.data() + str.length();
    // skip whitespace
    while (beg != end && is_ws(*beg)) { beg++; }

    auto res = std::from_chars(beg, end, val);
    return res.ec == std::errc();
}

template <typename T>
bool parse_num(const char* str, T& val)
{
    if (!str) {
        return false;
    }
    return parse_num(std::string_view(str), val);
}

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

template <class CastAs, class R, class P>
inline auto dur_count(const tim::duration<R, P>& time) {
    return tim::duration_cast<CastAs>(time).count();
}

template <class R, class P>
inline std::string clock_dur_str(const tim::duration<R, P>& dur)
{
    if (dur > tim::seconds(1)) {
        return std::to_string(dur_count<tim::seconds>(dur)) + "s";
    } else if (dur > tim::milliseconds(1)) {
        return std::to_string(dur_count<tim::milliseconds>(dur)) + "ms";
    } else if (dur > tim::microseconds(1)) {
        return std::to_string(dur_count<tim::microseconds>(dur)) + "us";
    } else {
        return std::to_string(dur_count<tim::nanoseconds>(dur)) + "ns";
    }
}

inline void timeit(const char* msg, auto&& func)
{
    auto tbegin = clk::now();
    func();
    auto tend = clk::now();
    logMESSAGE("--- %s: %s", msg, clock_dur_str(tend - tbegin).c_str());
}

struct ebco_first_then_second_args_t {
    explicit ebco_first_then_second_args_t() = default;
};
struct ebco_second_args_t {
    explicit ebco_second_args_t() = default;
};

//
// Empty base-class optimization pair. 
// Stores two values, but if the first is an empty class, it takes no space.
// https://en.cppreference.com/w/cpp/language/ebo.html
//
template <class TMaybeEmpty, class U, bool = std::is_empty_v<TMaybeEmpty> && !std::is_final_v<TMaybeEmpty>>
class ebco_pair : private TMaybeEmpty
{
public:
    template <class FirstArg, class... SecondArgs>
    explicit ebco_pair(ebco_first_then_second_args_t, FirstArg&& f_arg, SecondArgs&&... s_args) :
        TMaybeEmpty(std::forward<FirstArg>(f_arg)), m_value(std::forward<SecondArgs>(s_args)...)
    {}

    template <class... SecondArgs>
    explicit ebco_pair(ebco_second_args_t, SecondArgs&&... s_args) :
        TMaybeEmpty(), m_value(std::forward<SecondArgs>(s_args)...)
    {}

    inline TMaybeEmpty& ebco_first() noexcept { return *this; }
    inline const TMaybeEmpty& ebco_first() const noexcept { return *this; }

    inline U& ebco_second() noexcept { return m_value; }
    inline const U& ebco_second() const noexcept { return m_value; }

private:
    U m_value;
};

template <class TMaybeEmpty, class U>
class ebco_pair<TMaybeEmpty, U, false>
{
public:
    template <class FirstArg, class... SecondArgs>
    explicit ebco_pair(ebco_first_then_second_args_t, FirstArg&& f_arg, SecondArgs&&... s_args) :
        m_first(std::forward<FirstArg>(f_arg)), m_value(std::forward<SecondArgs>(s_args)...)
    {}

    template <class... SecondArgs>
    explicit ebco_pair(ebco_second_args_t, SecondArgs&&... s_args) :
        m_first(), m_value(std::forward<SecondArgs>(s_args)...)
    {}
    
    inline TMaybeEmpty& ebco_first() noexcept { return m_first; }
    inline const TMaybeEmpty& ebco_first() const noexcept { return m_first; }

    inline U& ebco_second() noexcept { return m_value; }
    inline const U& ebco_second() const noexcept { return m_value; }

private:
    TMaybeEmpty m_first;
    U m_value;
};

// Check if a type is an instance of a template composed
// entirely of type parameters.
template <class, template <class...> class>
struct is_instance_of : std::false_type {};

template <class... Args, template <class...> class U>
struct is_instance_of<U<Args...>, U> : std::true_type {};

// Check if a type is an instance of a template composed
// entirely of non-type parameters.
template <class, template <auto...> class>
struct is_instance_of_nontype : std::false_type {};

template <auto... Args, template <auto...> class U>
struct is_instance_of_nontype<U<Args...>, U> : std::true_type {};

// Defer static_asserts until instantiation time
template <typename...>
struct deferred_false : std::false_type {};

template <typename... Ts>
constexpr bool deferred_false_v = deferred_false<Ts...>::value;

#endif
