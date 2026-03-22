#pragma once

/**
 * @file  logging.hpp
 * @brief Header-only, thread-safe C++ logging library (C++14 compatible).
 */

#include <chrono>
#include "logging.h"
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <iostream>

namespace ncmv
{

// ─────────────────────────────────────────────────────────────────────────────
// Log level
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Severity levels. Ordered from least to most severe.
 * Use log_level::none as the minimum level to silence all output.
 */
enum class log_level : int
{
    debug   = 0,
    info    = 1,
    warning = 2,
    error   = 3,
    none    = 4  ///< Sentinel — disables all output when set as minimum level.
};

// ─────────────────────────────────────────────────────────────────────────────
// Interfaces
// ─────────────────────────────────────────────────────────────────────────────

/** Write a pre-formatted record to an output destination. */
class i_log_sink
{
public:
    virtual ~i_log_sink() = default;
    virtual void write(const std::string& record) = 0;
};

/** Convert raw log data into a formatted string. */
class i_log_formatter
{
public:
    virtual ~i_log_formatter() = default;
    virtual std::string format(log_level          level,
                               long long          elapsed_ms,
                               const std::string& message) const = 0;
};

/** Decide whether a record at the given level should be emitted. */
class i_log_filter {
public:
    virtual ~i_log_filter() = default;
    virtual bool is_enabled(log_level level) const noexcept = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Default implementations  (SRP — one responsibility each)
// ─────────────────────────────────────────────────────────────────────────────

/** Routes records to any std::ostream.  Default stream: std::cout. */
class ostream_sink final : public i_log_sink
{
public:
    explicit ostream_sink(std::ostream& out = std::cout) : out_(out) {}
    
    void write(const std::string& record) override
    {
        out_ << record << '\n';
    }

private:
    std::ostream& out_;
};

// ─── internal helpers ────────────────────────────────────────────────────────
namespace detail
{

inline const char* level_label(log_level lv) noexcept
{
    switch (lv) {
        case log_level::debug:   return "DEBUG";
        case log_level::info:    return "INFO ";
        case log_level::warning: return "WARN ";
        case log_level::error:   return "ERROR";
        default:                 return "?????";
    }
}

// Base case — nothing left to append.
inline void build_message(std::ostringstream& /*oss*/) noexcept {}

// Recursive case — stream the first argument, then recurse on the rest.
template <typename T, typename... Rest>
void build_message(std::ostringstream& oss, T&& first, Rest&&... rest)
{
    oss << std::forward<T>(first);
    build_message(oss, std::forward<Rest>(rest)...);
}

/** Concatenate any number of heterogeneous arguments into a single string. */
template <typename... Args>
std::string make_message(Args&&... args)
{
    std::ostringstream oss;
    build_message(oss, std::forward<Args>(args)...);
    return oss.str();
}

} // namespace detail
// ─────────────────────────────────────────────────────────────────────────────

/** Formats records as:  [<elapsed_ms>ms][<LEVEL>] <message> */
class default_formatter final : public i_log_formatter {
public:
    std::string format(log_level          level,
                       long long          elapsed_ms,
                       const std::string& message) const override {
        std::ostringstream oss;
        oss << '[' << elapsed_ms << "ms]["
            << detail::level_label(level) << "] "
            << message;
        return oss.str();
    }
};

/** Passes records whose level >= the configured minimum level. */
class level_filter final : public i_log_filter
{
public:
    explicit level_filter(log_level min_level = log_level::debug) noexcept
        : min_level_(min_level) {}

    bool is_enabled(log_level level) const noexcept override
    {
        return static_cast<int>(level) >= static_cast<int>(min_level_);
    }

    /** Update the minimum level at runtime. */
    void set_min_level(log_level min_level) noexcept { min_level_ = min_level; }

private:
    log_level min_level_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Logger
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Thread-safe logger.  Records flow through: filter → formatter → sink.
 *
 * Default collaborators (used when not injected):
 *   sink      — ostream_sink(std::cout)
 *   formatter — default_formatter
 *   filter    — level_filter(log_level::debug)   [accepts all levels]
 *
 * Thread safety: a single std::mutex serialises all log() calls and
 * configuration changes; safe to share across threads.
 */
class logger
{
public:
    using clock_t = std::chrono::steady_clock;

    // ── Constructors ──────────────────────────────────────────────────────

    /** Default: all levels enabled, output to std::cout. */
    logger()
        : sink_     (std::make_shared<ostream_sink>())
        , formatter_(std::make_shared<default_formatter>())
        , filter_   (std::make_shared<level_filter>())
        , start_    (clock_t::now())
    {}

    /** Logs to the provided stream; all other settings at their defaults. */
    explicit logger(std::ostream& stream)
        : sink_     (std::make_shared<ostream_sink>(stream))
        , formatter_(std::make_shared<default_formatter>())
        , filter_   (std::make_shared<level_filter>())
        , start_    (clock_t::now())
    {}

    /** Full dependency-injection constructor — supply all three collaborators. */
    logger(std::shared_ptr<i_log_sink>     sink,
           std::shared_ptr<i_log_formatter> formatter,
           std::shared_ptr<i_log_filter>    filter)
        : sink_     (std::move(sink))
        , formatter_(std::move(formatter))
        , filter_   (std::move(filter))
        , start_    (clock_t::now())
    {}

    logger(const logger&)            = delete;
    logger& operator=(const logger&) = delete;
    logger(logger&&)                 = delete;
    logger& operator=(logger&&)      = delete;

    // ── Runtime configuration ─────────────────────────────────────────────

    /** Replace the output sink. */
    void set_sink(std::shared_ptr<i_log_sink> sink)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        sink_ = std::move(sink);
    }

    /** Replace the formatter. */
    void set_formatter(std::shared_ptr<i_log_formatter> formatter)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        formatter_ = std::move(formatter);
    }

    /** Replace the filter. */
    void set_filter(std::shared_ptr<i_log_filter> filter)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        filter_ = std::move(filter);
    }

    /**
     * Convenience: adjust minimum level on the active filter.
     * No-op when the current filter is not a level_filter instance.
     * For full control, use set_filter() with a custom i_log_filter.
     */
    void set_min_level(log_level level)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (auto* f = dynamic_cast<level_filter*>(filter_.get()))
        {
            f->set_min_level(level);
        }
    }

    // ── Core logging API ─────────────────────────────────────────────────

    /** Log with an explicit level and any number of heterogeneous arguments. */
    template <typename... Args>
    void log(log_level level, Args&&... args)
    {
        std::lock_guard<std::mutex> lk(mutex_);

        if (!filter_->is_enabled(level)) return;

        sink_->write(
            formatter_->format(
                level,
                elapsed_ms(),
                detail::make_message(std::forward<Args>(args)...)
            )
        );
    }

    // ── Convenience wrappers ──────────────────────────────────────────────

    template <typename... Args>
    void log_debug(Args&&... args)
    {
        log(log_level::debug, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_info(Args&&... args)
    {
        log(log_level::info, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_warning(Args&&... args)
    {
        log(log_level::warning, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_error(Args&&... args)
    {
        log(log_level::error, std::forward<Args>(args)...);
    }

private:
    long long elapsed_ms() const noexcept
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            clock_t::now() - start_
        ).count();
    }

    std::shared_ptr<i_log_sink>      sink_;
    std::shared_ptr<i_log_formatter> formatter_;
    std::shared_ptr<i_log_filter>    filter_;
    const clock_t::time_point        start_;
    std::mutex                       mutex_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Singleton registry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Process-wide logger singleton.
 *
 * Follows the Meyer's singleton pattern — initialised on first use, destroyed
 * at program exit.  Safe for multi-threaded access in C++11 and later.
 *
 * Usage:
 *   // Configure once (e.g. in main()):
 *   ncmv::logger_registry::instance().set_min_level(ncmv::log_level::info);
 *
 *   // Log from anywhere:
 *   ncmv::logger_registry::instance().log_info("Hello from thread ", id);
 */
class logger_registry
{
public:
    /** Returns the single shared logger. */
    static logger& instance() noexcept
    {
        static logger instance_;
        return instance_;
    }

    /**
     * Convenience factory: reconfigure the global logger with a custom sink,
     * formatter, and filter in one call.
     *
     * @param sink      Destination for formatted records.
     * @param formatter Converts raw data to a string.
     * @param filter    Decides which levels are emitted.
     */
    static void configure(std::shared_ptr<i_log_sink>      sink,
                          std::shared_ptr<i_log_formatter> formatter = std::make_shared<default_formatter>(),
                          std::shared_ptr<i_log_filter>    filter    = std::make_shared<level_filter>())
    {
        instance().set_sink     (std::move(sink));
        instance().set_formatter(std::move(formatter));
        instance().set_filter   (std::move(filter));
    }

    // Non-constructible — access only via instance().
    logger_registry()                                = delete;
    ~logger_registry()                               = delete;
    logger_registry(const logger_registry&)          = delete;
    logger_registry& operator=(const logger_registry&) = delete;
};

} // namespace ncmv

// ─────────────────────────────────────────────────────────────────────────────
// Convenience macros — route through the process-wide singleton
// ─────────────────────────────────────────────────────────────────────────────

/**
 * LOG_DEBUG / LOG_INFO / LOG_WARNING / LOG_ERROR
 *
 * Accept any number of heterogeneous arguments, forwarded directly to the
 * matching logger::log_*() method on the global singleton.
 *
 * Example:
 *   LOG_INFO("Satellites in view: ", count, " (fix=", fix, ")");
 *   LOG_ERROR("UART read failed — errno: ", errno);
 */
#define LOG_DEBUG(...)   ::ncmv::logger_registry::instance().log_debug  (__VA_ARGS__)
#define LOG_INFO(...)    ::ncmv::logger_registry::instance().log_info   (__VA_ARGS__)
#define LOG_WARNING(...) ::ncmv::logger_registry::instance().log_warning(__VA_ARGS__)
#define LOG_ERROR(...)   ::ncmv::logger_registry::instance().log_error  (__VA_ARGS__)

/**
 * LOG_SET_LEVEL(level)
 *
 * Adjust the minimum log level on the global singleton's active filter at
 * runtime.  @p level must be a ncmv::log_level enumerator.
 *
 * Example:
 *   LOG_SET_LEVEL(ncmv::log_level::warning);  // suppress DEBUG and INFO
 */
#define LOG_SET_LEVEL(level) ::ncmv::logger_registry::instance().set_min_level(level)
