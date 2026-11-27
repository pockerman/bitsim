// SPDX-License-Identifier: Apache-2.0

#ifndef BITSIM_UTILS_LOGGER_H
#define BITSIM_UTILS_LOGGER_H

/**
 * @file
 * @brief Logging for bitsim and the libraries built on it, on top of spdlog.
 *
 * A message is written with the stream operator, so anything that has an
 * operator<< can be logged:
 *
 * @code
 * BITSIM_LOG_INFO << "Loaded sensor backend: " << name;
 * BITSIM_LOG_WARN << "Ignoring an invalid IMU message: " << text;
 * @endcode
 *
 * Messages go to named loggers. bitsim logs to "bitsim"; bitrl logs to
 * "bitrl"; BITSIM_LOG_TO(name, level) logs to any other name, and the logger is
 * made the first time it is used. Every logger writes to the same sinks:
 * standard error, in colour, and the files added with add_file_sink().
 *
 * The level of every logger is info unless it is changed with set_level(), or
 * with the SPDLOG_LEVEL environment variable, read when the first logger is
 * made: SPDLOG_LEVEL=debug for all of them, SPDLOG_LEVEL=warn,bitrl=debug for
 * warnings and above except for bitrl. A message below the level of its logger
 * is not formatted at all, so its stream expressions are not evaluated.
 *
 * spdlog is used only in the implementation, so projects that use bitsim do
 * not need its headers. bitsim's own messages are compiled in only when it is
 * built with the BITSIM_LOG option, which is on by default.
 */

#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace bitsim::log
{

/// @brief The severity of a message, and the lowest severity a logger writes
enum class Level
{
    trace,
    debug,
    info,
    warn,
    error,
    critical,
    off
};

/// @brief Set the level of every logger, and of the loggers made later
void set_level(Level level);

/// @brief Set the level of the logger with the given name, making it if needed
void set_level(const std::string &logger, Level level);

/// @brief The level of the logger with the given name, making it if needed
Level get_level(const std::string &logger);

/**
 * @brief Write the messages of every logger to a file as well.
 * @param filename The file, made if it does not exist
 * @param truncate Empty the file first, rather than appending to it
 */
void add_file_sink(const std::string &filename, bool truncate = true);

/**
 * @brief Set the format of the messages of every sink, in spdlog's pattern
 * syntax. The default is "[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v": the time, the
 * logger, the level and the message.
 */
void set_pattern(const std::string &pattern);

/// @brief Write out what the sinks hold
void flush();

namespace detail
{

/// @brief A logger; what it is is known only to the implementation
struct LoggerHandle;

/// @brief The logger with the given name, made the first time. It lives until
/// the program ends.
LoggerHandle *handle(const std::string &logger);

/// @brief Whether the logger writes messages of the given level
bool enabled(LoggerHandle *logger, Level level) noexcept;

/// @brief Write a message, without its trailing new lines
void write(LoggerHandle *logger, Level level, std::string_view message) noexcept;

/// @brief Collects one message with operator<< and writes it when it is
/// destroyed, at the end of the statement
class LogLine
{
  public:
    LogLine(LoggerHandle *logger, Level level) noexcept : logger_(logger), level_(level) {}

    LogLine(const LogLine &) = delete;
    LogLine &operator=(const LogLine &) = delete;

    ~LogLine() { write(logger_, level_, stream_.view()); }

    template <typename T> LogLine &operator<<(const T &value)
    {
        stream_ << value;
        return *this;
    }

    /// @brief For std::endl and the other manipulators
    LogLine &operator<<(std::ostream &(*manipulator)(std::ostream &))
    {
        manipulator(stream_);
        return *this;
    }

  private:
    LoggerHandle *logger_;
    Level level_;
    std::ostringstream stream_;
};

} // namespace detail
} // namespace bitsim::log

/**
 * @brief Log a message to the logger with the given name, a string literal, at
 * the given level: BITSIM_LOG_TO("mylib", bitsim::log::Level::info) << "...";
 * The logger is looked up once for every place this is used.
 */
#define BITSIM_LOG_TO(logger_name, level)                                                          \
    if (auto *const bitsim_log_logger =                                                            \
            []()                                                                                   \
        {                                                                                          \
            static auto *const handle = ::bitsim::log::detail::handle(logger_name);                \
            return handle;                                                                         \
        }();                                                                                       \
        !::bitsim::log::detail::enabled(bitsim_log_logger, level))                                 \
    {                                                                                              \
    }                                                                                              \
    else                                                                                           \
        ::bitsim::log::detail::LogLine(bitsim_log_logger, level)

#define BITSIM_LOG_TRACE BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::trace)
#define BITSIM_LOG_DEBUG BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::debug)
#define BITSIM_LOG_INFO BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::info)
#define BITSIM_LOG_WARN BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::warn)
#define BITSIM_LOG_ERROR BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::error)
#define BITSIM_LOG_CRITICAL BITSIM_LOG_TO("bitsim", ::bitsim::log::Level::critical)

#endif // BITSIM_UTILS_LOGGER_H
