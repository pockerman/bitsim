// SPDX-License-Identifier: Apache-2.0

#include "bitsim/utils/logger.h"

#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <map>
#include <memory>
#include <mutex>

namespace bitsim::log
{
namespace detail
{

struct LoggerHandle
{
    std::shared_ptr<spdlog::logger> logger;
};

namespace
{

const std::string DEFAULT_PATTERN = "[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v";

spdlog::level::level_enum to_spdlog(Level level)
{
    switch (level)
    {
    case Level::trace:
        return spdlog::level::trace;
    case Level::debug:
        return spdlog::level::debug;
    case Level::info:
        return spdlog::level::info;
    case Level::warn:
        return spdlog::level::warn;
    case Level::error:
        return spdlog::level::err;
    case Level::critical:
        return spdlog::level::critical;
    case Level::off:
        return spdlog::level::off;
    }
    return spdlog::level::info;
}

Level from_spdlog(spdlog::level::level_enum level)
{
    switch (level)
    {
    case spdlog::level::trace:
        return Level::trace;
    case spdlog::level::debug:
        return Level::debug;
    case spdlog::level::info:
        return Level::info;
    case spdlog::level::warn:
        return Level::warn;
    case spdlog::level::err:
        return Level::error;
    case spdlog::level::critical:
        return Level::critical;
    default:
        return Level::off;
    }
}

// The loggers and the sinks they share. Made on first use and never destroyed,
// so that logging still works while other static objects are destroyed.
struct Registry
{
    Registry() : sinks(std::make_shared<spdlog::sinks::dist_sink_mt>())
    {
        sinks->add_sink(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());

        // spdlog::initialize_logger gives every new logger spdlog's global
        // formatter, which the logger passes on to its sinks, ours: so the
        // pattern is set there
        spdlog::set_pattern(DEFAULT_PATTERN);

        // SPDLOG_LEVEL=warn,bitrl=debug: the levels apply to the loggers that
        // spdlog::initialize_logger is given
        spdlog::cfg::load_env_levels();
    }

    std::mutex mutex;
    std::shared_ptr<spdlog::sinks::dist_sink_mt> sinks;
    std::string pattern{DEFAULT_PATTERN};
    std::map<std::string, std::unique_ptr<LoggerHandle>> loggers;
};

Registry &registry()
{
    static auto *const instance = new Registry();
    return *instance;
}

// The caller holds the mutex
LoggerHandle *find_or_make(Registry &r, const std::string &name)
{
    if (auto itr = r.loggers.find(name); itr != r.loggers.end())
    {
        return itr->second.get();
    }

    auto logger = std::make_shared<spdlog::logger>(name, r.sinks);

    // registers it with spdlog and gives it the level of SPDLOG_LEVEL, or the
    // level set for all loggers, info by default
    spdlog::initialize_logger(logger);

    // the messages are few, so write every one out at once rather than lose
    // the last ones when the program crashes
    logger->flush_on(spdlog::level::trace);

    auto handle = std::make_unique<LoggerHandle>(LoggerHandle{logger});
    auto *result = handle.get();
    r.loggers.emplace(name, std::move(handle));
    return result;
}

} // namespace

LoggerHandle *handle(const std::string &logger)
{
    auto &r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return find_or_make(r, logger);
}

bool enabled(LoggerHandle *logger, Level level) noexcept
{
    return logger != nullptr && logger->logger->should_log(to_spdlog(level));
}

void write(LoggerHandle *logger, Level level, std::string_view message) noexcept
{
    if (logger == nullptr)
    {
        return;
    }

    while (!message.empty() && (message.back() == '\n' || message.back() == '\r'))
    {
        message.remove_suffix(1);
    }

    try
    {
        logger->logger->log(to_spdlog(level), "{}", message);
    }
    catch (...)
    {
        // a message that cannot be written must not end the program
    }
}

} // namespace detail

void set_level(Level level)
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);

    // the level of the loggers made from now on, and of all the loggers
    // spdlog knows, ours among them
    spdlog::set_level(detail::to_spdlog(level));
}

void set_level(const std::string &logger, Level level)
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    detail::find_or_make(r, logger)->logger->set_level(detail::to_spdlog(level));
}

Level get_level(const std::string &logger)
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return detail::from_spdlog(detail::find_or_make(r, logger)->logger->level());
}

void add_file_sink(const std::string &filename, bool truncate)
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, truncate);
    file->set_pattern(r.pattern);
    r.sinks->add_sink(file);
}

void set_pattern(const std::string &pattern)
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.pattern = pattern;

    // for the loggers made from now on, and through the loggers spdlog knows,
    // for our sinks
    spdlog::set_pattern(pattern);
    r.sinks->set_pattern(pattern);
}

void flush()
{
    auto &r = detail::registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.sinks->flush();
}

} // namespace bitsim::log
