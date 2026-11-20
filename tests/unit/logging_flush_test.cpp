#include <fcntl.h>
#include <sys/resource.h>
#include <unistd.h>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/log.h"
#include "core/log_pattern.h"

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Json = nlohmann::json;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void write_all(int fd, const std::string& text) {
    std::size_t offset = 0;
    while (offset < text.size()) {
        const auto written = ::write(fd, text.data() + offset, text.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        require(written > 0, "写入测试结果失败");
        offset += static_cast<std::size_t>(written);
    }
}

class RedirectStdout {
   public:
    explicit RedirectStdout(const std::filesystem::path& path) {
        std::fflush(stdout);
        saved_ = ::dup(STDOUT_FILENO);
        require(saved_ >= 0, "保存标准输出失败");
        const int output = ::open(path.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0600);
        if (output < 0) {
            ::close(saved_);
            saved_ = -1;
            throw std::runtime_error("创建终端日志文件失败");
        }
        const bool redirected = ::dup2(output, STDOUT_FILENO) >= 0;
        ::close(output);
        if (!redirected) {
            ::close(saved_);
            saved_ = -1;
            throw std::runtime_error("重定向标准输出失败");
        }
    }

    ~RedirectStdout() {
        if (saved_ >= 0) {
            std::fflush(stdout);
            ::dup2(saved_, STDOUT_FILENO);
            ::close(saved_);
        }
    }

   private:
    int saved_ = -1;
};

class TemporaryDirectory {
   public:
    TemporaryDirectory() {
        char pattern[] = "/tmp/plane-logging-flush-XXXXXX";
        const auto* created = ::mkdtemp(pattern);
        require(created != nullptr, "创建日志测试临时目录失败");
        path = created;
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }

    std::filesystem::path path;
};

std::string read_file(const std::filesystem::path& path) {
    std::ifstream file(path);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

bool contains(const std::filesystem::path& path, const std::string& text) {
    return read_file(path).find(text) != std::string::npos;
}

double wait_until_visible(const std::filesystem::path& path, const std::string& text,
                          std::chrono::milliseconds timeout) {
    const auto start = Clock::now();
    while (Clock::now() - start < timeout) {
        if (contains(path, text)) {
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        }
        std::this_thread::sleep_for(5ms);
    }
    throw std::runtime_error("单条普通日志未在刷新周期后可见: " + text);
}

std::size_t thread_count() {
    return static_cast<std::size_t>(std::distance(std::filesystem::directory_iterator("/proc/self/task"),
                                                 std::filesystem::directory_iterator()));
}

void require_threads(std::size_t expected, const std::string& message) {
    const auto deadline = Clock::now() + 1s;
    while (Clock::now() < deadline) {
        if (thread_count() == expected) return;
        std::this_thread::sleep_for(5ms);
    }
    require(thread_count() == expected, message);
}

Json test_mode(const std::filesystem::path& directory, bool console, int interval_ms) {
    const auto initial_threads = thread_count();
    plane::core::LogOptions options;
    options.log_dir = directory.string();
    options.console = console;
    options.flush_interval_ms = interval_ms;
    plane::core::init_logging(options);
    const auto retained_logger = spdlog::default_logger();
    require(retained_logger->flush_level() == spdlog::level::warn,
            "前台和后台必须统一为 warn/error 即时刷新");
    require_threads(initial_threads + 1, "初始化应仅创建一个周期刷新线程");

    const auto path = directory / "plane.log";
    SPDLOG_INFO("仅输出一次的普通日志: flush_interval_ms={}", interval_ms);
    const double visible_ms = wait_until_visible(path, "仅输出一次的普通日志", 2500ms);
    SPDLOG_WARN("需要立即可见的警告");
    require(contains(path, "需要立即可见的警告"), "warn 日志返回后必须可读");
    SPDLOG_ERROR("需要立即可见的错误");
    require(contains(path, "需要立即可见的错误"), "error 日志返回后必须可读");
    SPDLOG_INFO("退出前最后一条普通日志");
    plane::core::shutdown_logging();
    // 保留 logger 引用，确保不是依赖析构隐式关闭文件才刷新退出日志。
    require(contains(path, "退出前最后一条普通日志"), "正常退出必须显式刷新尾部普通日志");
    require_threads(initial_threads, "停止日志后周期刷新线程未回收");
    return {{"console", console}, {"flush_interval_ms", interval_ms}, {"visible_after_ms", visible_ms}};
}

void test_reinitialization(const std::filesystem::path& directory) {
    const auto initial_threads = thread_count();
    plane::core::LogOptions options;
    options.console = false;
    options.log_dir = (directory / "first").string();
    options.flush_interval_ms = 3000;
    plane::core::init_logging(options);
    const auto retained_logger = spdlog::default_logger();
    SPDLOG_INFO("第一次初始化留下的尾部日志");
    plane::core::shutdown_logging();
    require(contains(directory / "first/plane.log", "第一次初始化留下的尾部日志"),
            "重新初始化前必须刷新原日志器");
    require_threads(initial_threads, "重新初始化前旧刷新线程未退出");

    options.log_dir = (directory / "second").string();
    options.flush_interval_ms = 60;
    plane::core::init_logging(options);
    require_threads(initial_threads + 1, "重新初始化后存在重复刷新线程");
    SPDLOG_INFO("重新初始化后按新周期刷新的日志");
    wait_until_visible(directory / "second/plane.log", "重新初始化后按新周期刷新的日志", 1000ms);
    const auto second_logger = spdlog::default_logger();
    SPDLOG_INFO("直接重新初始化前的尾部日志");
    options.log_dir = (directory / "third").string();
    options.flush_interval_ms = 80;
    plane::core::init_logging(options);
    require(contains(directory / "second/plane.log", "直接重新初始化前的尾部日志"),
            "直接重新初始化必须刷新旧日志器");
    require_threads(initial_threads + 1, "直接重新初始化未回收原刷新线程");
    SPDLOG_INFO("直接重新初始化后按新周期刷新的日志");
    wait_until_visible(directory / "third/plane.log", "直接重新初始化后按新周期刷新的日志", 1000ms);
    plane::core::shutdown_logging();
    plane::core::shutdown_logging();
    require_threads(initial_threads, "重复停止后仍残留刷新线程");

    for (const int invalid : {0, -1}) {
        options.flush_interval_ms = invalid;
        bool rejected = false;
        try {
            plane::core::init_logging(options);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "非正日志刷新周期必须被拒绝");
        require_threads(initial_threads, "非法初始化不得留下刷新线程");
    }
}

// 转发给真实线程安全文件 sink，只统计显式 flush，不将字节写满导致的缓冲刷新混为一谈。
class CountingSink final : public spdlog::sinks::sink {
   public:
    explicit CountingSink(spdlog::sink_ptr wrapped) : wrapped_(std::move(wrapped)) {}
    void log(const spdlog::details::log_msg& message) override { wrapped_->log(message); }
    void flush() override {
        ++flushes;
        wrapped_->flush();
    }
    void set_pattern(const std::string& pattern) override { wrapped_->set_pattern(pattern); }
    void set_formatter(std::unique_ptr<spdlog::formatter> formatter) override {
        wrapped_->set_formatter(std::move(formatter));
    }

    std::atomic<std::size_t> flushes{0};

   private:
    spdlog::sink_ptr wrapped_;
};

double cpu_seconds() {
    rusage usage{};
    require(::getrusage(RUSAGE_SELF, &usage) == 0, "读取 CPU 使用时间失败");
    return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
           static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1'000'000.0;
}

Json benchmark_phase(const std::filesystem::path& directory, bool console, bool periodic, int seconds, int rate_hz) {
    plane::core::LogOptions options;
    options.app_name = periodic ? "periodic" : "baseline";
    options.log_dir = directory.string();
    options.console = console;
    options.flush_interval_ms = 1000;
    if (periodic) {
        plane::core::init_logging(options);
        spdlog::flush_every(0ms);
    } else {
        std::filesystem::create_directories(directory);
        std::vector<spdlog::sink_ptr> sinks;
        if (console) sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            (directory / "baseline.log").string(), options.max_file_size_bytes, options.max_files));
        const auto logger = std::make_shared<spdlog::logger>(options.app_name, sinks.begin(), sinks.end());
        logger->set_level(spdlog::level::info);
        logger->flush_on(console ? spdlog::level::warn : spdlog::level::info);
        spdlog::set_default_logger(logger);
        plane::core::set_project_log_pattern();
    }
    const auto retained_logger = spdlog::default_logger();
    auto file_counter = std::make_shared<CountingSink>(retained_logger->sinks().back());
    retained_logger->sinks().back() = file_counter;
    if (periodic) spdlog::flush_every(std::chrono::milliseconds(options.flush_interval_ms));

    const auto count = static_cast<std::size_t>(seconds) * static_cast<std::size_t>(rate_hz);
    const auto step = std::chrono::nanoseconds(1'000'000'000 / rate_hz);
    const double cpu_before = cpu_seconds();
    const auto start = Clock::now();
    for (std::size_t index = 0; index < count; ++index) {
        SPDLOG_INFO("固定负载性能测试日志：普通运行状态与数据摘要，保持相同内容、速率和条数");
        std::this_thread::sleep_until(start + step * static_cast<std::int64_t>(index + 1));
    }
    const double wall_seconds = std::chrono::duration<double>(Clock::now() - start).count();
    // 停止定时器后读取计数，区分负载期间刷新和关闭阶段的最终刷新。
    if (periodic) spdlog::flush_every(0ms);
    const double used_cpu_seconds = cpu_seconds() - cpu_before;
    const auto load_flushes = file_counter->flushes.load();
    plane::core::shutdown_logging();
    const auto total_flushes = file_counter->flushes.load();
    return {{"policy", periodic ? "periodic" : (console ? "warn" : "info")},
            {"console", console},
            {"duration_seconds", wall_seconds},
            {"cpu_seconds", used_cpu_seconds},
            {"cpu_percent_one_core", 100.0 * used_cpu_seconds / wall_seconds},
            {"message_count", count},
            {"rate_hz", rate_hz},
            {"flush_count_during_load", load_flushes},
            {"flush_count_at_shutdown", total_flushes - load_flushes},
            {"flush_count_total", total_flushes},
            {"flush_interval_ms", periodic ? options.flush_interval_ms : 0},
            {"log_file", (directory / (options.app_name + ".log")).string()}};
}

int positive_number(const char* text) {
    const std::string value(text);
    int number = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), number);
    require(result.ec == std::errc{} && result.ptr == value.data() + value.size() && number > 0,
            "基准时长和频率必须为正整数");
    return number;
}

Json run_benchmark(int argc, char** argv) {
    int seconds = 60;
    int rate_hz = 50;
    bool console = false;
    auto directory = std::filesystem::temp_directory_path() /
                     ("plane-log-flush-benchmark-" + std::to_string(::getpid()));
    for (int index = 2; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--console") {
            console = true;
        } else if (argument == "--duration-seconds" && index + 1 < argc) {
            seconds = positive_number(argv[++index]);
        } else if (argument == "--rate-hz" && index + 1 < argc) {
            rate_hz = positive_number(argv[++index]);
        } else if (argument == "--log-dir" && index + 1 < argc) {
            directory = argv[++index];
        } else {
            throw std::invalid_argument("未知或不完整的基准参数: " + argument);
        }
    }
    require(seconds <= 3600 && rate_hz <= 10000, "基准时长或频率超过测试范围");
    std::filesystem::create_directories(directory);
    Json baseline;
    Json periodic;
    {
        RedirectStdout redirected(directory / "console.log");
        write_all(STDERR_FILENO, "开始采集现有刷新策略的固定负载 CPU 基线。\n");
        baseline = benchmark_phase(directory, console, false, seconds, rate_hz);
        write_all(STDERR_FILENO, "开始采集定时刷新策略的相同负载 CPU 数据。\n");
        periodic = benchmark_phase(directory, console, true, seconds, rate_hz);
    }
    const double delta = periodic["cpu_percent_one_core"].get<double>() -
                         baseline["cpu_percent_one_core"].get<double>();
    return {{"baseline", baseline},
            {"periodic", periodic},
            {"cpu_delta_percentage_points", delta},
            {"cpu_target_met", delta <= 1.0},
            {"console_log", (directory / "console.log").string()},
            {"flush_count_definition", "真实文件 sink 的显式 flush 次数；不包含缓冲区写满触发的自动写出"}};
}

Json run_tests() {
    TemporaryDirectory directory;
    require(plane::core::LogOptions{}.flush_interval_ms == 1000, "普通日志默认刷新周期必须为 1000 毫秒");
    Json observations = Json::array();
    {
        RedirectStdout redirected(directory.path / "console.log");
        observations.push_back(test_mode(directory.path / "no-console-default", false, 1000));
        observations.push_back(test_mode(directory.path / "console-custom", true, 120));
        test_reinitialization(directory.path / "reinitialize");
    }
    return {{"status", "passed"}, {"visibility", observations}};
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Json result;
        if (argc > 1) {
            require(std::string(argv[1]) == "--benchmark", "仅支持 --benchmark 或无参数运行测试");
            result = run_benchmark(argc, argv);
        } else {
            result = run_tests();
        }
        write_all(STDOUT_FILENO, result.dump(2) + '\n');
        return result.value("cpu_target_met", true) ? 0 : 2;
    } catch (const std::exception& error) {
        plane::core::shutdown_logging();
        write_all(STDERR_FILENO, std::string("日志刷新测试失败: ") + error.what() + '\n');
        return 1;
    }
}
