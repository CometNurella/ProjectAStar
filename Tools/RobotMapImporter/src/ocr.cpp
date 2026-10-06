#include "ocr.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <opencv2/imgcodecs.hpp>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace mapimport {
namespace {

constexpr auto kProcessTimeout = std::chrono::seconds(60);
constexpr std::size_t kMaxOutputBytes = 16U * 1024U * 1024U;

struct ScratchDirectory {
    std::filesystem::path path;
    ~ScratchDirectory() {
        std::error_code ignored;
        // Only remove the unique directory created by this invocation.
        if (!path.empty()) std::filesystem::remove_all(path, ignored);
    }
};

std::filesystem::path makeScratch(const std::filesystem::path& workDir) {
    const auto root = std::filesystem::absolute(workDir);
    std::filesystem::create_directories(root);
    static std::atomic<unsigned long long> counter{0};
    for (unsigned int attempt = 0; attempt < 100; ++attempt) {
        const auto stamp = std::chrono::high_resolution_clock::now()
                               .time_since_epoch().count();
        std::ostringstream name;
        name << "ocr-" << std::hex << stamp << '-' << counter.fetch_add(1);
        auto path = root / name.str();
        std::error_code error;
        if (std::filesystem::create_directory(path, error)) return path;
        if (error && error != std::errc::file_exists)
            throw std::runtime_error("Cannot create OCR temporary directory: " + error.message());
    }
    throw std::runtime_error("Cannot create a unique OCR temporary directory.");
}

std::string readLimited(const std::filesystem::path& file) {
    std::ifstream input(file, std::ios::binary);
    if (!input) return {};
    std::string output;
    char buffer[8192];
    while (input.read(buffer, sizeof(buffer)) || input.gcount()) {
        const auto count = static_cast<std::size_t>(input.gcount());
        if (output.size() + count > kMaxOutputBytes)
            throw std::runtime_error("OCR output exceeds the 16 MB limit.");
        output.append(buffer, count);
    }
    return output;
}

std::string brief(const std::string& text) {
    constexpr std::size_t limit = 1500;
    return text.size() <= limit ? text : text.substr(0, limit) + "...";
}

#ifdef _WIN32
std::wstring utf16(const std::string& value) {
    if (value.empty()) return {};
    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                        value.data(), static_cast<int>(value.size()), nullptr, 0);
    if (!size) throw std::runtime_error("OCR path contains invalid UTF-8.");
    std::wstring result(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                        static_cast<int>(value.size()), result.data(), size);
    return result;
}

// Windows command-line escaping follows CommandLineToArgvW/CRT rules.
std::wstring quote(const std::wstring& argument) {
    std::wstring result = L"\"";
    std::size_t slashes = 0;
    for (const auto c : argument) {
        if (c == L'\\') { ++slashes; continue; }
        if (c == L'\"') {
            result.append(2 * slashes + 1, L'\\');
            result.push_back(c);
        } else {
            result.append(slashes, L'\\');
            result.push_back(c);
        }
        slashes = 0;
    }
    result.append(2 * slashes, L'\\');
    result.push_back(L'\"');
    return result;
}

struct Handle {
    HANDLE value = INVALID_HANDLE_VALUE;
    ~Handle() { if (value && value != INVALID_HANDLE_VALUE) CloseHandle(value); }
};

int run(const std::vector<std::string>& args,
        const std::filesystem::path& stdoutPath,
        const std::filesystem::path& stderrPath) {
    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    Handle out{CreateFileW(stdoutPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ,
                           &security, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr)};
    Handle err{CreateFileW(stderrPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ,
                           &security, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr)};
    Handle in{CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                          &security, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr)};
    if (out.value == INVALID_HANDLE_VALUE || err.value == INVALID_HANDLE_VALUE ||
        in.value == INVALID_HANDLE_VALUE)
        throw std::runtime_error("Cannot open OCR process input/output files.");

    std::wstring command;
    for (const auto& arg : args) {
        if (!command.empty()) command.push_back(L' ');
        command += quote(utf16(arg));
    }
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = in.value;
    startup.hStdOutput = out.value;
    startup.hStdError = err.value;
    PROCESS_INFORMATION information{};
    // No shell is used. If the executable is a basename, Windows searches PATH.
    if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr, TRUE,
                        CREATE_NO_WINDOW, nullptr, nullptr, &startup, &information)) {
        const auto code = GetLastError();
        throw std::runtime_error("Cannot start Tesseract (Windows error " +
            std::to_string(code) + "). Install Tesseract with English language data, "
            "or supply --tesseract with the full path to tesseract.exe.");
    }
    Handle process{information.hProcess};
    Handle thread{information.hThread};
    const auto waited = WaitForSingleObject(process.value,
        static_cast<DWORD>(std::chrono::duration_cast<std::chrono::milliseconds>(kProcessTimeout).count()));
    if (waited == WAIT_TIMEOUT) {
        TerminateProcess(process.value, 124);
        WaitForSingleObject(process.value, INFINITE);
        throw std::runtime_error("Tesseract exceeded the 60 second OCR limit.");
    }
    if (waited != WAIT_OBJECT_0) {
        TerminateProcess(process.value, 125);
        WaitForSingleObject(process.value, INFINITE);
        throw std::runtime_error("Cannot wait for the Tesseract process.");
    }
    DWORD exitCode = 0;
    if (!GetExitCodeProcess(process.value, &exitCode))
        throw std::runtime_error("Cannot read the Tesseract exit code.");
    return static_cast<int>(exitCode);
}
#else
int run(const std::vector<std::string>& args,
        const std::filesystem::path& stdoutPath,
        const std::filesystem::path& stderrPath) {
    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0)
        throw std::runtime_error("Cannot initialize the OCR process.");
    struct ActionsGuard {
        posix_spawn_file_actions_t* actions;
        ~ActionsGuard() { posix_spawn_file_actions_destroy(actions); }
    } guard{&actions};
    int error = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    if (!error) error = posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO,
        stdoutPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (!error) error = posix_spawn_file_actions_addopen(&actions, STDERR_FILENO,
        stderrPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (error) throw std::runtime_error("Cannot open OCR process input/output files: " +
                                      std::error_code(error, std::generic_category()).message());
    std::vector<char*> argv;
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    pid_t process = 0;
    error = posix_spawnp(&process, args.front().c_str(), &actions, nullptr, argv.data(), environ);
    if (error) throw std::runtime_error("Cannot start Tesseract: " +
        std::error_code(error, std::generic_category()).message() +
        ". Install Tesseract with English language data, or supply --tesseract.");
    const auto deadline = std::chrono::steady_clock::now() + kProcessTimeout;
    int status = 0;
    for (;;) {
        const auto waited = waitpid(process, &status, WNOHANG);
        if (waited == process) break;
        if (waited == -1 && errno != EINTR) {
            kill(process, SIGKILL);
            while (waitpid(process, &status, 0) == -1 && errno == EINTR) {}
            throw std::runtime_error("Cannot wait for the Tesseract process.");
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            kill(process, SIGKILL);
            while (waitpid(process, &status, 0) == -1 && errno == EINTR) {}
            throw std::runtime_error("Tesseract exceeded the 60 second OCR limit.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return WIFSIGNALED(status) ? 128 + WTERMSIG(status) : 1;
}
#endif

std::vector<std::string> splitTsv(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (int field = 0; field < 11; ++field) {
        const auto end = line.find('\t', start);
        if (end == std::string::npos) return {};
        fields.push_back(line.substr(start, end - start));
        start = end + 1;
    }
    fields.push_back(line.substr(start));
    return fields;
}

int integer(const std::string& value) {
    std::size_t used = 0;
    const auto number = std::stoi(value, &used);
    if (used != value.size()) throw std::invalid_argument("invalid integer");
    return number;
}

} // namespace

bool isObstacleWord(const std::string& text) {
    std::string normalized;
    for (const unsigned char c : text) {
        if (c >= 'a' && c <= 'z') normalized.push_back(static_cast<char>(c - 'a' + 'A'));
        else if (c >= 'A' && c <= 'Z') normalized.push_back(static_cast<char>(c));
        else if (c >= '0' && c <= '9') return false;
        else if (c >= 128) return false;
    }
    return normalized == "OBSTACLE";
}

std::vector<Word> parseTsv(const std::string& tsv, const cv::Size& imageSize) {
    if (imageSize.width <= 0 || imageSize.height <= 0)
        throw std::invalid_argument("OCR image dimensions must be positive.");
    std::istringstream input(tsv);
    std::string line;
    if (!std::getline(input, line) || line.rfind("level\tpage_num\t", 0) != 0)
        throw std::runtime_error("Tesseract returned invalid TSV output.");
    std::vector<Word> words;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        const auto fields = splitTsv(line);
        if (fields.size() != 12) continue;
        try {
            if (integer(fields[0]) != 5 || fields[11].empty()) continue;
            const int left = integer(fields[6]), top = integer(fields[7]);
            const int width = integer(fields[8]), height = integer(fields[9]);
            std::size_t used = 0;
            const auto confidence = std::stof(fields[10], &used);
            if (used != fields[10].size() || !std::isfinite(confidence) ||
                confidence < 0.0F || confidence > 100.0F ||
                left < 0 || top < 0 || width <= 0 || height <= 0 ||
                left > imageSize.width - width || top > imageSize.height - height) continue;
            words.push_back({fields[11], confidence, cv::Rect(left, top, width, height)});
        } catch (const std::invalid_argument&) { /* Ignore malformed rows. */ }
          catch (const std::out_of_range&) { /* Ignore overflowing fields. */ }
    }
    return words;
}

std::vector<Word> recognize(const cv::Mat& image,
                            const std::filesystem::path& workDir,
                            const std::string& executable,
                            const std::string& tessdata,
                            int pageSegmentationMode) {
    if (image.empty()) throw std::invalid_argument("Cannot OCR an empty image.");
    if (executable.empty()) throw std::invalid_argument("Tesseract executable must not be empty.");
    if (pageSegmentationMode < 3 || pageSegmentationMode > 13)
        throw std::invalid_argument("OCR page segmentation mode must be between 3 and 13.");
    ScratchDirectory scratch{makeScratch(workDir)};
    const auto png = scratch.path / "input.png";
    const auto out = scratch.path / "words.tsv";
    const auto err = scratch.path / "stderr.txt";
    // imencode plus filesystem I/O also supports Unicode filenames on Windows.
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".png", image, encoded))
        throw std::runtime_error("Cannot encode the image for OCR.");
    std::ofstream file(png, std::ios::binary);
    if (!file || !file.write(reinterpret_cast<const char*>(encoded.data()),
                             static_cast<std::streamsize>(encoded.size())))
        throw std::runtime_error("Cannot save the temporary image for OCR.");
    file.close();
    const auto utf8Path = png.u8string();
    const std::string imagePath(reinterpret_cast<const char*>(utf8Path.data()), utf8Path.size());
    std::vector<std::string> args{executable, imagePath, "stdout", "-l", "eng", "--psm",
                                  std::to_string(pageSegmentationMode)};
    if (!tessdata.empty()) { args.push_back("--tessdata-dir"); args.push_back(tessdata); }
    // Avoid requiring a separately installed configs/tsv file.
    args.push_back("-c"); args.push_back("tessedit_create_tsv=1");
    const auto exitCode = run(args, out, err);
    if (exitCode != 0) throw std::runtime_error("Tesseract failed (exit " +
        std::to_string(exitCode) + "): " + brief(readLimited(err)));
    return parseTsv(readLimited(out), image.size());
}

} // namespace mapimport
