#include "ocr.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <thread>
#include <opencv2/imgproc.hpp>

#ifdef _WIN32
// This SDK's C++/WinRT headers include <experimental/coroutine> in C++17.
// We use the async operation's Status/GetResults API, without coroutines.
// Recent MSVC versions require this SDK compatibility opt-in even when the
// program does not use the deprecated coroutine language extension.
#if defined(_MSC_VER) && !defined(__cpp_impl_coroutine)
#ifndef _SILENCE_EXPERIMENTAL_COROUTINE_DEPRECATION_WARNINGS
#define _SILENCE_EXPERIMENTAL_COROUTINE_DEPRECATION_WARNINGS
#endif
#endif
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Globalization.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Media.Ocr.h>
#include <winrt/Windows.Storage.Streams.h>
#endif

namespace mapimport {

std::vector<Word> recognizeWindows(const cv::Mat& image) {
    if (image.empty()) throw std::invalid_argument("Cannot OCR an empty image.");
#ifdef _WIN32
    // A dedicated MTA thread makes the blocking CLI API independent of the
    // caller's COM apartment and keeps WinRT objects on their creation thread.
    return std::async(std::launch::async, [&image]() {
        try {
            winrt::init_apartment(winrt::apartment_type::multi_threaded);
            struct ApartmentGuard {
                ~ApartmentGuard() { winrt::uninit_apartment(); }
            } apartment;
            namespace imaging = winrt::Windows::Graphics::Imaging;
            namespace ocr = winrt::Windows::Media::Ocr;
            namespace streams = winrt::Windows::Storage::Streams;
            using winrt::Windows::Foundation::AsyncStatus;
            ocr::OcrEngine engine = ocr::OcrEngine::TryCreateFromLanguage(
                winrt::Windows::Globalization::Language(L"en-US"));
            if (!engine) {
                for (const auto& language : ocr::OcrEngine::AvailableRecognizerLanguages()) {
                    const auto tag = winrt::to_string(language.LanguageTag());
                    if (tag == "en" || tag.rfind("en-", 0) == 0) {
                        engine = ocr::OcrEngine::TryCreateFromLanguage(language);
                        if (engine) break;
                    }
                }
            }
            if (!engine) throw std::runtime_error(
                "Windows English OCR language data is unavailable. Install an English "
                "language pack with OCR in Windows Settings, or use --ocr tesseract.");
            const int maximum = static_cast<int>(ocr::OcrEngine::MaxImageDimension());
            if (maximum <= 0) throw std::runtime_error("Windows returned an invalid OCR image limit.");
            cv::Mat source;
            const double scale = std::min(1.0, static_cast<double>(maximum) /
                                              std::max(image.cols, image.rows));
            if (scale < 1.0) cv::resize(image, source, cv::Size(), scale, scale, cv::INTER_AREA);
            else source = image;
            cv::Mat bgra;
            if (source.channels() == 1) cv::cvtColor(source, bgra, cv::COLOR_GRAY2BGRA);
            else if (source.channels() == 3) cv::cvtColor(source, bgra, cv::COLOR_BGR2BGRA);
            else if (source.channels() == 4) bgra = source.clone();
            else throw std::invalid_argument("Windows OCR expects a gray, BGR, or BGRA image.");
            if (bgra.depth() != CV_8U) throw std::invalid_argument("Windows OCR expects an 8-bit image.");
            if (!bgra.isContinuous()) bgra = bgra.clone();
            const auto byteCount = bgra.total() * bgra.elemSize();
            if (byteCount > std::numeric_limits<uint32_t>::max())
                throw std::runtime_error("The image is too large for Windows OCR.");
            streams::DataWriter writer;
            writer.WriteBytes(winrt::array_view<const uint8_t>(bgra.data, bgra.data + byteCount));
            const auto bitmap = imaging::SoftwareBitmap::CreateCopyFromBuffer(
                writer.DetachBuffer(), imaging::BitmapPixelFormat::Bgra8,
                bgra.cols, bgra.rows, imaging::BitmapAlphaMode::Ignore);
            const auto operation = engine.RecognizeAsync(bitmap);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
            while (operation.Status() == AsyncStatus::Started) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    operation.Cancel();
                    throw std::runtime_error("Windows OCR exceeded the 60 second limit.");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            const auto result = operation.GetResults();
            std::vector<Word> words;
            const double sx = static_cast<double>(image.cols) / bgra.cols;
            const double sy = static_cast<double>(image.rows) / bgra.rows;
            for (const auto& line : result.Lines()) {
                for (const auto& word : line.Words()) {
                    const auto box = word.BoundingRect();
                    int left = static_cast<int>(std::floor(box.X * sx));
                    int top = static_cast<int>(std::floor(box.Y * sy));
                    int right = static_cast<int>(std::ceil((box.X + box.Width) * sx));
                    int bottom = static_cast<int>(std::ceil((box.Y + box.Height) * sy));
                    left = std::clamp(left, 0, image.cols);
                    top = std::clamp(top, 0, image.rows);
                    right = std::clamp(right, 0, image.cols);
                    bottom = std::clamp(bottom, 0, image.rows);
                    if (right > left && bottom > top) words.push_back({
                        winrt::to_string(word.Text()), -1.0F,
                        cv::Rect(left, top, right - left, bottom - top)});
                }
            }
            return words;
        } catch (const winrt::hresult_error& error) {
            throw std::runtime_error("Windows OCR failed: " + winrt::to_string(error.message()) +
                ". Check the installed English OCR language pack or use --ocr tesseract.");
        }
    }).get();
#else
    throw std::runtime_error("Windows OCR is available only on Windows; use --ocr tesseract.");
#endif
}

} // namespace mapimport
