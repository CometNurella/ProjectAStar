#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace mapimport {

struct Word {
    std::string text;
    float confidence = 0.0F;
    cv::Rect box;
};

// The input may be a full map or a cropped region. Coordinates are relative
// to the supplied image; a caller cropping a region must add its offset.
// The executable is invoked directly, without a command shell.
std::vector<Word> recognize(const cv::Mat& image,
                            const std::filesystem::path& workDir,
                            const std::string& executable = "tesseract",
                            const std::string& tessdata = "",
                            int pageSegmentationMode = 11);

// TSV parsing is separate so tests can check the Tesseract output contract.
std::vector<Word> parseTsv(const std::string& tsv, const cv::Size& imageSize);

// Windows 10/11 OCR uses installed English OCR language data. Confidence is
// -1 because the Windows API does not expose per-word confidence scores.
// Throws an explanatory error on other operating systems.
std::vector<Word> recognizeWindows(const cv::Mat& image);

// Case and punctuation do not matter. Digit substitutions are rejected:
// OBSTACLE. matches; OBSTAC1E, NOTOBSTACLE, and OBSTACLE2 do not.
bool isObstacleWord(const std::string& text);

} // namespace mapimport
