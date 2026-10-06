#include "ocr.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int checks = 0;
void check(bool condition, const char* description) {
    ++checks;
    if (!condition) throw std::runtime_error(description);
}
template<class Function>
void rejects(Function function, const char* description) {
    bool threw = false;
    try { function(); } catch (const std::exception&) { threw = true; }
    check(threw, description);
}
const std::string header =
    "level\tpage_num\tblock_num\tpar_num\tline_num\tword_num\tleft\ttop\twidth\theight\tconf\ttext\n";
std::string row(const std::string& geometry, const std::string& confidence,
                const std::string& word = "OBSTACLE") {
    return "5\t1\t1\t1\t1\t1\t" + geometry + "\t" + confidence + "\t" + word + "\n";
}
}

int main() {
    try {
        for (const auto& text : std::vector<std::string>{"OBSTACLE", "obstacle", "Obstacle",
                 "(OBSTACLE)", "OBSTACLE.", " OBSTACLE "})
            check(mapimport::isObstacleWord(text), "Valid obstacle label was rejected.");
        for (const auto& text : std::vector<std::string>{"", "ROAD", "NOTOBSTACLE", "OBSTACLES",
                 "OBSTACLE2", "2OBSTACLE", "OBS1TACLE", "OBSTAC1E", "OBST4ACLE",
                 "OBSTAC", "OBSTACLE ROAD", "\xC3\x93" "BSTACLE"})
            check(!mapimport::isObstacleWord(text), "Invalid obstacle label was accepted.");

        const auto valid = mapimport::parseTsv(header +
            row("20\t30\t80\t20", "96.2") +
            row("0\t0\t100\t100", "100", "ROAD"), cv::Size(100, 100));
        check(valid.size() == 2, "Valid TSV words were lost.");
        check(valid[0].text == "OBSTACLE", "TSV text changed.");
        check(valid[0].box == cv::Rect(20, 30, 80, 20), "TSV box changed.");
        check(std::abs(valid[0].confidence - 96.2F) < 0.001F, "TSV confidence changed.");
        check(valid[1].box == cv::Rect(0, 0, 100, 100), "Border-aligned box rejected.");

        for (const auto& geometry : std::vector<std::string>{
                 "-1\t30\t80\t20", "20\t-1\t80\t20", "20\t30\t0\t20",
                 "20\t30\t80\t0", "21\t30\t80\t20", "20\t81\t80\t20",
                 "0\t0\t2147483647\t20", "2147483648\t0\t20\t20",
                 "20x\t30\t80\t20", "20\t30\t80.1\t20"})
            check(mapimport::parseTsv(header + row(geometry, "99"), cv::Size(100,100)).empty(),
                  "Malformed or out-of-image TSV box was accepted.");
        for (const auto& confidence : std::vector<std::string>{"-1", "100.1", "NaN", "inf", "99x"})
            check(mapimport::parseTsv(header + row("20\t30\t80\t20", confidence),
                                    cv::Size(100,100)).empty(), "Invalid TSV confidence accepted.");
        check(mapimport::parseTsv(header + row("20\t30\t80\t20", "0"),
                                  cv::Size(100,100)).size() == 1, "Zero confidence row lost.");
        check(mapimport::parseTsv(header + row("20\t30\t80\t20", "99", ""),
                                  cv::Size(100,100)).empty(), "Empty word accepted.");
        check(mapimport::parseTsv(header +
            "4\t1\t1\t1\t1\t0\t20\t30\t80\t20\t99\tOBSTACLE\n",
            cv::Size(100,100)).empty(), "Non-word TSV row accepted.");
        check(mapimport::parseTsv(header + "5\tbroken row\n", cv::Size(100,100)).empty(),
              "Incomplete TSV row accepted.");
        check(mapimport::parseTsv(header, cv::Size(100,100)).empty(), "Empty TSV data changed.");
        auto crlf = header + row("20\t30\t80\t20", "99");
        for (std::size_t position = 0; (position = crlf.find('\n', position)) != std::string::npos;
             position += 2) crlf.insert(position, 1, '\r');
        check(mapimport::parseTsv(crlf, cv::Size(100,100)).size() == 1, "CRLF TSV rejected.");
        rejects([] { mapimport::parseTsv("not TSV", cv::Size(100,100)); },
                "Invalid TSV header accepted.");
        rejects([] { mapimport::parseTsv("", cv::Size(100,100)); }, "Empty output accepted.");
        rejects([] { mapimport::parseTsv(header, cv::Size(0,100)); }, "Zero image width accepted.");
        rejects([] { mapimport::parseTsv(header, cv::Size(100,-1)); }, "Negative image height accepted.");
        std::cout << "OCR TSV parsing and exact label matching: " << checks << " checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "OCR test failed: " << error.what() << '\n';
        return 1;
    }
}
