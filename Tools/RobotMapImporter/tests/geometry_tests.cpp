#include "map.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int checks = 0;
void check(bool condition, const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& operation, const char* message) {
    ++checks;
    try { operation(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error(message);
}
}

int main() {
    try {
        cv::Mat grid = cv::Mat::zeros(3, 3, CV_8U);
        grid.at<unsigned char>(0, 1) = 255;
        grid.at<unsigned char>(0, 2) = 255;
        grid.at<unsigned char>(1, 0) = 255;
        grid.at<unsigned char>(2, 1) = 255;
        const auto json = mapimport::robotJson(grid, 0.5, "Test_map", "Test_case", {0, 0}, {2, 0});
        check(json["origin"] == "bottom-left", "JSON origin");
        check(json["width"] == 3 && json["height"] == 3, "JSON dimensions");
        check(json["maps"]["Test_map"]["obstacles_rect_x1y1x2y2"] ==
            nlohmann::json::array({{1, 2, 2, 2}, {0, 1, 0, 1}, {1, 0, 1, 0}}),
            "Inclusive row runs or image-row y flip are incorrect");
        check(json["maps"]["Test_map"]["cases"][0]["Ct"] == nlohmann::json::array({0.25, 0.25}),
            "Start must be a world cell center");
        check(json["maps"]["Test_map"]["cases"][0]["n"] == nlohmann::json::array({1.25, 0.25}),
            "Goal must be a world cell center");
        rejects([&]{mapimport::robotJson(grid, 0.5, "Map", "Case", {1, 0}, {2, 0});}, "Blocked start accepted");
        rejects([&]{mapimport::robotJson(grid, 0.5, "Map", "Case", {0, 0}, {2, 2});}, "Blocked goal accepted");
        rejects([&]{mapimport::robotJson(grid, 0.5, "Map", "Case", {-1, 0}, {2, 0});}, "Outside start accepted");
        rejects([&]{mapimport::robotJson(grid, 0.5, "Map", "Case", {0, 0}, {3, 0});}, "Outside goal accepted");
        for (double size : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
            rejects([&]{mapimport::robotJson(grid, size, "Map", "Case", {0, 0}, {2, 0});}, "Invalid cell size accepted");
        for (const auto* id : {"", "../Map", "COM1", "con", "Map name", "Map:1"})
            rejects([&]{mapimport::validateId(id);}, "Unsafe ID accepted");

        mapimport::Shape concave;
        concave.contour = {{10, 10}, {90, 10}, {90, 40}, {40, 40}, {40, 90}, {10, 90}};
        concave.bounds = {10, 10, 81, 81};
        concave.area = 3900;
        concave.selected = true;
        const auto mask = mapimport::obstacleMask({100, 100}, {concave});
        check(mask.at<unsigned char>(25, 70) != 0, "Concave top bar absent");
        check(mask.at<unsigned char>(70, 25) != 0, "Concave leg absent");
        check(mask.at<unsigned char>(70, 70) == 0, "Concave free notch filled");
        check(mapimport::enclosingShape({concave}, {20, 18, 50, 15}) == 0, "Inside label not associated");
        check(mapimport::enclosingShape({concave}, {50, 55, 20, 20}) == -1, "Label in free notch incorrectly associated");
        mapimport::Shape narrowNotch;
        narrowNotch.contour = {{10, 10}, {90, 10}, {90, 30}, {50, 30}, {50, 40}, {90, 40}, {90, 90}, {10, 90}};
        narrowNotch.bounds = {10, 10, 81, 81};
        narrowNotch.area = 6000;
        check(mapimport::enclosingShape({narrowNotch}, {20, 20, 60, 60}) == -1,
              "Label box crossing a concave notch accepted despite its corners and center being inside");
        concave.selected = false;
        check(cv::countNonZero(mapimport::obstacleMask({100, 100}, {concave})) == 0, "Unselected shape blocked");

        cv::Mat onePixel = cv::Mat::zeros(20, 20, CV_8U);
        onePixel.at<unsigned char>(1, 1) = 255;
        const auto anyTouched = mapimport::occupancy(onePixel, 2, 2, 0);
        check(anyTouched.at<unsigned char>(0, 0) != 0 && cv::countNonZero(anyTouched) == 1,
              "Conservative occupancy must retain touched cell");
        check(cv::countNonZero(mapimport::occupancy(onePixel, 2, 2, 0.5)) == 0,
              "Coverage threshold ignored");

        cv::Mat drawing(300, 400, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::rectangle(drawing, {40, 50}, {340, 230}, cv::Scalar(200, 70, 30), 5);
        auto detection = mapimport::detect(drawing, 40, 90, 0, 0.001);
        check(mapimport::enclosingShape(detection.shapes, {110, 110, 120, 30}) >= 0,
              "Closed colored outline not found");
        cv::rectangle(drawing, {130, 42}, {220, 60}, cv::Scalar(255, 255, 255), cv::FILLED);
        detection = mapimport::detect(drawing, 40, 90, 0, 0.001);
        check(mapimport::enclosingShape(detection.shapes, {110, 110, 120, 30}) == -1,
              "Open pen stroke invented a closed obstacle");
        cv::Mat framedPage(300, 400, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::rectangle(framedPage, {0, 0}, {399, 299}, cv::Scalar(200, 70, 30), 30);
        const auto frameDetection = mapimport::detect(framedPage, 40, 90, 0, 0.001);
        check(mapimport::enclosingShape(frameDetection.shapes, {110, 110, 120, 30}) == -1,
              "Interior contour of an excluded page border became obstacle candidate");
        const auto corrected = mapimport::rectify(drawing, {{{0, 0}, {399, 0}, {399, 299}, {0, 299}}}, 40, 30);
        check(std::abs(static_cast<double>(corrected.cols)/corrected.rows - 40.0/30.0) < 0.005,
              "Rectification must preserve square-cell grid aspect");
        rejects([&]{mapimport::rectify(drawing, {{{-1, 0}, {399, 0}, {399, 299}, {0, 299}}}, 40, 30);},
                "Outside corner accepted");
        rejects([&]{mapimport::rectify(drawing, {{{0, 0}, {0, 299}, {399, 299}, {399, 0}}}, 40, 30);},
                "Wrong corner order accepted");
        std::cout << "Geometry checks passed: " << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Geometry check failed after " << checks << " checks: " << error.what() << '\n';
        return 1;
    }
}
