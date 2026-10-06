#pragma once
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <array>
#include <string>
#include <vector>

namespace mapimport {
struct Shape {
    std::vector<cv::Point> contour;
    cv::Rect bounds;
    double area = 0;
    bool selected = false;
    std::string reason;
};
struct Detection {
    cv::Mat ink;
    std::vector<Shape> shapes;
};
// Corners are image pixels in TL, TR, BR, BL order; output is square-cell grid aspect.
cv::Mat rectify(const cv::Mat& image, const std::array<cv::Point2f, 4>& corners,
                int gridWidth, int gridHeight);
Detection detect(const cv::Mat& image, int saturation, int dark, int gapPixels,
                 double minAreaFraction);
int enclosingShape(const std::vector<Shape>& shapes, const cv::Rect& label);
int shapeAt(const std::vector<Shape>& shapes, cv::Point2f point);
cv::Mat obstacleMask(cv::Size size, const std::vector<Shape>& shapes);
cv::Mat occupancy(const cv::Mat& mask, int width, int height, double coverage);
nlohmann::json robotJson(const cv::Mat& grid, double cellSize, const std::string& mapId,
                         const std::string& caseId, cv::Point start, cv::Point goal);
void validateId(const std::string& id);
}
