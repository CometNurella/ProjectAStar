#include "map.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mapimport {
cv::Mat rectify(const cv::Mat& image, const std::array<cv::Point2f, 4>& corners,
                int gridWidth, int gridHeight) {
    std::vector<cv::Point2f> polygon(corners.begin(), corners.end());
    for (auto p : polygon)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
            p.x >= image.cols || p.y >= image.rows)
            throw std::invalid_argument("Map corners must lie inside the source image.");
    if (!cv::isContourConvex(polygon) || cv::contourArea(polygon, true) < 100)
        throw std::invalid_argument("Corners must form a nondegenerate clockwise TL, TR, BR, BL quadrilateral.");
    const double sourceWidth = (cv::norm(corners[1]-corners[0]) + cv::norm(corners[2]-corners[3])) / 2;
    const double sourceHeight = (cv::norm(corners[3]-corners[0]) + cv::norm(corners[2]-corners[1])) / 2;
    // Grid ratio defines map geometry, rather than assuming a photographed page is square.
    const double pixelsPerCell = std::min(
        std::clamp(std::max(sourceWidth/gridWidth, sourceHeight/gridHeight), 4.0, 24.0),
        std::sqrt(4000000.0/(static_cast<double>(gridWidth)*gridHeight)));
    const int width = static_cast<int>(std::ceil(gridWidth*pixelsPerCell));
    const int height = static_cast<int>(std::ceil(gridHeight*pixelsPerCell));
    std::array<cv::Point2f,4> target{cv::Point2f(0,0),cv::Point2f(static_cast<float>(width-1),0),
        cv::Point2f(static_cast<float>(width-1),static_cast<float>(height-1)),cv::Point2f(0,static_cast<float>(height-1))};
    cv::Mat result;
    cv::warpPerspective(image, result, cv::getPerspectiveTransform(corners.data(),target.data()),
                        {width,height},cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
    return result;
}

Detection detect(const cv::Mat& image, int saturation, int dark, int gapPixels,
                 double minAreaFraction) {
    cv::Mat hsv, gray, color, shadowCorrected, darkInk;
    cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
    cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> channels;
    cv::split(hsv,channels);
    cv::threshold(channels[1],color,saturation,255,cv::THRESH_BINARY);
    cv::threshold(gray,darkInk,dark,255,cv::THRESH_BINARY_INV);
    // Local contrast supplements the color mask under uneven lighting.
    cv::adaptiveThreshold(gray,shadowCorrected,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV,51,12);
    Detection result;
    cv::bitwise_or(color,darkInk,result.ink);
    cv::bitwise_or(result.ink,shadowCorrected,result.ink);
    if (gapPixels > 0)
        cv::morphologyEx(result.ink,result.ink,cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE,{2*gapPixels+1,2*gapPixels+1}));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(result.ink.clone(),contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    const double minArea = std::max(100.0,static_cast<double>(image.total())*minAreaFraction);
    for (std::size_t i=0;i<contours.size();++i) {
        const double area = std::abs(cv::contourArea(contours[i]));
        const auto box = cv::boundingRect(contours[i]);
        if (area < minArea || area > image.total()*0.90 || box.width < 20 || box.height < 20) continue;
        if (box.x<=0 || box.y<=0 || box.br().x>=image.cols || box.br().y>=image.rows) continue;
        if (box.width>image.cols*0.85 && box.height>image.rows*0.85) continue;
        // Inner/outer edges of one pen outline describe one object. Keep its outer edge.
        const int parent=hierarchy[i][3];
        if (parent >= 0) {
            const double parentArea=std::abs(cv::contourArea(contours[parent]));
            if (parentArea < image.total()*0.90 && parentArea/area < 1.35) continue;
        }
        result.shapes.push_back({contours[i],box,area,false,{}});
    }
    std::sort(result.shapes.begin(),result.shapes.end(),[](const auto& a,const auto& b){return a.area<b.area;});
    return result;
}

int shapeAt(const std::vector<Shape>& shapes, cv::Point2f point) {
    for (std::size_t i=0;i<shapes.size();++i)
        if (cv::pointPolygonTest(shapes[i].contour,point,false)>=0) return static_cast<int>(i);
    return -1;
}

int enclosingShape(const std::vector<Shape>& shapes,const cv::Rect& label) {
    const std::array<cv::Point2f,5> samples{cv::Point2f(static_cast<float>(label.x),static_cast<float>(label.y)),
        cv::Point2f(static_cast<float>(label.x+label.width-1),static_cast<float>(label.y)),
        cv::Point2f(static_cast<float>(label.x),static_cast<float>(label.y+label.height-1)),
        cv::Point2f(static_cast<float>(label.x+label.width-1),static_cast<float>(label.y+label.height-1)),
        cv::Point2f(label.x+label.width/2.0f,label.y+label.height/2.0f)};
    for (std::size_t i=0;i<shapes.size();++i) {
        const auto& shape=shapes[i];
        if (shape.area < label.area()*1.5) continue;
        bool contains=true;
        for (auto p:samples) if (cv::pointPolygonTest(shape.contour,p,false)<0) {contains=false;break;}
        if (contains) {
            // A concave notch can cross the middle of the word even when all corners fit.
            const cv::Rect local(label.x-shape.bounds.x,label.y-shape.bounds.y,label.width,label.height);
            if (local.x<0 || local.y<0 || local.br().x>shape.bounds.width || local.br().y>shape.bounds.height) continue;
            cv::Mat filled=cv::Mat::zeros(shape.bounds.size(),CV_8U);
            std::vector<std::vector<cv::Point>> contour{shape.contour};
            cv::drawContours(filled,contour,0,cv::Scalar(255),cv::FILLED,cv::LINE_8,cv::noArray(),0,-shape.bounds.tl());
            if(cv::countNonZero(filled(local))==label.area()) return static_cast<int>(i);
        }
    }
    return -1;
}

cv::Mat obstacleMask(cv::Size size,const std::vector<Shape>& shapes) {
    cv::Mat mask=cv::Mat::zeros(size,CV_8U);
    for(const auto& shape:shapes) if(shape.selected) {
        std::vector<std::vector<cv::Point>> contour{shape.contour};
        cv::drawContours(mask,contour,0,cv::Scalar(255),cv::FILLED);
    }
    return mask;
}

cv::Mat occupancy(const cv::Mat& mask,int width,int height,double coverage) {
    cv::Mat grid=cv::Mat::zeros(height,width,CV_8U);
    for(int row=0;row<height;++row) for(int x=0;x<width;++x) {
        const int x0=x*mask.cols/width, x1=(x+1)*mask.cols/width;
        const int y0=row*mask.rows/height, y1=(row+1)*mask.rows/height;
        const cv::Rect block(x0,y0,x1-x0,y1-y0);
        const double fraction=static_cast<double>(cv::countNonZero(mask(block)))/block.area();
        if ((coverage==0 && fraction>0) || (coverage>0 && fraction>=coverage)) grid.at<unsigned char>(row,x)=255;
    }
    return grid;
}

void validateId(const std::string& id) {
    if(id.empty() || id.size()>128) throw std::invalid_argument("Map/case IDs need 1-128 ASCII letters, digits, '_' or '-'.");
    std::string upper;
    for(unsigned char c:id) {
        if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'))
            throw std::invalid_argument("Map/case IDs need ASCII letters, digits, '_' or '-'.");
        upper.push_back(static_cast<char>(c>='a'&&c<='z'?c-'a'+'A':c));
    }
    if(upper=="CON"||upper=="PRN"||upper=="AUX"||upper=="NUL" ||
       (upper.size()==4&&(upper.substr(0,3)=="COM"||upper.substr(0,3)=="LPT")&&upper[3]>='1'&&upper[3]<='9'))
        throw std::invalid_argument("Map/case ID cannot be a Windows device name.");
}

nlohmann::json robotJson(const cv::Mat& grid,double cellSize,const std::string& mapId,
                         const std::string& caseId,cv::Point start,cv::Point goal) {
    validateId(mapId);validateId(caseId);
    if(!std::isfinite(cellSize)||cellSize<=0||!std::isfinite(cellSize*grid.cols)||!std::isfinite(cellSize*grid.rows))
        throw std::invalid_argument("Cell size must be positive and produce finite world bounds.");
    for(auto p:{start,goal}) {
        if(p.x<0||p.y<0||p.x>=grid.cols||p.y>=grid.rows) throw std::invalid_argument("Start/goal cell is outside the grid.");
        if(grid.at<unsigned char>(grid.rows-1-p.y,p.x)) throw std::invalid_argument("Start/goal cell is blocked. Choose a free cell.");
        for(int c:{p.x,p.y}) {
            const double center=(c+0.5)*cellSize;
            if(!std::isfinite(center)||std::floor(center/cellSize)!=c)
                throw std::invalid_argument("Cell size cannot represent the selected cell center reliably.");
        }
    }
    nlohmann::json rectangles=nlohmann::json::array();
    // Exact horizontal runs retain concave objects; no obstacle bounding-box approximation.
    for(int row=0;row<grid.rows;++row) for(int x=0;x<grid.cols;) {
        if(!grid.at<unsigned char>(row,x)) {++x;continue;}
        const int first=x;
        while(x<grid.cols&&grid.at<unsigned char>(row,x)) ++x;
        const int y=grid.rows-1-row;
        rectangles.push_back({first,y,x-1,y});
    }
    return {{"width",grid.cols},{"height",grid.rows},{"cell_size",cellSize},{"origin","bottom-left"},
        {"maps",{{mapId,{{"desc","Map converted from a drawing; selected labeled obstacle contours"},
            {"obstacles_rect_x1y1x2y2",rectangles},{"cases",nlohmann::json::array({
                {{"id",caseId},{"Ct",{(start.x+0.5)*cellSize,(start.y+0.5)*cellSize}},
                 {"n",{(goal.x+0.5)*cellSize,(goal.y+0.5)*cellSize}}}})}}}}}};
}
}
