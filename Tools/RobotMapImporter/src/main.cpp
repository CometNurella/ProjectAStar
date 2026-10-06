#include "map.hpp"
#include "ocr.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif

namespace fs=std::filesystem;
using namespace mapimport;
namespace {
fs::path utf8Path(const std::string& text) {return fs::u8path(text);}
std::string utf8(const fs::path& path) {
    const auto text=path.u8string();return {reinterpret_cast<const char*>(text.data()),text.size()};
}
struct Options {
    fs::path image,output;
    int width=100,height=100,saturation=45,dark=100,gap=1;
    double cellSize=1,coverage=0,minArea=0.001,minConfidence=40;
    std::string mapId="DRAWN_MAP",caseId="DRAWN-C1",tesseract="tesseract",tessdata;
#ifdef MAPIMPORT_WINDOWS_OCR
    std::string ocr="windows";
#else
    std::string ocr="tesseract";
#endif
    bool headless=false,scan=false,manual=false,allowEmpty=false;
    std::optional<cv::Point> start,goal;
    std::optional<std::array<cv::Point2f,4>> corners;
};
void help() {
    std::cout<<"Robot drawing importer (C++17 / OpenCV)\n"
      "--image FILE --output FILE.json [options]\n"
      "  --grid-width N --grid-height N  (default 100 x 100)\n"
      "  --cell-size NUMBER              world units per square cell (default 1)\n"
      "  --map-id ID --case-id ID        default DRAWN_MAP, DRAWN-C1\n"
      "  --ocr windows|tesseract         Windows OCR when built, otherwise Tesseract\n"
      "  --tesseract FILE --tessdata DIR optional Tesseract executable/model directory\n"
      "  --corners TLx TLy TRx TRy BRx BRy BLx BLy (source-image pixels)\n"
      "  --scan                         entire image is the map rectangle\n"
      "  --start-cell X Y --goal-cell X Y (bottom-left cell indices)\n"
      "  --headless                     skip crop/review windows; requires endpoints\n"
      "  --manual                       skip OCR, select obstacles in preview\n"
      "  --allow-empty                  explicitly permit an obstacle-free map\n"
      "  --saturation N --dark N        ink thresholds 0..255 (default 45,100)\n"
      "  --gap N                        close small pen gaps, 0..8 pixels (default 1)\n"
      "  --min-area FRACTION            smallest candidate area / image (default .001)\n"
      "  --coverage FRACTION            occupied fraction per cell; 0=any pixel\n"
      "  --min-confidence NUMBER        Tesseract threshold, default 40\n"
      "Interactive: click corners TL TR BR BL, Enter to continue; S uses full image.\n"
      "Review: O then click toggles obstacle; 1 chooses start; 2 chooses goal.\n"
      "Enter exports after review; R clears crop clicks; Esc cancels.\n";
}
Options parse(int argc,char** argv) {
    Options o;
    auto next=[&](int& i){if(++i>=argc) throw std::invalid_argument("Missing option value.");return std::string(argv[i]);};
    auto number=[&](int& i){const auto s=next(i);std::size_t end=0;const double n=std::stod(s,&end);
        if(end!=s.size()||!std::isfinite(n))throw std::invalid_argument("Expected a finite number: "+s);return n;};
    auto integer=[&](int& i){const double n=number(i);if(n!=std::floor(n)||n< -2147483647.0||n>2147483647.0)
        throw std::invalid_argument("Expected an integer.");return static_cast<int>(n);};
    for(int i=1;i<argc;++i) {
        const std::string a=argv[i];
        if(a=="--image")o.image=utf8Path(next(i));
        else if(a=="--output")o.output=utf8Path(next(i));
        else if(a=="--grid-width")o.width=integer(i);
        else if(a=="--grid-height")o.height=integer(i);
        else if(a=="--cell-size")o.cellSize=number(i);
        else if(a=="--map-id")o.mapId=next(i);
        else if(a=="--case-id")o.caseId=next(i);
        else if(a=="--ocr")o.ocr=next(i);
        else if(a=="--tesseract")o.tesseract=next(i);
        else if(a=="--tessdata")o.tessdata=next(i);
        else if(a=="--saturation")o.saturation=integer(i);
        else if(a=="--dark")o.dark=integer(i);
        else if(a=="--gap")o.gap=integer(i);
        else if(a=="--coverage")o.coverage=number(i);
        else if(a=="--min-area")o.minArea=number(i);
        else if(a=="--min-confidence")o.minConfidence=number(i);
        else if(a=="--start-cell"||a=="--goal-cell") {
            const int x=integer(i),y=integer(i);if(a=="--start-cell")o.start=cv::Point(x,y);else o.goal=cv::Point(x,y);
        }
        else if(a=="--corners") {
            std::array<cv::Point2f,4> p;
            for(auto& v:p) {const double x=number(i),y=number(i);v={static_cast<float>(x),static_cast<float>(y)};}o.corners=p;
        }
        else if(a=="--headless")o.headless=true;
        else if(a=="--scan")o.scan=true;
        else if(a=="--manual")o.manual=true;
        else if(a=="--allow-empty")o.allowEmpty=true;
        else throw std::invalid_argument("Unknown option: "+a);
    }
    if(o.image.empty()||o.output.empty())throw std::invalid_argument("Provide --image and --output; use --help for options.");
    if(o.width<2||o.height<2||o.width>500||o.height>500)throw std::invalid_argument("Grid dimensions must be 2..500.");
    if(o.cellSize<=0||!std::isfinite(o.cellSize*o.width)||!std::isfinite(o.cellSize*o.height))throw std::invalid_argument("Cell size must be positive with finite grid bounds.");
    if(o.saturation<0||o.saturation>255||o.dark<0||o.dark>255||o.gap<0||o.gap>8||
        o.coverage<0||o.coverage>1||o.minArea<=0||o.minArea>=0.5||o.minConfidence<0||o.minConfidence>100)
        throw std::invalid_argument("Threshold/coverage/min-area/confidence setting is outside its supported range.");
    if(o.headless&&(!o.start||!o.goal))throw std::invalid_argument("Headless mode requires --start-cell and --goal-cell.");
    if(o.headless&&o.manual)throw std::invalid_argument("--manual requires the interactive review window.");
    if(o.ocr!="tesseract"&&o.ocr!="windows")throw std::invalid_argument("Choose --ocr windows or --ocr tesseract.");
#ifndef MAPIMPORT_WINDOWS_OCR
    if(o.ocr=="windows")throw std::invalid_argument("This build does not include Windows OCR. Use --ocr tesseract.");
#endif
    validateId(o.mapId);validateId(o.caseId);
    return o;
}

// imdecode/imencode streams also support Unicode paths on Windows.
cv::Mat readImage(const fs::path& path) {
    std::ifstream file(path,std::ios::binary);
    if(!file)throw std::runtime_error("Cannot read image: "+utf8(path));
    file.seekg(0,std::ios::end);const auto length=file.tellg();file.seekg(0);
    if(length<=0||length>128*1024*1024)throw std::runtime_error("Image file must be nonempty and at most 128 MiB.");
    std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
    file.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    if(!file)throw std::runtime_error("Cannot fully read image.");
    cv::Mat image=cv::imdecode(bytes,cv::IMREAD_COLOR);
    if(image.empty())throw std::runtime_error("Unsupported or invalid image.");
    if(image.cols<100||image.rows<100||image.total()>40000000)throw std::runtime_error("Use an image at least 100x100 pixels and at most 40 megapixels.");
    return image;
}
void writeImage(const fs::path& path,const cv::Mat& image) {
    std::vector<unsigned char> bytes;if(!cv::imencode(".png",image,bytes))throw std::runtime_error("PNG encoding failed.");
    std::ofstream file(path,std::ios::binary);file.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    if(!file)throw std::runtime_error("Cannot write diagnostic image: "+utf8(path));
}
void writeJson(const fs::path& path,const nlohmann::json& json) {
    std::ofstream file(path);file<<json.dump(2)<<'\n';if(!file)throw std::runtime_error("Cannot write JSON: "+utf8(path));
}
struct Clicks {double scale=1;std::vector<cv::Point2f> points;};
void cornerClick(int event,int x,int y,int,void* data) {
    auto& state=*static_cast<Clicks*>(data);
    if(event==cv::EVENT_LBUTTONDOWN&&state.points.size()<4)
        state.points.emplace_back(static_cast<float>(x/state.scale),static_cast<float>(y/state.scale));
}
std::array<cv::Point2f,4> fullCorners(const cv::Mat& image) {
    return {cv::Point2f(0,0),cv::Point2f(static_cast<float>(image.cols-1),0),
        cv::Point2f(static_cast<float>(image.cols-1),static_cast<float>(image.rows-1)),cv::Point2f(0,static_cast<float>(image.rows-1))};
}
std::array<cv::Point2f,4> chooseCorners(const cv::Mat& image) {
    Clicks state;state.scale=std::min({1.0,1100.0/image.cols,800.0/image.rows});
    const std::string window="Map rectangle: TL TR BR BL | Enter | S full image | R reset | Esc cancel";
    cv::namedWindow(window,cv::WINDOW_AUTOSIZE);cv::setMouseCallback(window,cornerClick,&state);
    for(;;) {
        cv::Mat shown;cv::resize(image,shown,{},state.scale,state.scale);
        for(std::size_t i=0;i<state.points.size();++i) {
            const cv::Point p(cvRound(state.points[i].x*state.scale),cvRound(state.points[i].y*state.scale));
            cv::circle(shown,p,5,{0,0,255},-1);cv::putText(shown,std::to_string(i+1),p,cv::FONT_HERSHEY_SIMPLEX,0.6,{0,0,255},2);
        }
        cv::imshow(window,shown);const int key=cv::waitKey(30)&255;
        if(key==27||cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1)throw std::runtime_error("Cancelled; no robot JSON exported.");
        if(key=='r'||key=='R')state.points.clear();
        if(key=='s'||key=='S') {cv::destroyWindow(window);return fullCorners(image);}
        if((key==10||key==13)&&state.points.size()==4) {
            std::array<cv::Point2f,4> p;std::copy(state.points.begin(),state.points.end(),p.begin());cv::destroyWindow(window);return p;
        }
    }
}
cv::Mat preview(const cv::Mat& image,const cv::Mat& grid,const std::vector<Shape>& shapes,
                const std::optional<cv::Point>& start,const std::optional<cv::Point>& goal) {
    cv::Mat result=image.clone();
    cv::Mat expanded;cv::resize(grid,expanded,image.size(),0,0,cv::INTER_NEAREST);
    cv::Mat tint=image.clone();tint.setTo(cv::Scalar(60,60,255),expanded);cv::addWeighted(tint,0.4,result,0.6,0,result);
    for(std::size_t i=0;i<shapes.size();++i) {
        const std::vector<std::vector<cv::Point>> contour{shapes[i].contour};
        cv::drawContours(result,contour,0,shapes[i].selected?cv::Scalar(0,0,255):cv::Scalar(0,170,200),2);
        cv::putText(result,std::to_string(i+1),shapes[i].bounds.tl(),cv::FONT_HERSHEY_SIMPLEX,0.5,{20,20,20},1);
    }
    for(const auto& endpoint:{std::make_pair(start,cv::Scalar(20,170,20)),std::make_pair(goal,cv::Scalar(200,0,180))})
        if(endpoint.first) {
            const auto p=*endpoint.first;
            cv::Point pixel(cvRound((p.x+0.5)*image.cols/grid.cols),cvRound((grid.rows-1-p.y+0.5)*image.rows/grid.rows));
            cv::circle(result,pixel,8,endpoint.second,3);
        }
    return result;
}
struct Review {double scale=1;int mode=0,width=100,height=100;cv::Size size;std::vector<Shape>* shapes;
    std::optional<cv::Point>* start;std::optional<cv::Point>* goal;};
void reviewClick(int event,int x,int y,int,void* data) {
    if(event!=cv::EVENT_LBUTTONDOWN)return;
    auto& r=*static_cast<Review*>(data);const cv::Point2f p(static_cast<float>(x/r.scale),static_cast<float>(y/r.scale));
    if(p.x<0||p.y<0||p.x>=r.size.width||p.y>=r.size.height)return;
    if(r.mode==0) {
        const int i=shapeAt(*r.shapes,p);if(i>=0) {auto& s=(*r.shapes)[i];s.selected=!s.selected;s.reason="manual review";}
    } else {
        const cv::Point cell(static_cast<int>(p.x*r.width/r.size.width),r.height-1-static_cast<int>(p.y*r.height/r.size.height));
        if(r.mode==1)*r.start=cell;else *r.goal=cell;
    }
}
void review(const cv::Mat& image,std::vector<Shape>& shapes,Options& o) {
    Review state{std::min({1.0,1100.0/image.cols,800.0/image.rows}),0,o.width,o.height,image.size(),&shapes,&o.start,&o.goal};
    const std::string window="Review: O toggle obstacles | 1 start | 2 goal | Enter export | Esc cancel";
    cv::namedWindow(window,cv::WINDOW_AUTOSIZE);cv::setMouseCallback(window,reviewClick,&state);
    for(;;) {
        const auto grid=occupancy(obstacleMask(image.size(),shapes),o.width,o.height,o.coverage);
        auto shown=preview(image,grid,shapes,o.start,o.goal);cv::resize(shown,shown,{},state.scale,state.scale);cv::imshow(window,shown);
        const int key=cv::waitKey(30)&255;
        if(key==27||cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1)throw std::runtime_error("Cancelled; no robot JSON exported.");
        if(key=='o'||key=='O')state.mode=0;else if(key=='1')state.mode=1;else if(key=='2')state.mode=2;
        else if(key==10||key==13) {
            try {
                if(!o.start||!o.goal)throw std::invalid_argument("Choose start (1) and goal (2) first.");
                if(cv::countNonZero(grid)==0&&!o.allowEmpty)throw std::invalid_argument("Select at least one obstacle, or restart with --allow-empty for an intentionally empty map.");
                robotJson(grid,o.cellSize,o.mapId,o.caseId,*o.start,*o.goal);cv::destroyWindow(window);return;
            } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';}
        }
    }
}
}

int run(int argc,char** argv) {
    try {
        if(argc==1||(argc==2&&std::string(argv[1])=="--help")) {help();return 0;}
        auto o=parse(argc,argv);
        if(fs::absolute(o.image).lexically_normal()==fs::absolute(o.output).lexically_normal() ||
            (fs::exists(o.image)&&fs::exists(o.output)&&fs::equivalent(o.image,o.output)))
            throw std::invalid_argument("The output JSON must use a different file from the source image.");
        const auto source=readImage(o.image);
        const auto corners=o.corners?*o.corners:(o.scan||o.headless?fullCorners(source):chooseCorners(source));
        const auto image=rectify(source,corners,o.width,o.height);
        auto detection=detect(image,o.saturation,o.dark,o.gap,o.minArea);
        const auto output=fs::absolute(o.output);
        const auto diagnostics=output.parent_path()/utf8Path(utf8(output.stem())+"_review");
        fs::create_directories(diagnostics);
        writeImage(diagnostics/"rectified.png",image);writeImage(diagnostics/"ink.png",detection.ink);
        std::vector<Word> words;
        if(!o.manual) {
#ifdef MAPIMPORT_WINDOWS_OCR
            if(o.ocr=="windows")words=recognizeWindows(image);
            else
#endif
                words=recognize(image,diagnostics,o.tesseract,o.tessdata);
        }
        nlohmann::json report={{"source",utf8(fs::absolute(o.image))},{"ocr",o.manual?"manual":o.ocr},
            {"corners",nlohmann::json::array()},{"words",nlohmann::json::array()},{"unmatched_labels",0},{"exported",false}};
        for(auto p:corners)report["corners"].push_back({p.x,p.y});
        int unmatched=0,accepted=0;
        for(const auto& word:words) {
            const bool label=isObstacleWord(word.text);
            const bool confident=word.confidence<0||word.confidence>=o.minConfidence;
            const int shape=label&&confident?enclosingShape(detection.shapes,word.box):-1;
            report["words"].push_back({{"text",word.text},{"confidence",word.confidence<0?nlohmann::json(nullptr):nlohmann::json(word.confidence)},
                {"box",{word.box.x,word.box.y,word.box.width,word.box.height}},{"obstacle_label",label},{"shape",shape}});
            if(label&&shape>=0) {auto& s=detection.shapes[shape];s.selected=true;s.reason="OCR OBSTACLE";++accepted;}
            else if(label)++unmatched;
        }
        report["unmatched_labels"]=unmatched;report["accepted_labels"]=accepted;
        writeJson(diagnostics/"report.json",report);
        std::cout<<detection.shapes.size()<<" candidate shapes, "<<accepted<<" associated OBSTACLE labels, "<<unmatched<<" unmatched/low-confidence labels.\n";
        auto grid=occupancy(obstacleMask(image.size(),detection.shapes),o.width,o.height,o.coverage);
        writeImage(diagnostics/"preview.png",preview(image,grid,detection.shapes,o.start,o.goal));
        if(o.headless&&unmatched>0)throw std::runtime_error("An OBSTACLE label could not be assigned confidently. Inspect the report and use interactive review.");
        if(!o.headless)review(image,detection.shapes,o);
        const auto mask=obstacleMask(image.size(),detection.shapes);
        grid=occupancy(mask,o.width,o.height,o.coverage);
        if(cv::countNonZero(grid)==0&&!o.allowEmpty)throw std::runtime_error("No obstacles selected. Inspect preview/use interactive review; --allow-empty explicitly permits an empty map.");
        const auto json=robotJson(grid,o.cellSize,o.mapId,o.caseId,*o.start,*o.goal);
        report["selected_shapes"]=nlohmann::json::array();
        for(std::size_t i=0;i<detection.shapes.size();++i)if(detection.shapes[i].selected)
            report["selected_shapes"].push_back({{"index",i},{"reason",detection.shapes[i].reason},{"area",detection.shapes[i].area}});
        report["blocked_cells"]=cv::countNonZero(grid);
        writeImage(diagnostics/"obstacles.png",mask);writeImage(diagnostics/"grid.png",grid);
        writeImage(diagnostics/"preview.png",preview(image,grid,detection.shapes,o.start,o.goal));
        // Robot JSON is written only after endpoint and obstacle review checks succeed.
        writeJson(output,json);
        report["exported"]=true;
        writeJson(diagnostics/"report.json",report);
        std::cout<<"Exported "<<utf8(output)<<" ("<<cv::countNonZero(grid)<<" blocked cells).\nReview images: "<<utf8(diagnostics)<<'\n';
        return 0;
    } catch(const std::exception& error) {std::cerr<<"Map import failed: "<<error.what()<<'\n';return 1;}
}

#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {
    // Read native Unicode arguments, then use UTF-8 consistently in the importer.
    std::vector<std::string> text;
    std::vector<char*> arguments;
    text.reserve(argc);arguments.reserve(argc);
    for(int i=0;i<argc;++i) {
        const int length=WideCharToMultiByte(CP_UTF8,0,argv[i],-1,nullptr,0,nullptr,nullptr);
        if(length<=0) {std::cerr<<"Cannot decode command arguments.\n";return 1;}
        std::string converted(static_cast<std::size_t>(length),'\0');
        WideCharToMultiByte(CP_UTF8,0,argv[i],-1,converted.data(),length,nullptr,nullptr);
        converted.pop_back();text.push_back(std::move(converted));
    }
    for(auto& item:text)arguments.push_back(item.data());
    return run(argc,arguments.data());
}
#else
int main(int argc,char** argv) {return run(argc,argv);}
#endif
