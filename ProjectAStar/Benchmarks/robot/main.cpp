// App 1 uses the established shared core in ../Core/AstarCore.
// The legacy App1/AStar and App1/Grid files are retained as imported references;
// this executable does not compile or call their separate search implementation.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "../../AstarCore/Core/AStar.h"
#include "../../AstarCore/Grid/Grid.h"
#include "../../AstarCore/Heuristics/GridHeuristics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
constexpr int WIDTH = 100;
constexpr int HEIGHT = 100;
constexpr int RUN_COUNT = 10;

struct Point {
    int x = 0;
    int y = 0;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Point& other) const { return !(*this == other); }
    bool operator<(const Point& other) const {
        return x != other.x ? x < other.x : y < other.y;
    }
};

struct Rect { int x1, y1, x2, y2; };
struct TestCaseConfig {
    std::string mapName;
    std::string caseId;
    std::string description;
    Point start;
    Point goal;
    std::vector<Rect> obstacles;
};

enum class ApplicationStatus { OK, INVALID, NO_PATH };
const char* statusName(ApplicationStatus status) {
    switch (status) {
    case ApplicationStatus::OK: return "STATUS_OK";
    case ApplicationStatus::INVALID: return "STATUS_INVALID";
    case ApplicationStatus::NO_PATH: return "STATUS_NO_PATH";
    }
    return "STATUS_INVALID";
}

struct Configuration {
    std::string id;
    int connectivity;
    std::string heuristicName;
};
Configuration configurationFor(const std::string& id) {
    if (id == "E1") return { id,8,"octile" };
    if (id == "E2") return { id,8,"zero (Dijkstra)" };
    if (id == "E3") return { id,4,"Manhattan" };
    throw std::invalid_argument("Configuration must be E1, E2 or E3");
}

struct AStarResult {
    ApplicationStatus status{ ApplicationStatus::INVALID };
    std::string reason;
    std::optional<double> path_length;
    std::size_t n_turns{ 0 };
    std::optional<double> time_ms;
    std::vector<double> search_times_ms;
    std::size_t nodes_expanded{ 0 };
    std::size_t unique_obstacles{ 0 };
    std::vector<Point> path;
    std::size_t totalSteps() const { return path.empty() ? 0 : path.size() - 1; }
};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
bool approximatelyEqual(double a, double b, double tolerance = 1e-8) {
    return std::abs(a - b) <= tolerance;
}

std::size_t countTurns(const std::vector<Point>& path) {
    std::size_t turns = 0;
    for (std::size_t i = 2; i < path.size(); ++i) {
        const int dx1 = path[i - 1].x - path[i - 2].x;
        const int dy1 = path[i - 1].y - path[i - 2].y;
        const int dx2 = path[i].x - path[i - 1].x;
        const int dy2 = path[i].y - path[i - 1].y;
        if (dx1 != dx2 || dy1 != dy2) ++turns;
    }
    return turns;
}

void setObstacles(astar::Grid& grid, const std::vector<Rect>& obstacles) {
    for (const auto& rectangle : obstacles) {
        if (rectangle.x1 > rectangle.x2 || rectangle.y1 > rectangle.y2
            || !grid.isInside(rectangle.x1, rectangle.y1)
            || !grid.isInside(rectangle.x2, rectangle.y2)) {
            throw std::invalid_argument("Obstacle rectangle must have ordered inclusive bounds inside the grid");
        }
        for (int y = rectangle.y1; y <= rectangle.y2; ++y) {
            for (int x = rectangle.x1; x <= rectangle.x2; ++x) {
                grid.setWalkable(x, y, false);
            }
        }
    }
}

std::size_t countUniqueObstacles(const astar::Grid& grid) {
    std::size_t count = 0;
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (!grid.isWalkable(x, y)) ++count;
        }
    }
    return count;
}

void validatePath(const TestCaseConfig& test, const astar::Grid& grid,
    const Configuration& configuration, const AStarResult& result) {
    if (result.status != ApplicationStatus::OK) {
        require(result.path.empty() && !result.path_length, "Failure must have an empty path and absent cost");
        return;
    }
    require(!result.path.empty() && result.path.front() == test.start && result.path.back() == test.goal,
        "Path endpoints disagree with the case: " + test.caseId);
    double sum = 0.0;
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        const auto current = result.path[i];
        require(grid.isWalkable(current.x, current.y), "Path contains a blocked cell");
        if (i == 0) continue;
        const auto previous = result.path[i - 1];
        const int dx = current.x - previous.x;
        const int dy = current.y - previous.y;
        require(std::abs(dx) <= 1 && std::abs(dy) <= 1 && (dx || dy), "Path contains a non-adjacent move");
        if (dx && dy) {
            require(configuration.connectivity == 8, "Four-direction path contains a diagonal");
            require(grid.isWalkable(previous.x + dx, previous.y)
                && grid.isWalkable(previous.x, previous.y + dy), "Path cuts an obstacle corner");
            sum += std::sqrt(2.0);
        }
        else {
            sum += 1.0;
        }
    }
    require(result.path_length && approximatelyEqual(sum, *result.path_length), "Path sum differs from core cost");
}

AStarResult runBenchmark(const TestCaseConfig& test, const Configuration& configuration,
    int runCount = RUN_COUNT) {
    AStarResult result;
    astar::Grid grid(WIDTH, HEIGHT, configuration.connectivity);
    // Graph construction, inclusive rectangle validation and endpoint checks
    // are intentionally outside the measured core-search interval.
    try {
        setObstacles(grid, test.obstacles);
    }
    catch (const std::invalid_argument& error) {
        result.reason = error.what();
        return result;
    }
    result.unique_obstacles = countUniqueObstacles(grid);
    if (!grid.isInside(test.start.x, test.start.y) || !grid.isInside(test.goal.x, test.goal.y)) {
        result.reason = "Start or goal is outside the grid";
        return result;
    }
    if (!grid.isWalkable(test.start.x, test.start.y) || !grid.isWalkable(test.goal.x, test.goal.y)) {
        result.reason = "Start or goal is blocked";
        return result;
    }
    // Even start==goal reaches the shared core after validation: it returns one
    // path cell, zero moves, zero cost and zero expansions.
    const auto start = grid.toNodeId(test.start.x, test.start.y);
    const auto goal = grid.toNodeId(test.goal.x, test.goal.y);
    const astar::Heuristic heuristic = configuration.id == "E1"
        ? astar::grid_heuristics::octile(grid)
        : configuration.id == "E2" ? astar::grid_heuristics::zero()
        : astar::grid_heuristics::manhattan(grid);
    astar::SearchResult first;
    for (int repetition = 0; repetition < runCount; ++repetition) {
        const astar::Astar search;
        const auto begin = std::chrono::steady_clock::now();
        const auto coreResult = search.findPath(grid, start, goal, heuristic);
        const auto end = std::chrono::steady_clock::now();
        result.search_times_ms.push_back(std::chrono::duration<double, std::milli>(end - begin).count());
        if (repetition == 0) {
            first = coreResult;
        }
        else {
            require(first.found == coreResult.found && first.path == coreResult.path
                && first.expandedNodes == coreResult.expandedNodes
                && (!first.found || approximatelyEqual(first.cost, coreResult.cost)),
                "Repeated fresh searches changed the result: " + test.caseId);
        }
    }
    result.time_ms = std::accumulate(result.search_times_ms.begin(), result.search_times_ms.end(), 0.0)
        / static_cast<double>(result.search_times_ms.size());
    result.nodes_expanded = first.expandedNodes;
    result.status = first.found ? ApplicationStatus::OK : ApplicationStatus::NO_PATH;
    if (first.found) {
        result.path_length = first.cost;
        for (auto id : first.path) {
            const auto point = grid.toGridNode(id);
            result.path.push_back({ point.x,point.y });
        }
        result.n_turns = countTurns(result.path);
    }
    // Turn counting, path validation and serialization are outside the timer.
    validatePath(test, grid, configuration, result);
    return result;
}

std::string jsonString(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<unsigned>(c) << std::dec << std::setfill(' ');
            }
            else out << static_cast<char>(c);
        }
    }
    out << '"';
    return out.str();
}
std::string number(const std::optional<double>& value, const char* absent = "null") {
    if (!value) return absent;
    std::ostringstream out;
    out << std::setprecision(17) << *value;
    return out.str();
}

std::string serializeJson(const TestCaseConfig& test, const Configuration& configuration,
    const AStarResult& result) {
    std::ostringstream out;
    out << "{\n";
    bool first = true;
    const auto field = [&](const std::string& name, const std::string& value) {
        if (!first) out << ",\n";
        first = false;
        out << "  " << jsonString(name) << ": " << value;
        };
    const auto text = [&](const std::string& name, const std::string& value) { field(name, jsonString(value)); };
    const auto integer = [&](const std::string& name, std::size_t value) { field(name, std::to_string(value)); };
    integer("schema_version", 2);
    text("application", "APP1");
    text("map", test.mapName);
    text("case", test.caseId);
    text("description", test.description);
    text("input_source", "original Code 2 App1 built-in fixtures; endpoints and obstacles preserved");
    text("search_core", "shared Core/AstarCore astar::Astar::findPath");
    text("configuration", configuration.id);
    text("algorithm", configuration.id == "E2" ? "Dijkstra" : "A*");
    text("heuristic", configuration.heuristicName);
    integer("connectivity", configuration.connectivity);
    text("origin", "bottom-left");
    integer("width", WIDTH);
    integer("height", HEIGHT);
    field("cell_size", "1");
    field("corner_cutting", "false");
    field("start", "[" + std::to_string(test.start.x) + ", " + std::to_string(test.start.y) + "]");
    field("goal", "[" + std::to_string(test.goal.x) + ", " + std::to_string(test.goal.y) + "]");
    text("status", statusName(result.status));
    text("reason", result.reason);
    field("success", result.status == ApplicationStatus::OK ? "true" : "false");
    field("path_length", number(result.path_length));
    field("cost", number(result.path_length));
    integer("n_turns", result.n_turns);
    integer("turns", result.n_turns);
    integer("total_steps", result.totalSteps());
    integer("path_cell_count", result.path.size());
    integer("unique_obstacle_cells", result.unique_obstacles);
    integer("nodes_expanded", result.nodes_expanded);
    integer("expanded_nodes", result.nodes_expanded);
    integer("evaluated_nodes", result.nodes_expanded);
    text("expansion_definition", "neighbor processing events; goal selection and stale entries excluded; re-expansions counted");
    text("tie_breaking", "minimum f, minimum h, FIFO insertion order");
    integer("timing_runs", result.search_times_ms.size());
    text("timing_clock", "std::chrono::steady_clock");
    text("timing_scope", "Astar::findPath only; fresh internal search state on every call");
    text("timing_aggregation", "arithmetic mean");
    field("execution_time_ms", number(result.time_ms));
    std::ostringstream times;
    times << '[';
    for (std::size_t i = 0; i < result.search_times_ms.size(); ++i) {
        if (i) times << ", ";
        times << std::setprecision(17) << result.search_times_ms[i];
    }
    times << ']';
    field("search_times_ms", times.str());
    std::ostringstream path;
    path << '[';
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        if (i) path << ", ";
        path << '[' << result.path[i].x << ", " << result.path[i].y << ']';
    }
    path << ']';
    field("path", path.str());
    out << "\n}\n";
    return out.str();
}

std::string serializeLog(const TestCaseConfig& test, const Configuration& configuration,
    const AStarResult& result) {
    std::ostringstream out;
    out << "======================================================\n"
        << "Map: " << test.mapName << " | Case: " << test.caseId << '\n'
        << "Description: " << test.description << '\n'
        << "input_source: original Code 2 App1 built-in fixtures; endpoints and obstacles preserved\n"
        << "search_core: shared Core/AstarCore astar::Astar::findPath\n"
        << "configuration: " << configuration.id << '\n'
        << "algorithm: " << (configuration.id == "E2" ? "Dijkstra" : "A*") << '\n'
        << "heuristic: " << configuration.heuristicName << '\n'
        << "connectivity: " << configuration.connectivity << '\n'
        << "start: (" << test.start.x << ',' << test.start.y << ")\n"
        << "goal: (" << test.goal.x << ',' << test.goal.y << ")\n"
        << "status: " << statusName(result.status) << '\n'
        << "reason: " << result.reason << '\n'
        << "path_length: " << number(result.path_length, "N/A") << '\n'
        << "n_turns: " << result.n_turns << '\n'
        << "turns: " << result.n_turns << '\n'
        << "total_steps: " << result.totalSteps() << '\n'
        << "path_cell_count: " << result.path.size() << '\n'
        << "unique_obstacle_cells: " << result.unique_obstacles << '\n'
        << "nodes_expanded: " << result.nodes_expanded << '\n'
        << "expanded_nodes: " << result.nodes_expanded << '\n'
        << "evaluated_nodes: " << result.nodes_expanded << '\n'
        << "expansion_definition: neighbor processing events; goal selection and stale entries excluded; re-expansions counted\n"
        << "tie_breaking: minimum f, minimum h, FIFO insertion order\n"
        << "timing_runs: " << result.search_times_ms.size() << '\n'
        << "timing_clock: std::chrono::steady_clock\n"
        << "timing_scope: Astar::findPath only; fresh internal search state on every call\n"
        << "timing_aggregation: arithmetic mean\n"
        << "execution_time_ms: " << number(result.time_ms, "N/A") << '\n'
        << "search_times_ms: [";
    for (std::size_t i = 0; i < result.search_times_ms.size(); ++i) {
        if (i) out << ", ";
        out << std::setprecision(17) << result.search_times_ms[i];
    }
    out << "]\npath: ";
    if (result.path.empty()) out << "[]";
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        if (i) out << " -> ";
        out << '(' << result.path[i].x << ',' << result.path[i].y << ')';
    }
    out << '\n';
    if (result.status != ApplicationStatus::INVALID) {
        astar::Grid grid(WIDTH, HEIGHT, configuration.connectivity);
        setObstacles(grid, test.obstacles);
        int minX = std::min(test.start.x, test.goal.x), maxX = std::max(test.start.x, test.goal.x);
        int minY = std::min(test.start.y, test.goal.y), maxY = std::max(test.start.y, test.goal.y);
        for (const auto& point : result.path) {
            minX = std::min(minX, point.x); maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y); maxY = std::max(maxY, point.y);
        }
        minX = std::max(0, minX - 2); maxX = std::min(WIDTH - 1, maxX + 2);
        minY = std::max(0, minY - 2); maxY = std::min(HEIGHT - 1, maxY + 2);
        const std::set<Point> pathSet(result.path.begin(), result.path.end());
        out << "\nASCII crop: x=" << minX << ".." << maxX << ", y=" << minY << ".." << maxY << '\n'
            << "Bottom-left origin: y increases upward; rows print from high y to low y.\n"
            << "Legend: S start, G goal, @ same-cell endpoints, # obstacle, * path, . free\n";
        for (int y = maxY; y >= minY; --y) {
            out << std::setw(2) << y << " | ";
            for (int x = minX; x <= maxX; ++x) {
                const Point point{ x,y };
                if (point == test.start && point == test.goal) out << "@ ";
                else if (point == test.start) out << "S ";
                else if (point == test.goal) out << "G ";
                else if (!grid.isWalkable(x, y)) out << "# ";
                else if (pathSet.count(point)) out << "* ";
                else out << ". ";
            }
            out << '\n';
        }
    }
    out << "======================================================\n";
    return out.str();
}

void writeFile(const fs::path& path, const std::string& contents) {
    std::ofstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot create output file: " + path.string());
    file << contents;
    if (!file) throw std::runtime_error("Cannot write output file: " + path.string());
}

void executeAndVisualize(const TestCaseConfig& test, const Configuration& configuration,
    const fs::path& output) {
    const auto result = runBenchmark(test, configuration);
    const std::string log = serializeLog(test, configuration, result);
    fs::create_directories(output / "result");
    fs::create_directories(output / "log");
    writeFile(output / "result" / ("benchmark_result_" + test.caseId + ".json"), serializeJson(test, configuration, result));
    writeFile(output / "log" / ("benchmark_log_" + test.caseId + ".txt"), log);
    std::cout << log;
}

void selfTest(const std::vector<TestCaseConfig>& cases) {
    const std::map<std::string, std::pair<double, double>> expected{
        {"M1-C1",{110.84,129}}, {"M1-C2",{103.84,122}}, {"M1-C3",{109.70,122}},
        {"M2-C1",{154.31,159}}, {"M2-C2",{146.77,162}}, {"M2-C3",{119.31,124}},
        {"M3-C1",{128.15,151}}, {"M3-C2",{183.74,206}},
        {"M3-C3",{145.67,165}}, {"M3-C4",{150.67,170}}, {"EC-1",{0,0}}
    };
    std::size_t checks = 0;
    for (const auto& test : cases) {
        const auto e1 = runBenchmark(test, configurationFor("E1"), 1);
        const auto e2 = runBenchmark(test, configurationFor("E2"), 1);
        const auto e3 = runBenchmark(test, configurationFor("E3"), 1);
        require(e1.status == e2.status && e1.status == e3.status, "Configurations disagree on status"); ++checks;
        if (const auto cost = expected.find(test.caseId); cost != expected.end()) {
            require(e1.status == ApplicationStatus::OK && e1.path_length && e2.path_length && e3.path_length,
                "Expected reachable case failed");
            require(approximatelyEqual(*e1.path_length, cost->second.first, 0.01)
                && approximatelyEqual(*e3.path_length, cost->second.second, 0.01)
                && approximatelyEqual(*e1.path_length, *e2.path_length), "Specification cost mismatch: " + test.caseId); ++checks;
        }
        else {
            const auto expectedStatus = test.caseId == "M3-C5" ? ApplicationStatus::NO_PATH : ApplicationStatus::INVALID;
            require(e1.status == expectedStatus && !e1.path_length && e1.path.empty(), "Failure contract mismatch"); ++checks;
        }
        if (test.caseId == "M1-C1") { require(e1.unique_obstacles == 351, "Overlapping obstacles counted twice"); ++checks; }
        if (test.caseId == "EC-1") {
            require(e1.totalSteps() == 0 && e1.path.size() == 1
                && e1.nodes_expanded == 0 && e1.n_turns == 0, "Same-cell metrics differ"); ++checks;
        }
        if (e1.status == ApplicationStatus::INVALID) {
            require(!e1.time_ms && e1.search_times_ms.empty()
                && e1.nodes_expanded == 0, "Invalid endpoints reached timed search"); ++checks;
        }
    }
    // Validate both endpoints before handling the same-cell shortcut, and reject
    // invalid rectangles rather than silently clipping them.
    for (const TestCaseConfig& regression : {
         TestCaseConfig{"regression","blocked-same-cell","",{31,50},{31,50},{{30,30,32,70}}},
         TestCaseConfig{"regression","out-of-bounds-same-cell","",{150,50},{150,50},{}},
         TestCaseConfig{"regression","blocked-goal","",{50,50},{31,50},{{30,30,32,70}}},
         TestCaseConfig{"regression","out-of-bounds-goal","",{50,50},{-1,50},{}},
         TestCaseConfig{"regression","invalid-rectangle","",{0,0},{1,1},{{2,2,1,1}}} }) {
        const auto result = runBenchmark(regression, configurationFor("E1"), 1);
        require(result.status == ApplicationStatus::INVALID && !result.time_ms && result.path.empty(),
            "Validation regression failed: " + regression.caseId); ++checks;
    }
    for (bool blockX : {false, true}) {
        astar::Grid corner(2, 2, 8);
        corner.setWalkable(blockX ? 1 : 0, blockX ? 0 : 1, false);
        const auto result = astar::Astar{}.findPath(corner, 0, 3, astar::grid_heuristics::octile(corner));
        require(result.found && approximatelyEqual(result.cost, 2), "Diagonal crossed a blocked corner"); ++checks;
    }
    std::cout << "App 1 self-test passed: " << checks << " checks; all 14 original cases and E1/E2/E3 costs.\n";
}

void help() {
    std::cout << "Usage: app1_benchmark [--configuration E1|E2|E3] [--output PATH] [--self-test]\n"
        << "E1: 8 directions + octile; E2: 8 directions + zero; E3: 4 directions + Manhattan.\n"
        << "Default: E1, output/APP1/E1. --output writes PATH/result and PATH/log.\n"
        << "--self-test alone validates without writing benchmarks.\n"
        << "--self-test --output PATH validates and exports the selected configuration.\n"
        << "Each valid case records a mean of 10 fresh findPath calls.\n";
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    try {
        Configuration configuration = configurationFor("E1");
        fs::path output;
        bool outputSpecified = false;
        bool testRequested = false;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") { help(); return 0; }
            if (argument == "--configuration" && i + 1 < argc) configuration = configurationFor(argv[++i]);
            else if (argument == "--output" && i + 1 < argc) { output = argv[++i]; outputSpecified = true; }
            else if (argument == "--self-test") testRequested = true;
            else throw std::invalid_argument("Unknown or incomplete option: " + argument);
        }
        if (!outputSpecified) output = fs::path("output") / "APP1" / configuration.id;

        std::vector<Rect> obsM1 = { {30,30,32,70}, {68,30,70,70}, {30,68,70,70} };
        std::vector<Rect> obsM2 = { {10,45,80,47}, {10,53,80,55}, {78,45,80,55} };
        std::vector<Rect> obsM3 = { {20,20,80,22}, {20,20,22,80}, {20,78,80,80}, {78,20,80,45}, {78,55,80,80}, {35,22,37,65}, {50,35,52,78}, {65,22,67,65}, {80,45,92,47}, {80,53,92,55}, {88,85,98,86}, {88,95,98,96}, {88,85,89,96}, {97,85,98,96} };

        std::vector<TestCaseConfig> testCases = {
            {"M1_U_shape", "M1-C1", "n ngay sau vách kín, đường thẳng bị chặn", {50,60}, {50,85}, obsM1},
            {"M1_U_shape", "M1-C2", "n chéo góc ngoài U", {50,60}, {85,85}, obsM1},
            {"M1_U_shape", "M1-C3", "Ct sát vách trái, n ở phía ngược lại", {40,65}, {15,90}, obsM1},
            {"M2_dead_end", "M2-C1", "Robot ở cuối hành lang cụt", {75,50}, {90,50}, obsM2},
            {"M2_dead_end", "M2-C2", "n phía trên hành lang", {75,50}, {75,80}, obsM2},
            {"M2_dead_end", "M2-C3", "Ct giữa hành lang", {40,50}, {90,50}, obsM2},
            {"M3_complex", "M3-C1", "Robot ở ngăn trong cùng, đi ziczac", {28,70}, {95,50}, obsM3},
            {"M3_complex", "M3-C2", "Ngăn giữa, n ở góc trên trái", {58,30}, {10,90}, obsM3},
            {"M3_complex", "M3-C3", "Ngăn thứ 2, n bên dưới phải", {44,70}, {95,20}, obsM3},
            {"M3_complex", "M3-C4", "Ngăn gần cổ, n ở góc dưới trái", {72,30}, {5,5}, obsM3},
            {"M3_complex", "M3-C5", "n nằm trong phòng kín -> NO_PATH", {28,70}, {93,90}, obsM3},

            // CÁC CASE NGOẠI LỆ (EDGE CASES)
            {"Edge_Cases", "EC-1", "Ct = n (Chi phí 0)", {50,50}, {50,50}, {}},
            {"Edge_Cases", "EC-2", "Ct hoặc n trên vật cản -> INVALID", {31,50}, {50,50}, obsM1},
            {"Edge_Cases", "EC-3", "Ct hoặc n ngoài lưới -> INVALID", {150,50}, {50,50}, {}}
        };


        if (testRequested) {
            selfTest(testCases);
            if (!outputSpecified) return 0;
        }
        for (const auto& test : testCases) executeAndVisualize(test, configuration, output);
        std::cout << "Wrote 14 benchmark JSON/log pairs for " << configuration.id
            << " to " << output.string() << '\n';
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "App 1 benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
