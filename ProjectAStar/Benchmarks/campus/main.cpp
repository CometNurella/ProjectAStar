#include "../../AstarCore/Core/AStar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace fs = std::filesystem;
struct NodeInfo { std::string name; double x; double y; };
struct CampusEdge { std::string to; double weight; };
using CampusNodes = std::map<std::string, NodeInfo>;
using CampusEdges = std::map<std::string, std::vector<CampusEdge>>;
constexpr double BASELINE_ALPHA = 1.0;
constexpr std::size_t BENCHMARK_RUNS = 10;
constexpr const char* TIMING_SCOPE = "astar::Astar::findPath only; fresh state per call; validation and export excluded";

CampusNodes defaultNodes() {
    return { {"A", {"Main Gate", 0, 0}}, {"B", {"Library", 3, 1}},
            {"C", {"Building A", 1, 4}}, {"D", {"Canteen", 5, 3}},
            {"E", {"Building B", 4, 6}}, {"F", {"Dormitory", 7, 4}},
            {"G", {"Football Field", 8, 1}} };
}
CampusEdges defaultEdges() {
    return { {"A", {{"B", 320}, {"C", 410}}},
            {"B", {{"A", 320}, {"C", 360}, {"D", 280}}},
            {"C", {{"A", 410}, {"B", 360}, {"E", 320}}},
            {"D", {{"B", 280}, {"E", 300}, {"F", 220}, {"G", 420}}},
            {"E", {{"C", 320}, {"D", 300}, {"F", 280}}},
            {"F", {{"D", 220}, {"E", 280}, {"G", 350}}},
            {"G", {{"F", 350}, {"D", 420}}} };
}

// This adapter supplies campus edges to the shared core. It contains no search loop.
class CampusGraph final : public astar::Graph {
    std::map<std::string, astar::NodeId> m_ids;
    std::vector<std::string> m_labels;
    std::vector<NodeInfo> m_nodes;
    std::vector<std::vector<astar::Edge>> m_edges;
public:
    CampusGraph(const CampusNodes& nodes, const CampusEdges& edges) {
        if (nodes.empty()) { throw std::invalid_argument("Campus graph has no nodes"); }
        for (const auto& pair : nodes) {
            if (pair.first.empty() || !std::isfinite(pair.second.x) || !std::isfinite(pair.second.y)) {
                throw std::invalid_argument("Node IDs must be nonempty and coordinates finite");
            }
            m_ids.emplace(pair.first, m_nodes.size());
            m_labels.push_back(pair.first);
            m_nodes.push_back(pair.second);
        }
        m_edges.resize(m_nodes.size());
        for (const auto& pair : edges) {
            if (!contains(pair.first)) { throw std::invalid_argument("Unknown edge source: " + pair.first); }
            for (const auto& edge : pair.second) {
                if (!contains(edge.to)) { throw std::invalid_argument("Unknown edge destination: " + edge.to); }
                if (!std::isfinite(edge.weight) || edge.weight < 0.0) {
                    throw std::invalid_argument("Edge weights must be finite nonnegative distances in meters");
                }
                m_edges.at(nodeId(pair.first)).push_back({ nodeId(edge.to), edge.weight });
            }
        }
    }
    bool contains(const std::string& id) const { return m_ids.find(id) != m_ids.end(); }
    astar::NodeId nodeId(const std::string& id) const { return m_ids.at(id); }
    const std::string& label(astar::NodeId node) const { return m_labels.at(node); }
    std::size_t getNodeCount() const noexcept override { return m_nodes.size(); }
    std::vector<astar::Edge> getEdges(astar::NodeId node) const override { return m_edges.at(node); }
    double euclidean(astar::NodeId from, astar::NodeId to) const {
        return std::hypot(m_nodes.at(from).x - m_nodes.at(to).x, m_nodes.at(from).y - m_nodes.at(to).y);
    }
    void validateBaselineHeuristic() const {
        // Schematic coordinates are not assumed to be a 100-meter grid.
        // w(u,v) >= alpha*d(u,v) guarantees consistency by the triangle inequality.
        for (astar::NodeId from = 0; from < getNodeCount(); ++from) {
            for (const auto& edge : m_edges.at(from)) {
                const double bound = BASELINE_ALPHA * euclidean(from, edge.to);
                if (!std::isfinite(bound) || edge.cost < bound) {
                    throw std::invalid_argument("Edge distance is below alpha=1 Euclidean bound: " + label(from) + " -> " + label(edge.to));
                }
            }
        }
    }
};

struct BenchmarkResult {
    std::string algorithm, heuristic, status = "STATUS_INVALID", message;
    double alpha = 0.0;
    std::optional<double> pathLength, timeMs;
    std::size_t totalSteps = 0, expandedNodes = 0, runCount = 0;
    std::vector<double> searchTimesMs;
    std::vector<std::string> path;
};
BenchmarkResult runBenchmark(const CampusGraph& graph, const std::string& start, const std::string& goal, bool useHeuristic) {
    BenchmarkResult result;
    result.algorithm = useHeuristic ? "A* Algorithm" : "Dijkstra Algorithm";
    result.heuristic = useHeuristic ? "euclidean" : "zero";
    result.alpha = useHeuristic ? BASELINE_ALPHA : 0.0;
    if (!graph.contains(start) || !graph.contains(goal)) {
        result.message = "Start or goal ID is not a campus node";
        return result;
    }
    try { if (useHeuristic) { graph.validateBaselineHeuristic(); } }
    catch (const std::invalid_argument& error) { result.message = error.what(); return result; }
    const auto startId = graph.nodeId(start), goalId = graph.nodeId(goal);
    const astar::Heuristic heuristic = [&graph, useHeuristic](astar::NodeId node, astar::NodeId target) {
        return useHeuristic ? BASELINE_ALPHA * graph.euclidean(node, target) : 0.0;
        };
    const astar::Astar solver;
    astar::SearchResult first;
    double totalMs = 0.0;
    for (std::size_t repetition = 0; repetition < BENCHMARK_RUNS; ++repetition) {
        // findPath constructs fresh OPEN, CLOSED, and per-node state on every call.
        const auto begin = std::chrono::steady_clock::now();
        const astar::SearchResult current = solver.findPath(graph, startId, goalId, heuristic);
        const auto end = std::chrono::steady_clock::now();
        const double elapsedMs = std::chrono::duration<double, std::milli>(end - begin).count();
        totalMs += elapsedMs;
        result.searchTimesMs.push_back(elapsedMs);
        if (repetition == 0) { first = current; }
        else if (current.found != first.found || current.path != first.path || current.cost != first.cost || current.expandedNodes != first.expandedNodes) {
            throw std::runtime_error("Fresh search repetitions produced inconsistent results");
        }
    }
    result.runCount = BENCHMARK_RUNS;
    result.timeMs = totalMs / static_cast<double>(BENCHMARK_RUNS);
    result.expandedNodes = first.expandedNodes;
    result.status = first.found ? "STATUS_OK" : "STATUS_NO_PATH";
    if (first.found) {
        result.pathLength = first.cost;
        for (auto node : first.path) { result.path.push_back(graph.label(node)); }
        result.totalSteps = result.path.empty() ? 0 : result.path.size() - 1;
    }
    else { result.message = "OPEN exhausted without reaching the goal"; }
    return result;
}

std::string jsonString(const std::string& value) {
    std::ostringstream output;
    output << '"';
    for (unsigned char byte : value) {
        switch (byte) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (byte < 0x20) { output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(byte) << std::dec << std::setfill(' '); }
            else { output << static_cast<char>(byte); }
        }
    }
    output << '"';
    return output.str();
}
std::string optionalNumber(const std::optional<double>& value) {
    if (!value) { return "null"; }
    std::ostringstream output;
    output << std::setprecision(17) << *value;
    return output.str();
}
std::string pathText(const BenchmarkResult& result) {
    if (result.path.empty()) { return "N/A"; }
    std::ostringstream output;
    for (std::size_t index = 0; index < result.path.size(); ++index) { output << (index == 0 ? "" : " -> ") << result.path.at(index); }
    return output.str();
}
std::string resultJson(const BenchmarkResult& result, const std::string& start, const std::string& goal) {
    std::ostringstream output;
    output << "{\n  \"algorithm\": " << jsonString(result.algorithm)
        << ",\n  \"start_node\": " << jsonString(start) << ",\n  \"goal_node\": " << jsonString(goal)
        << ",\n  \"status\": " << jsonString(result.status) << ",\n  \"message\": " << jsonString(result.message)
        << ",\n  \"heuristic\": " << jsonString(result.heuristic) << ",\n  \"heuristic_alpha\": " << result.alpha
        << ",\n  \"coordinate_units\": \"schematic\",\n  \"cost_units\": \"meters\""
        << ",\n  \"benchmark_runs\": " << result.runCount
        << ",\n  \"timing_aggregation\": \"arithmetic mean\""
        << ",\n  \"timing_scope\": " << jsonString(TIMING_SCOPE)
        << ",\n  \"computational_time_ms\": " << optionalNumber(result.timeMs)
        << ",\n  \"path_length_meters\": " << optionalNumber(result.pathLength)
        << ",\n  \"number_of_nodes_expanded\": " << result.expandedNodes
        << ",\n  \"total_steps\": " << result.totalSteps << ",\n  \"path_cell_count\": " << result.path.size() << ",\n  \"path\": [";
    for (std::size_t index = 0; index < result.path.size(); ++index) { output << (index == 0 ? "" : ", ") << jsonString(result.path.at(index)); }
    output << "],\n  \"search_times_ms\": [";
    for (std::size_t index = 0; index < result.searchTimesMs.size(); ++index) {
        output << (index == 0 ? "" : ", ") << std::setprecision(17) << result.searchTimesMs.at(index);
    }
    output << "]\n}\n";
    return output.str();
}

std::string renderVisualizationMap(const BenchmarkResult& result, const CampusNodes& nodes, const std::string& start, const std::string& goal) {
    constexpr int width = 10, height = 8;
    std::vector<std::string> grid(height, std::string(width, '.'));
    for (std::size_t index = 1; index < result.path.size(); ++index) {
        const auto& from = nodes.at(result.path.at(index - 1));
        const auto& to = nodes.at(result.path.at(index));
        const int steps = static_cast<int>(std::max(std::abs(to.x - from.x), std::abs(to.y - from.y)));
        for (int step = 0; step <= steps; ++step) {
            const double fraction = steps == 0 ? 0.0 : static_cast<double>(step) / steps;
            const int x = static_cast<int>(std::round(from.x + (to.x - from.x) * fraction));
            const int y = static_cast<int>(std::round(from.y + (to.y - from.y) * fraction));
            if (x >= 0 && x < width && y >= 0 && y < height && grid.at(y).at(x) == '.') { grid.at(y).at(x) = '*'; }
        }
    }
    for (const auto& pair : nodes) {
        const int x = static_cast<int>(pair.second.x), y = static_cast<int>(pair.second.y);
        if (x >= 0 && x < width && y >= 0 && y < height) { grid.at(y).at(x) = pair.first.at(0); }
    }
    std::ostringstream output;
    output << "Campus map: A-G retain node IDs; * marks schematic route segments.\nStart: " << start << "; goal: " << goal << ". Route distance comes from edge weights.\n";
    for (int y = height - 1; y >= 0; --y) {
        output << std::setw(2) << y << " | ";
        for (char symbol : grid.at(y)) { output << symbol << ' '; }
        output << '\n';
    }
    output << "     0 1 2 3 4 5 6 7 8 9\n";
    return output.str();
}
std::string resultLog(const BenchmarkResult& result, const CampusNodes& nodes, const std::string& start, const std::string& goal) {
    std::ostringstream output;
    output << "APP 2 CAMPUS BENCHMARK: " << result.algorithm << "\nStart: " << start << "; goal: " << goal
        << "\nHeuristic: " << result.heuristic << "; alpha: " << result.alpha << "; coordinates: schematic; edge distance: meters\nStatus: " << result.status << '\n';
    if (!result.message.empty()) { output << "Detail: " << result.message << '\n'; }
    output << "Path length: " << (result.pathLength ? optionalNumber(result.pathLength) + " meters" : "N/A")
        << "\nTotal steps (edges): " << result.totalSteps << "\nPath cell count: " << result.path.size()
        << "\nNodes expanded (neighbor processing; goal excluded): " << result.expandedNodes
        << "\nComputational time: " << (result.timeMs ? optionalNumber(result.timeMs) + " ms" : "N/A") << " (mean of " << result.runCount << " fresh core searches)"
        << "\nBenchmark runs: " << result.runCount << "\nTiming aggregation: arithmetic mean"
        << "\nTiming scope: " << TIMING_SCOPE << "\nPath: " << pathText(result) << "\n\n"
        << renderVisualizationMap(result, nodes, start, goal);
    return output.str();
}
std::string comparisonLog(const BenchmarkResult& aStar, const BenchmarkResult& dijkstra, const std::string& start, const std::string& goal) {
    std::ostringstream output;
    output << "APP 2 COMPARISON: A* vs Dijkstra\nStart: " << start << "; goal: " << goal
        << "\nBoth use the shared A* core; Dijkstra uses h=0. A* uses alpha=1 Euclidean.\nEdge distances: meters; coordinates: schematic.\n\n";
    for (const auto* result : { &aStar, &dijkstra }) {
        output << result->algorithm << ": " << result->status << "; path " << pathText(*result)
            << "; distance " << (result->pathLength ? optionalNumber(result->pathLength) + " m" : "N/A")
            << "; expansions " << result->expandedNodes << "; mean time " << (result->timeMs ? optionalNumber(result->timeMs) + " ms" : "N/A") << '\n';
    }
    if (aStar.pathLength && dijkstra.pathLength) {
        if (std::abs(*aStar.pathLength - *dijkstra.pathLength) > 1e-9) { throw std::runtime_error("A* and Dijkstra disagree on optimal distance"); }
        output << "Both return the same optimal distance.\n";
        if (aStar.expandedNodes == dijkstra.expandedNodes) { output << "This query demonstrates no reduction in node expansions.\n"; }
        else { output << "Observed expansion difference (Dijkstra minus A*): " << static_cast<long long>(dijkstra.expandedNodes) - static_cast<long long>(aStar.expandedNodes) << " nodes.\n"; }
    }
    if (aStar.timeMs && dijkstra.timeMs) { output << "Times are 10-run means on a seven-node graph; small timing differences do not establish a general speed advantage.\n"; }
    return output.str();
}
void writeToFile(const fs::path& path, const std::string& content) {
    std::ofstream file(path, std::ios::binary);
    if (!file) { throw std::runtime_error("Cannot open output file: " + path.string()); }
    file << content;
    file.close();
    if (!file) { throw std::runtime_error("Cannot finish output file: " + path.string()); }
}
void disconnectNode(CampusEdges& edges, const CampusNodes& nodes, const std::string& label) {
    if (nodes.find(label) == nodes.end()) { throw std::invalid_argument("Cannot disconnect unknown node: " + label); }
    edges[label].clear();
    for (auto& pair : edges) {
        auto& outgoing = pair.second;
        outgoing.erase(std::remove_if(outgoing.begin(), outgoing.end(), [&label](const CampusEdge& edge) { return edge.to == label; }), outgoing.end());
    }
}

#ifndef APP2_TESTING
int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    try {
        fs::path outputDirectory = "app2_output";
        std::string start = "A", goal = "F";
        std::vector<std::string> disconnected;
        for (int index = 1; index < argc; ++index) {
            const std::string option = argv[index];
            if (option == "--help") { std::cout << "Usage: app2 [--output DIR] [--start A-G] [--goal A-G] [--disconnect A-G]\n"; return 0; }
            if (index + 1 >= argc) { throw std::invalid_argument("Missing value for " + option); }
            const std::string value = argv[++index];
            if (option == "--output") { outputDirectory = fs::u8path(value); }
            else if (option == "--start") { start = value; }
            else if (option == "--goal") { goal = value; }
            else if (option == "--disconnect") { disconnected.push_back(value); }
            else { throw std::invalid_argument("Unknown option: " + option); }
        }
        const CampusNodes nodes = defaultNodes();
        CampusEdges edges = defaultEdges();
        for (const auto& label : disconnected) { disconnectNode(edges, nodes, label); }
        const CampusGraph graph(nodes, edges);
        const auto aStar = runBenchmark(graph, start, goal, true);
        const auto dijkstra = runBenchmark(graph, start, goal, false);
        const std::string aStarLog = resultLog(aStar, nodes, start, goal);
        const std::string dijkstraLog = resultLog(dijkstra, nodes, start, goal);
        const std::string comparison = comparisonLog(aStar, dijkstra, start, goal);
        const std::string combinedJson = "{\n  \"application\": \"Application 2: Campus Map Pathfinding Evaluation\",\n  \"start_node\": " + jsonString(start)
            + ",\n  \"goal_node\": " + jsonString(goal) + ",\n  \"algorithms\": {\n    \"a_star\": " + resultJson(aStar, start, goal)
            + ",\n    \"dijkstra\": " + resultJson(dijkstra, start, goal) + "  }\n}\n";
        fs::create_directories(outputDirectory);
        writeToFile(outputDirectory / "a_star_log_App2.txt", aStarLog);
        writeToFile(outputDirectory / "a_star_result_App2.json", resultJson(aStar, start, goal));
        writeToFile(outputDirectory / "dijkstra_log_App2.txt", dijkstraLog);
        writeToFile(outputDirectory / "dijkstra_result_App2.json", resultJson(dijkstra, start, goal));
        writeToFile(outputDirectory / "benchmark_log_App2.txt", comparison);
        writeToFile(outputDirectory / "benchmark_result_App2.json", combinedJson);
        std::cout << aStarLog << '\n' << dijkstraLog << '\n' << comparison << "Exported six files to " << fs::absolute(outputDirectory).string() << '\n';
        return aStar.status == "STATUS_INVALID" || dijkstra.status == "STATUS_INVALID" ? 2 : 0;
    }
    catch (const std::exception& error) { std::cerr << "App 2 error: " << error.what() << '\n'; return 2; }
}
#endif
