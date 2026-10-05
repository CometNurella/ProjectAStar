#define APP2_TESTING
#include "../Benchmarks/campus/main.cpp"
#include <limits>
#include <numeric>

namespace {
    void check(bool value, const std::string& message) {
        if (!value) { throw std::runtime_error(message); }
    }
    template<class Action>
    void checkInvalid(Action action, const std::string& message) {
        try { action(); }
        catch (const std::invalid_argument&) { return; }
        throw std::runtime_error(message);
    }
    void verifyCampusPairs() {
        const auto nodes = defaultNodes();
        const CampusGraph graph(nodes, defaultEdges());
        for (const auto& source : nodes) {
            for (const auto& target : nodes) {
                const auto aStar = runBenchmark(graph, source.first, target.first, true);
                const auto dijkstra = runBenchmark(graph, source.first, target.first, false);
                check(aStar.status == "STATUS_OK" && dijkstra.status == "STATUS_OK", "All campus pairs must be reachable");
                check(aStar.pathLength && dijkstra.pathLength && *aStar.pathLength == *dijkstra.pathLength, "A* must match Dijkstra on all 49 pairs");
                check(aStar.totalSteps == aStar.path.size() - 1, "Steps must count edges");
                check(aStar.runCount == 10 && aStar.timeMs && aStar.searchTimesMs.size() == 10, "Successful timing must contain ten core runs");
                const double measuredMean = std::accumulate(aStar.searchTimesMs.begin(), aStar.searchTimesMs.end(), 0.0) / 10.0;
                check(*aStar.timeMs == measuredMean, "Exported timing must be the arithmetic mean of ten actual core calls");
                double pathCost = 0.0;
                for (std::size_t index = 1; index < aStar.path.size(); ++index) {
                    const auto edges = graph.getEdges(graph.nodeId(aStar.path.at(index - 1)));
                    const auto targetId = graph.nodeId(aStar.path.at(index));
                    const auto edge = std::find_if(edges.begin(), edges.end(), [targetId](const astar::Edge& value) { return value.to == targetId; });
                    check(edge != edges.end(), "Exported path must use actual campus edges");
                    pathCost += edge->cost;
                }
                check(pathCost == *aStar.pathLength, "Exported length must equal the sum of path edge weights");
            }
        }
        const auto baseline = runBenchmark(graph, "A", "F", true);
        const auto baselineDijkstra = runBenchmark(graph, "A", "F", false);
        check(baseline.path == std::vector<std::string>({"A", "B", "D", "F"}), "Baseline route must remain A-B-D-F");
        check(baseline.pathLength && *baseline.pathLength == 820, "A-F distance must be 820 meters");
        check(baseline.totalSteps == 3 && baseline.expandedNodes == 5 && baselineDijkstra.expandedNodes == 5, "A-F must have three moves and five non-goal expansions");
        const auto trivial = runBenchmark(graph, "A", "A", true);
        check(trivial.pathLength && *trivial.pathLength == 0 && trivial.totalSteps == 0 && trivial.expandedNodes == 0, "Trivial route must have zero cost, moves, and expansions");
        check(comparisonLog(baseline, baselineDijkstra, "A", "F").find("no reduction in node expansions") != std::string::npos, "Comparison must accurately describe equal expansion counts");
    }
    void verifyFailuresAndValidation() {
        const auto nodes = defaultNodes();
        const CampusGraph graph(nodes, defaultEdges());
        const auto invalid = runBenchmark(graph, "Z", "F", true);
        check(invalid.status == "STATUS_INVALID" && !invalid.pathLength && !invalid.timeMs && invalid.runCount == 0, "Invalid query must have absent cost/time and no searches");
        check(resultJson(invalid, "Z", "F").find("\"path_length_meters\": null") != std::string::npos, "Invalid JSON must use null path cost");
        check(resultLog(invalid, nodes, "Z", "F").find("Path length: N/A") != std::string::npos, "Invalid log must use N/A");
        auto disconnected = defaultEdges();
        disconnectNode(disconnected, nodes, "F");
        const CampusGraph noPathGraph(nodes, disconnected);
        for (bool useHeuristic : {true, false}) {
            const auto noPath = runBenchmark(noPathGraph, "A", "F", useHeuristic);
            check(noPath.status == "STATUS_NO_PATH" && !noPath.pathLength && noPath.path.empty(), "Disconnected goal must have absent path cost");
            check(noPath.expandedNodes == 6 && noPath.runCount == 10, "No-path search must exhaust six reachable nodes");
            check(resultJson(noPath, "A", "F").find("\"path_length_meters\": null") != std::string::npos, "No-path JSON must use null cost");
        }
        for (double badWeight : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            auto edges = defaultEdges();
            edges.at("A").at(0).weight = badWeight;
            checkInvalid([&] { CampusGraph rejected(nodes, edges); }, "Invalid weights must be rejected before core calls");
        }
        auto badDestination = defaultEdges();
        badDestination.at("A").at(0).to = "Z";
        checkInvalid([&] { CampusGraph rejected(nodes, badDestination); }, "Unknown edge destination must be rejected");
        auto badSource = defaultEdges();
        badSource.emplace("Z", std::vector<CampusEdge>{});
        checkInvalid([&] { CampusGraph rejected(nodes, badSource); }, "Unknown edge source must be rejected");
        auto badCoordinates = nodes;
        badCoordinates.at("A").x = std::numeric_limits<double>::quiet_NaN();
        checkInvalid([&] { CampusGraph rejected(badCoordinates, defaultEdges()); }, "Nonfinite coordinates must be rejected");
        auto shortEdge = defaultEdges();
        shortEdge.at("A").at(0).weight = 1.0;
        const CampusGraph insufficientBound(nodes, shortEdge);
        check(runBenchmark(insufficientBound, "A", "F", true).status == "STATUS_INVALID", "Unsafe coordinate-to-meter bound must be rejected");
        check(runBenchmark(insufficientBound, "A", "F", false).status == "STATUS_OK", "Dijkstra must accept finite nonnegative edge costs");
        check(jsonString("Z\"\\\n") == "\"Z\\\"\\\\\\n\"", "JSON labels must escape quotes, slashes, and controls");
    }
}
int main() {
    try {
        verifyCampusPairs();
        verifyFailuresAndValidation();
        std::cout << "PASS: App2 shared core, all 49 campus pairs, baseline counts, fresh timing, failures, and graph validation\n";
    } catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
