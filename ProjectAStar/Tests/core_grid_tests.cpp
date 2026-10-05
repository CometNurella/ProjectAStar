#include "../AstarCore/Core/AStar.h"
#include "../AstarCore/Grid/Grid.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
    void require(bool condition, const std::string& message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    template<class Exception, class Operation>
    void requireThrows(Operation operation, const std::string& message) {
        try {
            operation();
        } catch (const Exception&) {
            return;
        }
        throw std::runtime_error(message);
    }

    bool close(double lhs, double rhs) {
        return std::abs(lhs - rhs) < 1e-12;
    }

    bool hasEdge(const astar::Grid& grid, int fromX, int fromY, int toX, int toY, double cost) {
        const astar::NodeId destination = grid.toNodeId(toX, toY);
        for (const auto& edge : grid.getEdges(grid.toNodeId(fromX, fromY))) {
            if (edge.to == destination) {
                return close(edge.cost, cost);
            }
        }
        return false;
    }

    void testConnectivityAndCosts() {
        astar::Grid four(3, 3);
        require(four.getEdges(four.toNodeId(1, 1)).size() == 4, "Default grid must have four neighbors");
        require(four.getEdges(four.toNodeId(0, 0)).size() == 2, "Four-direction corner must have two neighbors");
        require(hasEdge(four, 1, 1, 1, 0, 1.0), "Orthogonal move must cost one");
        require(!hasEdge(four, 1, 1, 2, 2, std::sqrt(2.0)), "Default grid must not allow diagonal movement");

        astar::Grid eight(3, 3, 8);
        require(eight.getEdges(eight.toNodeId(1, 1)).size() == 8, "Eight-direction center must have eight neighbors");
        require(eight.getEdges(eight.toNodeId(0, 0)).size() == 3, "Eight-direction corner must have three neighbors");
        require(hasEdge(eight, 1, 1, 2, 2, std::sqrt(2.0)), "Diagonal move must cost sqrt(2)");
        require(hasEdge(eight, 1, 1, 1, 2, 1.0), "Eight-direction orthogonal move must cost one");

        const astar::Heuristic zero = [](astar::NodeId, astar::NodeId) { return 0.0; };
        astar::Astar solver;
        const auto orthogonalPath = solver.findPath(four, four.toNodeId(0, 0), four.toNodeId(2, 2), zero);
        const auto diagonalPath = solver.findPath(eight, eight.toNodeId(0, 0), eight.toNodeId(2, 2), zero);
        require(orthogonalPath.found && close(orthogonalPath.cost, 4.0), "Four-direction shortest path must cost four");
        require(diagonalPath.found && close(diagonalPath.cost, 2.0 * std::sqrt(2.0)), "Eight-direction shortest path must use two diagonals");
    }

    void testBlockedCornersAndSources() {
        astar::Grid grid(2, 2, 8);
        grid.setWalkable(1, 0, false);
        require(!hasEdge(grid, 0, 0, 1, 1, std::sqrt(2.0)), "Blocked horizontal side must forbid diagonal");
        require(grid.getEdges(grid.toNodeId(1, 0)).empty(), "Blocked source must have no outgoing edges");
        grid.setWalkable(1, 0, true);
        grid.setWalkable(0, 1, false);
        require(!hasEdge(grid, 0, 0, 1, 1, std::sqrt(2.0)), "Blocked vertical side must forbid diagonal");
        grid.setWalkable(0, 1, true);
        grid.setWalkable(1, 1, false);
        require(!hasEdge(grid, 0, 0, 1, 1, std::sqrt(2.0)), "Blocked diagonal destination must forbid diagonal");
        grid.setWalkable(1, 1, true);
        require(hasEdge(grid, 0, 0, 1, 1, std::sqrt(2.0)), "Clear sides and destination must permit diagonal");

        astar::Grid four(3, 3);
        four.setWalkable(1, 1, false);
        require(four.getEdges(four.toNodeId(1, 1)).empty(), "Four-direction blocked source must have no outgoing edges");

        // Check the same side-cell rule in every diagonal orientation.
        for (int dx : {-1, 1}) {
            for (int dy : {-1, 1}) {
                astar::Grid orientation(3, 3, 8);
                orientation.setWalkable(1 + dx, 1, false);
                require(!hasEdge(orientation, 1, 1, 1 + dx, 1 + dy, std::sqrt(2.0)), "All diagonal orientations must check horizontal side");
                orientation.setWalkable(1 + dx, 1, true);
                orientation.setWalkable(1, 1 + dy, false);
                require(!hasEdge(orientation, 1, 1, 1 + dx, 1 + dy, std::sqrt(2.0)), "All diagonal orientations must check vertical side");
            }
        }
    }

    void testBoundsAndConfiguration() {
        astar::Grid grid(3, 2, 8);
        require(!grid.isInside(-1, 0) && !grid.isInside(3, 0) && !grid.isInside(0, 2), "Bounds must exclude negative and upper-bound coordinates");
        require(!grid.isWalkable(-1, 0) && !grid.isWalkable(3, 0), "Outside coordinates must be non-walkable");
        require(grid.toNodeId(2, 1) == 5, "Coordinates must use y * width + x");
        const auto cell = grid.toGridNode(5);
        require(cell.x == 2 && cell.y == 1, "Node conversion must preserve coordinates");
        requireThrows<std::out_of_range>([&] { (void)grid.toNodeId(-1, 0); }, "Negative coordinates must throw");
        requireThrows<std::out_of_range>([&] { grid.setWalkable(3, 0, false); }, "Outside writes must throw");
        requireThrows<std::out_of_range>([&] { (void)grid.getEdges(6); }, "Invalid node IDs must throw");
        requireThrows<std::invalid_argument>([] { astar::Grid invalid(2, 2, 6); }, "Unsupported connectivity must throw");
        requireThrows<std::invalid_argument>([] { astar::Grid invalid(0, 2); }, "Zero width must throw");
        requireThrows<std::invalid_argument>([] { astar::Grid invalid(2, 0); }, "Zero height must throw");
        requireThrows<std::length_error>([] { astar::Grid invalid(std::numeric_limits<std::size_t>::max(), 2); }, "Oversized dimensions must throw before allocation");
    }

    class SmallGraph final : public astar::Graph {
    public:
        std::vector<std::vector<astar::Edge>> edges;
        explicit SmallGraph(std::vector<std::vector<astar::Edge>> value) : edges(std::move(value)) {}
        std::size_t getNodeCount() const noexcept override { return edges.size(); }
        std::vector<astar::Edge> getEdges(astar::NodeId node) const override { return edges.at(node); }
    };

    void testCoreCountingAndReopening() {
        // A is first expanded at g=3, then reopened at g=2 through B.
        // h(B)=3 is admissible but inconsistent with the B->A edge.
        SmallGraph graph({{{1, 3.0}, {2, 1.0}}, {{3, 2.0}}, {{1, 1.0}}, {}});
        const std::vector<double> values{4.0, 0.0, 3.0, 0.0};
        const astar::Heuristic inconsistent = [&values](astar::NodeId node, astar::NodeId) { return values.at(node); };
        astar::Astar solver;
        const auto result = solver.findPath(graph, 0, 3, inconsistent);
        require(result.found && close(result.cost, 4.0), "Reopening must recover optimal cost four");
        require(result.path == std::vector<astar::NodeId>({0, 2, 1, 3}), "Reopened parent chain must give the improved path");
        require(result.expandedNodes == 4, "Expanded count must include reopening and exclude selected goal");

        const astar::Heuristic zero = [](astar::NodeId, astar::NodeId) { return 0.0; };
        SmallGraph stale({{{1, 5.0}, {2, 1.0}}, {{3, 10.0}}, {{1, 1.0}}, {}});
        const auto staleResult = solver.findPath(stale, 0, 3, zero);
        require(staleResult.found && close(staleResult.cost, 12.0), "Improved queued route must yield cost twelve");
        require(staleResult.expandedNodes == 3, "Stale queue entry must not count as an expansion");

        const auto trivial = solver.findPath(graph, 0, 0, zero);
        require(trivial.found && trivial.path.size() == 1 && close(trivial.cost, 0.0) && trivial.expandedNodes == 0,
                "Start-equals-goal must have zero cost, zero moves, and zero expansions");

        SmallGraph unreachable({{{1, 1.0}}, {}, {}});
        const auto failure = solver.findPath(unreachable, 0, 2, zero);
        require(!failure.found && failure.path.empty() && std::isinf(failure.cost) && failure.expandedNodes == 2,
                "Unreachable goal must exhaust reachable nodes and preserve absent core cost");
    }
}

int main() {
    try {
        testConnectivityAndCosts();
        testBlockedCornersAndSources();
        testBoundsAndConfiguration();
        testCoreCountingAndReopening();
        std::cout << "PASS: grid connectivity, costs, blocked corners/sources, bounds, and core reopening/counting\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
