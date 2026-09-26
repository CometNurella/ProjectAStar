#include "Astar.h"
#include "../Heuristics/Heuristics.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <vector>

namespace astar {
    namespace {
        struct OpenEntry {
            NodeId id;
            double gScore;
            double hScore;
            double fScore;
            std::size_t insertionOrder;
        };

        struct CompareOpenEntry {
            bool operator()( const OpenEntry& lhs, const OpenEntry& rhs) const noexcept {
                if (lhs.fScore != rhs.fScore) {
                    return lhs.fScore > rhs.fScore;
                }

                if (lhs.hScore != rhs.hScore) {
                    return lhs.hScore > rhs.hScore;
                }

                return lhs.insertionOrder > rhs.insertionOrder;
            }
        };
    }

    SearchResult Astar::findPath(
        const Graph& graph,
        NodeId start,
        NodeId goal,
        const Heuristic& heuristic) const {
        const std::size_t nodeCount{
            graph.getNodeCount()
        };

        if (start >= nodeCount || goal >= nodeCount) {
            throw std::out_of_range{
                "Start or goal node ID is outside the graph"
            };
        }

        if (!heuristic) {
            throw std::invalid_argument{
                "Heuristic is empty"
            };
        }

        std::vector<Node> nodes;
        nodes.reserve(nodeCount);

        for (NodeId id{ 0 }; id < nodeCount; id++) {
            nodes.emplace_back(id);
        }

        std::vector<bool> closed(nodeCount, false);

        std::priority_queue<OpenEntry, std::vector<OpenEntry>, CompareOpenEntry> open;

        std::size_t insertionOrder{ 0 };

        SearchResult result{};

        const auto evaluateHeuristic =
            [&heuristic, goal](NodeId node) {
                const double h{
                    heuristic(node, goal)
                };

                if (!std::isfinite(h) || h < 0.0) {
                    throw std::invalid_argument{
                        "Heuristic must return a finite, non-negative value"
                    };
                }

                return h;
            };

        Node& startNode{ nodes[start] };

        startNode.gScore = 0.0;
        startNode.hScore = evaluateHeuristic(start);

        open.push(OpenEntry{
                startNode.id,
                startNode.gScore,
                startNode.hScore,
                startNode.fScore(),
                insertionOrder++
            }
        );

        while (!open.empty()) {
            const OpenEntry entry{ open.top() };
            open.pop();

            Node& current{ nodes[entry.id] };

            // Ignore outdated priority-queue entries.
            if (entry.gScore != current.gScore) {
                continue;
            }

            // Defensive check in case an already-expanded
            // active entry remains in the queue.
            if (closed[current.id]) {
                continue;
            }

            // Goal is tested when selected from OPEN,
            // not when first discovered.
            if (current.id == goal) {
                result.found = true;
                result.path =
                    reconstructPath(nodes, goal);
                result.cost = current.gScore;

                return result;
            }

            closed[current.id] = true;
            ++result.expandedNodes;

            for (const Edge& edge : graph.getEdges(current.id)) {
                if (edge.to >= nodeCount) {
                    throw std::out_of_range{
                        "Graph returned an invalid neighbor node ID"
                    };
                }

                if (!std::isfinite(edge.cost) || edge.cost < 0.0) {
                    throw std::invalid_argument{
                        "Graph edge costs must be finite and nonnegative"
                    };
                }

                Node& neighbor{ nodes[edge.to] };

                const double tentativeG{
                    current.gScore + edge.cost
                };

                if (tentativeG < neighbor.gScore) {
                    neighbor.parent = current.id;
                    neighbor.gScore = tentativeG;
                    neighbor.hScore =
                        evaluateHeuristic(neighbor.id);

                    // A better route to a CLOSED node was found.
                    // Reopen it.
                    if (closed[neighbor.id]) {
                        closed[neighbor.id] = false;
                    }

                    open.push(
                        OpenEntry{
                            neighbor.id,
                            neighbor.gScore,
                            neighbor.hScore,
                            neighbor.fScore(),
                            insertionOrder++
                        }
                    );
                }
            }
        }

        // OPEN became empty: no path exists.
        return result;
    }

    std::vector<NodeId> Astar::reconstructPath( const std::vector<Node>& nodes, NodeId goal) {
        std::vector<NodeId> path;

        NodeId current{ goal };

        while (true) {
            path.push_back(current);

            if (!nodes[current].parent.has_value()) {
                break;
            }

            current = nodes[current].parent.value();
        }

        std::reverse(path.begin(), path.end());

        return path;
    }
}