#pragma once

#include "Graph.h"
#include "Node.h"
#include "../Heuristics/Heuristics.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace astar {
    struct SearchResult {
        bool found{ false };

        std::vector<NodeId> path{};

        double cost{ std::numeric_limits<double>::infinity() };

        std::size_t expandedNodes{ 0 };
    };

    class Astar {
    private:
        [[nodiscard]]
        static std::vector<NodeId> reconstructPath(const std::vector<Node>& nodes, NodeId goal);

    public:
        [[nodiscard]]
        SearchResult findPath(const Graph& graph, NodeId start, NodeId goal, const Heuristic& heuristic) const;
    };
}