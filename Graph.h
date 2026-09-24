#pragma once

#include "Node.h"

#include <cstddef>
#include <vector>

namespace astar {
    struct Edge {
        NodeId to;
        double cost;
    };

    class Graph {
    public:
        [[nodiscard]]
        virtual std::size_t getNodeCount() const noexcept = 0;

        [[nodiscard]]
        virtual std::vector<Edge>
            getEdges(NodeId node) const = 0;

        virtual ~Graph() = default;
    };
}