#pragma once

#include "Node.h"

#include <cstddef>
#include <vector>

namespace astar
{
    class Graph
    {
    public:
        virtual ~Graph() = default;

        [[nodiscard]]
        virtual std::size_t getNodeCount() const noexcept = 0;

        [[nodiscard]]
        virtual std::vector<NodeId>
            getNeighbors(NodeId node) const = 0;

        [[nodiscard]]
        virtual double
            getCost(NodeId from, NodeId to) const = 0;
    };
}