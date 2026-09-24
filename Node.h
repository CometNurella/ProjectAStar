#pragma once

#include <cstddef>
#include <limits>
#include <optional>

namespace astar {
    using NodeId = std::size_t;

    struct Node {
        NodeId id;

        double gScore;
        double hScore;

        std::optional<NodeId> parent;

        explicit Node(NodeId id)
            : id(id),
            gScore(std::numeric_limits<double>::infinity()),
            hScore(0.0),
            parent(std::nullopt)
        {}

        [[nodiscard]]
        double fScore() const noexcept {return gScore + hScore;}
    };
}