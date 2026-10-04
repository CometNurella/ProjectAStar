#pragma once

#include "../Core/Graph.h"
#include "GridNode.h"

#include <cstddef>
#include <vector>

namespace astar {
    class Grid : public Graph {
    private:
        std::size_t m_width;
        std::size_t m_height;
        int m_connectivity;

        std::vector<bool> m_walkable;

    public:
        // Four-direction movement remains the default for existing callers.
        // Eight-direction movement forbids crossing blocked diagonal corners.
        Grid(std::size_t width, std::size_t height, int connectivity = 4);

        [[nodiscard]]
        std::size_t getNodeCount() const noexcept override;

        [[nodiscard]]
        std::vector<Edge> getEdges(NodeId node) const override;

        [[nodiscard]]
        bool isInside(int x, int y) const noexcept;

        [[nodiscard]]
        bool isWalkable(int x, int y) const noexcept;

        void setWalkable(int x, int y, bool walkable);

        [[nodiscard]]
        NodeId toNodeId(int x, int y) const;

        [[nodiscard]]
        GridNode toGridNode(NodeId node) const;

        [[nodiscard]]
        std::size_t getWidth() const noexcept;

        [[nodiscard]]
        std::size_t getHeight() const noexcept;
    };
}