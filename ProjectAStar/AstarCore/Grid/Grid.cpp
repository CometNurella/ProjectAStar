#include "Grid.h"

#include <stdexcept>

namespace astar {
    Grid::Grid(std::size_t width, std::size_t height)
        : m_width{ width }, m_height{ height }, m_walkable(width* height, true)
    {}

    std::size_t Grid::getNodeCount() const noexcept { return m_width * m_height; }

    bool Grid::isInside(int x, int y) const noexcept {
        return x >= 0 && y >= 0 &&
            static_cast<std::size_t>(x) < m_width &&
            static_cast<std::size_t>(y) < m_height;
    }

    bool Grid::isWalkable(int x, int y) const noexcept {
        if (!isInside(x, y)) {
            return false;
        }
        const NodeId id {
            static_cast<NodeId>(y) * m_width + static_cast<NodeId>(x)
        };
        return m_walkable[id];
    }

    void Grid::setWalkable(int x, int y, bool walkable) {
        if (!isInside(x, y)) {
            throw std::out_of_range{ "Grid coordinate is outside the grid" };
        }
        const NodeId id{ toNodeId(x, y) };
        m_walkable[id] = walkable;
    }

    NodeId Grid::toNodeId(int x, int y) const {
        if (!isInside(x, y)) {
            throw std::out_of_range{ "Grid coordinate is outside the grid" };
        }
        return static_cast<NodeId>(y) * m_width + static_cast<NodeId>(x);
    }

    GridNode Grid::toGridNode(NodeId node) const {
        if (node >= getNodeCount()) {
            throw std::out_of_range{ "Invalid node ID" };
        }
        return GridNode{ static_cast<int>(node % m_width), static_cast<int>(node / m_width) };
    }

    std::vector<Edge> Grid::getEdges(NodeId node) const {
        const GridNode current{ toGridNode(node) };

        std::vector<Edge> edges;

        const GridNode candidates[]{
            GridNode{current.x,     current.y - 1},
            GridNode{current.x + 1, current.y},
            GridNode{current.x,     current.y + 1},
            GridNode{current.x - 1, current.y}
        };

        for (const GridNode& candidate : candidates) {
            if (!isWalkable(candidate.x, candidate.y)) {
                continue;
            }
            edges.push_back(Edge{ toNodeId(candidate.x, candidate.y), 1.0 } );
        }
        return edges;
    }

    std::size_t Grid::getWidth() const noexcept { return m_width; }

    std::size_t Grid::getHeight() const noexcept { return m_height; }
}