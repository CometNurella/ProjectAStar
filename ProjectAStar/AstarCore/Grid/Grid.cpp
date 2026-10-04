#include "Grid.h"
#include "../Core/Node.h"
#include "../Core/Graph.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace astar {
    namespace {
        std::size_t checkedCellCount(std::size_t width, std::size_t height, int connectivity) {
            if (connectivity != 4 && connectivity != 8) {
                throw std::invalid_argument{ "Grid connectivity must be 4 or 8" };
            }
            if (width == 0 || height == 0) {
                throw std::invalid_argument{ "Grid dimensions must be positive" };
            }
            // Coordinates and their neighboring positions use int. Reserve
            // room for the +1 candidate just beyond the last valid coordinate.
            constexpr auto coordinateLimit = static_cast<std::size_t>(std::numeric_limits<int>::max());
            if (width > coordinateLimit || height > coordinateLimit ||
                width > std::numeric_limits<std::size_t>::max() / height) {
                throw std::length_error{ "Grid dimensions exceed the supported coordinate range" };
            }
            return width * height;
        }
    }

    Grid::Grid(std::size_t width, std::size_t height, int connectivity)
        : m_width{ width }, m_height{ height }, m_connectivity{ connectivity },
          m_walkable(checkedCellCount(width, height, connectivity), true)
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
        if (!isWalkable(current.x, current.y)) {
            return edges;
        }
        edges.reserve(static_cast<std::size_t>(m_connectivity));

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

        if (m_connectivity == 8) {
            const GridNode diagonalOffsets[]{
                GridNode{1, -1}, GridNode{1, 1},
                GridNode{-1, 1}, GridNode{-1, -1}
            };
            for (const GridNode& offset : diagonalOffsets) {
                const int x = current.x + offset.x;
                const int y = current.y + offset.y;
                if (!isWalkable(x, y) ||
                    !isWalkable(x, current.y) || !isWalkable(current.x, y)) {
                    continue;
                }
                edges.push_back(Edge{ toNodeId(x, y), std::sqrt(2.0) });
            }
        }
        return edges;
    }

    std::size_t Grid::getWidth() const noexcept { return m_width; }

    std::size_t Grid::getHeight() const noexcept { return m_height; }
}