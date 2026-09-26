#pragma once

#include "Heuristics.h"
#include "../Grid/Grid.h"

#include <algorithm>
#include <cmath>

namespace astar {
    namespace grid_heuristics {
        inline Heuristic zero() {
            return [](NodeId, NodeId) { return 0.0; };
        }

        inline Heuristic manhattan(const Grid& grid) {
            return [&grid](NodeId from, NodeId goal) {
                    const GridNode a{ grid.toGridNode(from) };
                    const GridNode b{ grid.toGridNode(goal) };

                    const int dx{ std::abs(a.x - b.x) };
                    const int dy{ std::abs(a.y - b.y) };

                    //return static_cast<double>(dx + dy);
                    return static_cast<astar::Heuristic>(dx + dy);
                };
        }

        inline Heuristic euclidean(const Grid& grid) {
            return [&grid](NodeId from, NodeId goal) {
                    const GridNode a{ grid.toGridNode(from) };
                    const GridNode b{ grid.toGridNode(goal) };

                    const double dx{ static_cast<double>(a.x - b.x) };

                    const double dy{ static_cast<double>(a.y - b.y) };

                    return std::sqrt(dx * dx + dy * dy);
                };
        }

        inline Heuristic chebyshev(const Grid& grid) {
            return [&grid](NodeId from, NodeId goal) {
                    const GridNode a{ grid.toGridNode(from) };
                    const GridNode b{ grid.toGridNode(goal) };

                    const int dx{ std::abs(a.x - b.x) };
                    const int dy{ std::abs(a.y - b.y) };

                    return static_cast<double>(std::max(dx, dy)); 
                };
        }

        inline Heuristic octile(
            const Grid& grid,
            double straightCost = 1.0,
            double diagonalCost = std::sqrt(2.0)) {
            return [&grid, straightCost, diagonalCost](NodeId from, NodeId goal) {
                    const GridNode a{ grid.toGridNode(from) };
                    const GridNode b{ grid.toGridNode(goal) };

                    const int dx{ std::abs(a.x - b.x) };
                    const int dy{ std::abs(a.y - b.y) };

                    const int minDelta{ std::min(dx, dy) };
                    const int maxDelta{ std::max(dx, dy) };

                    return diagonalCost * minDelta + straightCost * (maxDelta - minDelta);
                };
        }
    }
}