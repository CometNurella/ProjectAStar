#pragma once

#include "Node.h"

#include <functional>

namespace astar {
    using Heuristic = std::function<double(NodeId, NodeId)>;
}