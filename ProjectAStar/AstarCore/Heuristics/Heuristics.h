#pragma once

#include "../Core/Node.h"

#include <functional>

namespace astar {
    using Heuristic = std::function<double(NodeId, NodeId)>;
}