#pragma once
#include "../../AstarCore/Grid/GridNode.h"

#include<vector>
#include<string>

namespace application::robot {
	struct Rectangle {
		int x1, y1;
		int x2, y2;
	};

	struct RobotCase {
		std::string id;
		astar::GridNode Ct;
		astar::GridNode n;
	};

	struct RobotMap {
		std::string id;
		std::string description;
		std::vector<Rectangle> obstacles;
		std::vector<RobotCase> cases;
	};

	struct RobotInput {
		int width;
		int height;
		double cellSize;
		std::string origin;
		std::vector<RobotMap> maps;
	};

}