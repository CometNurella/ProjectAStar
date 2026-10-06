#pragma once
#include<vector>
#include<string>

namespace application::robot {
	struct Rectangle {
		int x1, y1;
		int x2, y2;
	};

	// Ct and n are world coordinates, not cell indices. Convert with floor
	// only after validating the point against this input's cell size and grid.
	struct WorldPoint {
		double x, y;
	};

	struct RobotCase {
		std::string id;
		WorldPoint Ct;
		WorldPoint n;
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
