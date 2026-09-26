#pragma once
#include<string>
#include<vector>

namespace application::campus {
	struct Coordinate {
		double x;
		double y;
	};

	struct CampusLocation {
		std::string id;
		std::string name;
		Coordinate position;
	};

	struct CampusRoad {
		std::string from;
		std::string to;
		double distanceMeters;
	};

	struct CampusCase {
		std::string id;
		std::string start;
		std::string goal;
	};

	struct CampusInput {
		std::vector<CampusLocation> locations;
		std::vector<CampusRoad> roads;
		std::vector<CampusCase> cases;
	};
}