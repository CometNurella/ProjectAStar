#include "RobotJsonLoader.h"
#include <nlohmann/json.hpp>

#include<fstream>
#include<string>
#include<stdexcept>

namespace application::io {
	robot::RobotInput loadRobotInput(const std::filesystem::path& filePath) {
		std::fstream file{ filePath };
		if (!file.is_open()) {
			throw std::runtime_error("Failed to open input file: " + filePath.string());
		}
		nlohmann::json json;
		file >> json;
		robot::RobotInput input{};
		// Parsing
		return input;
	}
}

