#pragma once
#include<filesystem>
#include "../Robot/RobotInput.h"

namespace application::io {
	[[nodiscard]]
	robot::RobotInput loadRobotInput(const std::filesystem::path& filePath);
}