#include "RobotJsonLoader.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace application::io {
    namespace {
        astar::GridNode parseGridNode(const nlohmann::json& json) {
            return astar::GridNode{
                json.at(0).get<int>(),
                json.at(1).get<int>()
            };
        }

        robot::Rectangle parseRectangle(const nlohmann::json& json) {
            return robot::Rectangle{
                json.at(0).get<int>(),
                json.at(1).get<int>(),
                json.at(2).get<int>(),
                json.at(3).get<int>()
            };
        }

        robot::RobotCase parseRobotCase(const nlohmann::json& json) {
            robot::RobotCase robotCase{};
            robotCase.id = json.at("id").get<std::string>();
            robotCase.Ct = parseGridNode(json.at("Ct"));
            robotCase.n = parseGridNode(json.at("n"));

            return robotCase;
        }

        robot::RobotMap parseRobotMap(const std::string& mapId, const nlohmann::json& json) {
            robot::RobotMap map{};
            map.id = mapId;
            map.description = json.at("desc").get<std::string>();

            for (const auto& rectangleJson : json.at("obstacles_rect_x1y1x2y2")) {
                map.obstacles.push_back(parseRectangle(rectangleJson));
            }

            for (const auto& caseJson : json.at("cases")) {
                map.cases.push_back(parseRobotCase(caseJson));
            }

            return map;
        }
    }

    robot::RobotInput loadRobotInput(const std::filesystem::path& filePath) {
        std::ifstream file{ filePath };

        if (!file) {
            throw std::runtime_error{
                "Failed to open robot input file: "
                + filePath.string()
            };
        }

        nlohmann::json json;
        file >> json;

        robot::RobotInput input{};
        input.width = json.at("width").get<int>();
        input.height = json.at("height").get<int>();
        input.cellSize = json.at("cell_size").get<double>();
        input.origin = json.at("origin").get<std::string>();

        for (const auto& [mapId, mapJson] : json.at("maps").items()) {
            input.maps.push_back(parseRobotMap(mapId, mapJson));
        }

        return input;
    }

}