#include "RobotJsonLoader.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace application::io {
    namespace {
        using Json = nlohmann::json;

        [[noreturn]] void invalid(const std::string& location, const std::string& message) {
            throw std::invalid_argument{ "Robot input " + location + ": " + message };
        }

        const Json& field(const Json& object, const char* name, const std::string& location) {
            if (!object.is_object()) {
                invalid(location, "must be an object");
            }
            const auto found = object.find(name);
            if (found == object.end()) {
                invalid(location + "." + name, "required field is missing");
            }
            return *found;
        }

        void arrayOfSize(const Json& value, std::size_t length, const std::string& location) {
            if (!value.is_array() || value.size() != length) {
                invalid(location, "must be an array of exactly " + std::to_string(length) + " numbers");
            }
        }

        double finiteNumber(const Json& value, const std::string& location) {
            // is_number() excludes booleans; get<int>() would otherwise accept
            // fractional JSON numbers by silently discarding their decimals.
            if (!value.is_number()) {
                invalid(location, "must be a finite number");
            }
            const double number = value.get<double>();
            if (!std::isfinite(number)) {
                invalid(location, "must be a finite number");
            }
            return number;
        }

        int integer(const Json& value, const std::string& location) {
            const double number = finiteNumber(value, location);
            if (std::trunc(number) != number ||
                number < static_cast<double>(std::numeric_limits<int>::min()) ||
                number > static_cast<double>(std::numeric_limits<int>::max())) {
                invalid(location, "must be an integer within the supported int range");
            }
            return static_cast<int>(number);
        }

        std::string string(const Json& value, const std::string& location) {
            if (!value.is_string()) {
                invalid(location, "must be a string");
            }
            return value.get<std::string>();
        }

        std::string safeId(const std::string& id, const std::string& location) {
            if (id.empty() || id.size() > 128) {
                invalid(location, "must contain 1 to 128 letters, digits, underscores or hyphens");
            }
            std::string upper;
            upper.reserve(id.size());
            for (const unsigned char character : id) {
                const bool letter = (character >= 'A' && character <= 'Z') ||
                    (character >= 'a' && character <= 'z');
                const bool digit = character >= '0' && character <= '9';
                if (!letter && !digit && character != '_' && character != '-') {
                    invalid(location, "must contain only letters, digits, underscores or hyphens");
                }
                upper.push_back(static_cast<char>(character >= 'a' && character <= 'z'
                    ? character - 'a' + 'A' : character));
            }
            const bool deviceId = upper == "CON" || upper == "PRN" || upper == "AUX" || upper == "NUL" ||
                (upper.size() == 4 && (upper.substr(0, 3) == "COM" || upper.substr(0, 3) == "LPT") &&
                    upper[3] >= '1' && upper[3] <= '9');
            if (deviceId) {
                invalid(location, "must not be a reserved Windows device name");
            }
            return upper;
        }

        robot::WorldPoint worldPoint(const Json& value, const std::string& location) {
            arrayOfSize(value, 2, location);
            // Keep finite out-of-grid coordinates so the runner can report
            // STATUS_INVALID for that case while still running the other cases.
            return robot::WorldPoint{
                finiteNumber(value[0], location + "[0]"),
                finiteNumber(value[1], location + "[1]")
            };
        }

        robot::Rectangle rectangle(const Json& value, const std::string& location, int width, int height) {
            arrayOfSize(value, 4, location);
            robot::Rectangle result{
                integer(value[0], location + "[0]"), integer(value[1], location + "[1]"),
                integer(value[2], location + "[2]"), integer(value[3], location + "[3]")
            };
            if (result.x1 < 0 || result.y1 < 0 || result.x1 > result.x2 || result.y1 > result.y2 ||
                result.x2 >= width || result.y2 >= height) {
                invalid(location, "bounds must be ordered, inclusive cell indices contained in the grid");
            }
            return result;
        }

        robot::RobotCase robotCase(const Json& value, const std::string& location, std::set<std::string>& caseIds) {
            robot::RobotCase result{};
            result.id = string(field(value, "id", location), location + ".id");
            if (!caseIds.insert(safeId(result.id, location + ".id")).second) {
                invalid(location + ".id", "duplicate case id '" + result.id +
                    "' (ids must be unique across maps, ignoring case for Windows filenames)");
            }
            result.Ct = worldPoint(field(value, "Ct", location), location + ".Ct");
            result.n = worldPoint(field(value, "n", location), location + ".n");
            return result;
        }

        robot::RobotMap robotMap(const std::string& id, const Json& value, int width, int height,
            std::set<std::string>& caseIds) {
            const std::string location = "$.maps." + id;
            safeId(id, "$.maps key");
            robot::RobotMap result{};
            result.id = id;
            result.description = string(field(value, "desc", location), location + ".desc");

            const Json& rectangles = field(value, "obstacles_rect_x1y1x2y2", location);
            if (!rectangles.is_array()) {
                invalid(location + ".obstacles_rect_x1y1x2y2", "must be an array");
            }
            for (std::size_t index = 0; index < rectangles.size(); ++index) {
                result.obstacles.push_back(rectangle(rectangles[index],
                    location + ".obstacles_rect_x1y1x2y2[" + std::to_string(index) + "]", width, height));
            }

            const Json& cases = field(value, "cases", location);
            if (!cases.is_array() || cases.empty()) {
                invalid(location + ".cases", "must be a nonempty array");
            }
            for (std::size_t index = 0; index < cases.size(); ++index) {
                result.cases.push_back(robotCase(cases[index],
                    location + ".cases[" + std::to_string(index) + "]", caseIds));
            }
            return result;
        }
    }

    robot::RobotInput loadRobotInput(const std::filesystem::path& filePath) {
        std::ifstream file{ filePath };
        if (!file) {
            throw std::runtime_error{ "Failed to open robot input file: " + filePath.string() };
        }

        Json document;
        try {
            document = Json::parse(file);
        }
        catch (const Json::exception& error) {
            throw std::invalid_argument{ "Invalid robot JSON in " + filePath.string() + ": " + error.what() };
        }
        if (file.bad()) {
            throw std::runtime_error{ "Failed to read robot input file: " + filePath.string() };
        }

        robot::RobotInput input{};
        input.width = integer(field(document, "width", "$"), "$.width");
        input.height = integer(field(document, "height", "$"), "$.height");
        if (input.width <= 0 || input.height <= 0) {
            invalid("$.width/height", "grid dimensions must be positive");
        }
        const auto width = static_cast<std::size_t>(input.width);
        const auto height = static_cast<std::size_t>(input.height);
        if (width > std::numeric_limits<std::size_t>::max() / height) {
            invalid("$.width/height", "grid cell count exceeds the supported range");
        }
        input.cellSize = finiteNumber(field(document, "cell_size", "$"), "$.cell_size");
        if (input.cellSize <= 0.0 ||
            !std::isfinite(static_cast<double>(input.width) * input.cellSize) ||
            !std::isfinite(static_cast<double>(input.height) * input.cellSize)) {
            invalid("$.cell_size", "must be positive and produce finite grid world bounds");
        }
        input.origin = string(field(document, "origin", "$"), "$.origin");
        if (input.origin != "bottom-left") {
            invalid("$.origin", "only 'bottom-left' is supported");
        }

        const Json& maps = field(document, "maps", "$");
        if (!maps.is_object() || maps.empty()) {
            invalid("$.maps", "must be a nonempty object keyed by map id");
        }
        std::set<std::string> caseIds;
        for (const auto& entry : maps.items()) {
            input.maps.push_back(robotMap(entry.key(), entry.value(), input.width, input.height, caseIds));
        }
        return input;
    }
}
