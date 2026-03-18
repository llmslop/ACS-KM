#include "acs_km/data_reader.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace acs_km {

namespace {

std::string trim_copy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::array<int, 2> parse_general_record(const std::string& line) {
    std::istringstream stream(line);
    std::array<int, 2> values{};
    if (!(stream >> values[0] >> values[1])) {
        throw std::runtime_error("Invalid vehicle/capacity record: '" + line + "'");
    }
    return values;
}

Request parse_request_record(const std::string& line) {
    std::istringstream stream(line);
    std::array<int, 8> values{};
    for (auto& value : values) {
        if (!(stream >> value)) {
            throw std::runtime_error("Invalid request record: '" + line + "'");
        }
    }

    return Request{
        .id = values[0],
        .x_coord = values[1],
        .y_coord = values[2],
        .demand = values[3],
        .start_window = static_cast<double>(values[4]),
        .end_window = static_cast<double>(values[5]),
        .service_time = static_cast<double>(values[6]),
        .available_time = static_cast<double>(values[7]),
    };
}

}  // namespace

DataReader::DataReader(std::filesystem::path file_path)
    : file_path_(std::move(file_path)) {}

InstanceData DataReader::read() const {
    if (file_path_.extension() != ".txt") {
        throw std::runtime_error("Only .txt input files are supported: '" + file_path_.string() + "'");
    }

    std::ifstream in(file_path_);
    if (!in.is_open()) {
        throw std::runtime_error("Unable to open input file: '" + file_path_.string() + "'");
    }

    InstanceData data;
    data.problem.name = file_path_.filename().string();

    bool read_general_on_next_line = false;
    bool in_requests_section = false;

    std::string line;
    while (std::getline(in, line)) {
        const auto trimmed = trim_copy(line);

        if (read_general_on_next_line && !trimmed.empty()) {
            const auto values = parse_general_record(trimmed);
            data.problem.vehicle_count = values[0];
            data.problem.capacity = values[1];
            read_general_on_next_line = false;
            continue;
        }

        if (in_requests_section && !trimmed.empty()) {
            const auto request = parse_request_record(trimmed);
            data.requests.push_back(request);

            if (request.available_time > 0.0) {
                data.dynamic_requests.push_back(request);
            } else if (request.id > 0) {
                // Match original Java semantics: customer IDs are 1-based and stored as
                // 0-based internal node indexes by subtracting 1 (depot id 0 is excluded).
                data.available_request_ids.push_back(request.id - 1);
            }
            continue;
        }

        if (line.rfind("NUMBER", 0) == 0) {
            read_general_on_next_line = true;
            continue;
        }
        if (line.rfind("CUST NO.", 0) == 0) {
            in_requests_section = true;
            continue;
        }
    }

    if (data.problem.vehicle_count <= 0 || data.problem.capacity <= 0) {
        throw std::runtime_error("Missing or invalid vehicle/capacity data in: '" + file_path_.string() + "'");
    }
    if (data.requests.empty()) {
        throw std::runtime_error("No request records found in: '" + file_path_.string() + "'");
    }

    data.problem.nodes.reserve(data.requests.size());
    for (const auto& request : data.requests) {
        data.problem.nodes.push_back(Point{.x = request.x_coord, .y = request.y_coord});
    }
    std::sort(data.dynamic_requests.begin(), data.dynamic_requests.end());

    return data;
}

}  // namespace acs_km
