#pragma once

#include <string>
#include <vector>

#include "acs_km/request.hpp"

namespace acs_km {

struct Point {
    int x{};
    int y{};
};

struct Problem {
    std::string name;
    int vehicle_count{};
    int capacity{};
    std::vector<Point> nodes;
};

struct InstanceData {
    Problem problem;
    std::vector<Request> requests;
    std::vector<Request> dynamic_requests;
    std::vector<int> available_request_ids;

    [[nodiscard]] std::size_t customer_count() const {
        return requests.empty() ? 0U : requests.size() - 1U;
    }
};

}  // namespace acs_km
