#pragma once

#include <cstddef>
#include <vector>

#include "acs_km/request.hpp"

namespace acs_km {

inline std::vector<int> collect_newly_available_nodes(const std::vector<Request>& dynamic_requests,
                                                  const double current_time,
                                                  std::size_t& last_known_dynamic_index) {
    std::vector<int> nodes;

    while (last_known_dynamic_index < dynamic_requests.size() &&
           current_time >= dynamic_requests[last_known_dynamic_index].available_time) {
        const auto id = dynamic_requests[last_known_dynamic_index].id;
        if (id > 0) {
            nodes.push_back(id - 1);
        }
        ++last_known_dynamic_index;
    }

    return nodes;
}

}  // namespace acs_km
