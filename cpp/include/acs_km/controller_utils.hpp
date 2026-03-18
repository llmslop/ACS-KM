#pragma once

#include <cstddef>
#include <vector>

#include "acs_km/ants_state.hpp"
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

inline int get_last_committed_pos(const AntAlgorithmState& ants, const int index_tour) {
    auto pos = 0;
    if (index_tour >= ants.best_so_far_ant.used_vehicles) {
        return pos;
    }

    const auto& tour = ants.best_so_far_ant.tours[static_cast<std::size_t>(index_tour)];
    if (tour.size() <= 2) {
        return pos;
    }
    for (std::size_t i = 1; i + 1 < tour.size(); ++i) {
        const auto node = tour[i];
        if (node >= 0 &&
            static_cast<std::size_t>(node) < ants.committed_nodes.size() &&
            ants.committed_nodes[static_cast<std::size_t>(node)]) {
            ++pos;
        } else {
            break;
        }
    }
    return pos;
}

inline bool check_new_committed_nodes(const AntAlgorithmState& ants,
                                      const AntSolution& best_ant,
                                      const int index_time_slice,
                                      const double length_time_slice) {
    auto index_tour = 0;
    while (index_tour < best_ant.used_vehicles) {
        const auto& tour = best_ant.tours[static_cast<std::size_t>(index_tour)];
        const auto start_pos = get_last_committed_pos(ants, index_tour);
        for (std::size_t i = static_cast<std::size_t>(start_pos + 1); i + 1 < tour.size(); ++i) {
            const auto node = tour[i];
            if (best_ant.begin_service[static_cast<std::size_t>(node + 1)] <=
                static_cast<double>(index_time_slice) * length_time_slice) {
                if (!ants.committed_nodes[static_cast<std::size_t>(node)]) {
                    return true;
                }
            } else {
                ++index_tour;
                break;
            }
        }
        if (index_tour < best_ant.used_vehicles) {
            const auto& current_tour = best_ant.tours[static_cast<std::size_t>(index_tour)];
            const auto current_start_pos = get_last_committed_pos(ants, index_tour);
            if (current_start_pos == static_cast<int>(current_tour.size()) - 2) {
                ++index_tour;
            }
        }
    }
    return false;
}

inline void commit_nodes(AntAlgorithmState& ants,
                         const AntSolution& best_ant,
                         const int index_time_slice,
                         const double length_time_slice) {
    auto index_tour = 0;
    while (index_tour < best_ant.used_vehicles) {
        const auto& tour = best_ant.tours[static_cast<std::size_t>(index_tour)];
        const auto start_pos = get_last_committed_pos(ants, index_tour);
        for (std::size_t i = static_cast<std::size_t>(start_pos + 1); i + 1 < tour.size(); ++i) {
            const auto node = tour[i];
            if ((best_ant.begin_service[static_cast<std::size_t>(node + 1)] <=
                 static_cast<double>(index_time_slice) * length_time_slice) &&
                !ants.committed_nodes[static_cast<std::size_t>(node)]) {
                ants.committed_nodes[static_cast<std::size_t>(node)] = true;
            } else {
                ++index_tour;
                break;
            }
        }
        if (index_tour < best_ant.used_vehicles) {
            const auto& current_tour = best_ant.tours[static_cast<std::size_t>(index_tour)];
            const auto current_start_pos = get_last_committed_pos(ants, index_tour);
            if (current_start_pos == static_cast<int>(current_tour.size()) - 2) {
                ++index_tour;
            }
        }
    }
}

}  // namespace acs_km
