#include "acs_km/vrptw_acs_core.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

#include "acs_km/controller_utils.hpp"

namespace acs_km {

bool termination_condition(const double elapsed_seconds, const double max_time_seconds) {
    return elapsed_seconds >= max_time_seconds;
}

bool is_feasible(const InstanceData& vrp,
                 const AntSolution& ant,
                 const int city,
                 const double begin_service,
                 const double begin_service_depot,
                 const int index_salesman) {
    if (vrp.requests.empty()) {
        return false;
    }
    if (index_salesman < 0 || static_cast<std::size_t>(index_salesman) >= ant.current_quantity.size()) {
        return false;
    }
    const auto& req_list = vrp.requests;
    if (city < 0 || static_cast<std::size_t>(city + 1) >= req_list.size()) {
        return false;
    }

    const auto current_quantity = ant.current_quantity[static_cast<std::size_t>(index_salesman)] +
                                  static_cast<double>(req_list[static_cast<std::size_t>(city + 1)].demand);
    return begin_service <= req_list[static_cast<std::size_t>(city + 1)].end_window &&
           current_quantity <= static_cast<double>(vrp.problem.capacity) &&
           begin_service_depot <= req_list[0].end_window;
}

void generate_initial_weights(AntAlgorithmState& ants, UtilitiesCore& random_source) {
    auto nr1 = random_source.random01();
    if (nr1 == 0.0) {
        nr1 = 1e-12;
    }
    ants.initial_weight1 = nr1;
    const auto nr2 = random_source.random01() * (1.0 - nr1);
    ants.initial_weight2 = nr2;
    ants.initial_weight3 = 1.0 - nr1 - nr2;
}

bool is_done(const AntAlgorithmState& ants) {
    return std::none_of(ants.ants.begin(), ants.ants.end(), [](const auto& ant) {
        return ant.to_visit > 0;
    });
}

bool check_committed_tours(const AntAlgorithmState& ants) {
    for (int index = 0; index < ants.best_so_far_ant.used_vehicles; ++index) {
        if (get_last_committed_pos(ants, index) > 0) {
            return true;
        }
    }
    return false;
}

void add_committed_nodes(AntSolution& ant,
                         const AntAlgorithmState& ants,
                         const InstanceData& instance) {
    const auto& req_list = instance.requests;
    if (req_list.empty()) {
        return;
    }

    auto start_index = 1;
    for (int i = 0; i < ants.best_so_far_ant.used_vehicles; ++i) {
        const auto index = get_last_committed_pos(ants, i);
        if (index <= 0) {
            continue;
        }

        if (ant.used_vehicles < (i + 1)) {
            ant.used_vehicles = i + 1;
            for (int l = start_index; l < ant.used_vehicles; ++l) {
                ant.tours.emplace_back(std::vector<int>{-1});
                ant.tour_lengths.push_back(0.0);
                ant.current_quantity.push_back(0.0);
                ant.current_time.push_back(0.0);
            }
            start_index = i + 1;
        }

        if (ant.tours[static_cast<std::size_t>(i)].empty()) {
            ant.tours[static_cast<std::size_t>(i)].push_back(-1);
        }
        const auto last_pos = ant.tours[static_cast<std::size_t>(i)].size() - 1;
        auto current_city = ant.tours[static_cast<std::size_t>(i)][last_pos] + 1;

        for (int j = 1; j <= index; ++j) {
            const auto city = ants.best_so_far_ant.tours[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            const auto distance = instance.problem.distance[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(city + 1)];
            const auto arrival_time = ant.current_time[static_cast<std::size_t>(i)] +
                                      req_list[static_cast<std::size_t>(current_city)].service_time + distance;
            const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(city + 1)].start_window);
            if (begin_service > req_list[static_cast<std::size_t>(city + 1)].end_window) {
                throw std::runtime_error("Method add_committed_nodes: solution infeasible");
            }

            ant.tours[static_cast<std::size_t>(i)].insert(
                ant.tours[static_cast<std::size_t>(i)].begin() + static_cast<std::ptrdiff_t>(j),
                city);
            if (city >= 0 && static_cast<std::size_t>(city) < ant.visited.size()) {
                ant.visited[static_cast<std::size_t>(city)] = true;
            }
            ant.to_visit -= 1;
            ant.current_time[static_cast<std::size_t>(i)] = begin_service;
            if (static_cast<std::size_t>(city + 1) < ant.begin_service.size()) {
                ant.begin_service[static_cast<std::size_t>(city + 1)] = begin_service;
            }
            ant.current_quantity[static_cast<std::size_t>(i)] += req_list[static_cast<std::size_t>(city + 1)].demand;
            current_city = city + 1;
        }
    }
}

}  // namespace acs_km
