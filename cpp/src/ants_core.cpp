#include "acs_km/ants_core.hpp"

#include <algorithm>
#include <limits>

namespace acs_km {

double heuristic(const Problem& problem, const int m, const int n) {
    return 1.0 / problem.distance[static_cast<std::size_t>(m)][static_cast<std::size_t>(n)];
}

void allocate_ants(AntAlgorithmState& ants, const InstanceData& instance) {
    const auto customer_count = instance.customer_count();
    ants.committed_nodes.assign(customer_count, false);

    ants.best_so_far_ant = AntSolution{};
    ants.best_so_far_ant.used_vehicles = 1;
    ants.best_so_far_ant.tours = {{-1, -1}};
    ants.best_so_far_ant.tour_lengths = {0.0};
    ants.best_so_far_ant.begin_service.assign(customer_count + 1, 0.0);
    ants.best_so_far_ant.current_time = {0.0};
    ants.best_so_far_ant.current_quantity = {0.0};
    ants.best_so_far_ant.visited.assign(customer_count, false);
    ants.best_so_far_ant.to_visit = static_cast<int>(instance.available_request_ids.size());
    ants.best_so_far_ant.longest_tour_length = std::numeric_limits<double>::max();
    ants.best_so_far_ant.cost_objectives = {0.0, 0.0};
    ants.best_so_far_ant.earliest_time = {{}};
    ants.best_so_far_ant.latest_time = {{}};
}

int find_best(const std::vector<AntSolution>& population) {
    if (population.empty()) {
        return -1;
    }

    auto min_vehicles = population[0].used_vehicles;
    for (std::size_t i = 1; i < population.size(); ++i) {
        if (population[i].used_vehicles < min_vehicles) {
            min_vehicles = population[i].used_vehicles;
        }
    }

    auto min_distance = std::numeric_limits<double>::max();
    auto min_index = 0;
    for (std::size_t i = 0; i < population.size(); ++i) {
        if (population[i].used_vehicles == min_vehicles && population[i].total_tour_length < min_distance) {
            min_distance = population[i].total_tour_length;
            min_index = static_cast<int>(i);
        }
    }
    return min_index;
}

void init_pheromone_trails(AntAlgorithmState& ants, const int customer_count, const double initial_trail) {
    ants.pheromone.assign(static_cast<std::size_t>(customer_count + 1),
                          std::vector<double>(static_cast<std::size_t>(customer_count + 1), initial_trail));
}

void preserve_pheromones(AntAlgorithmState& ants, const InstanceData& vrp, const double pheromone_preservation) {
    const auto customer_count = static_cast<int>(vrp.customer_count());
    for (int i = 0; i <= customer_count; ++i) {
        for (int j = 0; j <= i; ++j) {
            const auto i_available = std::find(vrp.available_request_ids.begin(), vrp.available_request_ids.end(), i - 1) !=
                                     vrp.available_request_ids.end();
            const auto j_available = std::find(vrp.available_request_ids.begin(), vrp.available_request_ids.end(), j - 1) !=
                                     vrp.available_request_ids.end();
            if ((i_available && j_available) || ((i == 0) && j_available) || ((j == 0) && i_available)) {
                ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * (1 - pheromone_preservation) +
                    pheromone_preservation * ants.trail_0;
                ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] =
                    ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            } else {
                ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = ants.trail_0;
                ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = ants.trail_0;
            }
        }
    }
}

void evaporation(AntAlgorithmState& ants, const int customer_count) {
    for (int i = 0; i <= customer_count; ++i) {
        for (int j = 0; j <= i; ++j) {
            ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                (1 - ants.rho) * ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] =
                ants.pheromone[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
        }
    }
}

}  // namespace acs_km
