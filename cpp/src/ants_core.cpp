#include "acs_km/ants_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "acs_km/vrptw_acs_core.hpp"
#include "acs_km/vrptw_utils.hpp"

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
    ants.restart_best_ant = ants.best_so_far_ant;
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

void ant_empty_memory(AntSolution& ant, const InstanceData& instance) {
    const auto customer_count = instance.customer_count();
    ant.total_tour_length = 0.0;
    ant.longest_tour_length = std::numeric_limits<double>::max();
    ant.index_longest_tour = 0;
    ant.added_empty_tour = false;

    ant.cost_objectives.assign(2, 0.0);
    ant.tour_lengths.clear();
    ant.current_quantity.clear();
    ant.current_time.clear();
    ant.tours.clear();
    ant.earliest_time.clear();
    ant.latest_time.clear();

    ant.used_vehicles = 1;
    ant.tour_lengths.push_back(0.0);
    ant.current_quantity.push_back(0.0);
    ant.current_time.push_back(0.0);
    ant.tours.push_back({});

    ant.visited.assign(customer_count, false);
    ant.begin_service.assign(customer_count + 1, 0.0);
    ant.to_visit = static_cast<int>(instance.available_request_ids.size());
}

void copy_from_to(const AntSolution& source, AntSolution& destination, const InstanceData& instance) {
    ant_empty_memory(destination, instance);
    destination.total_tour_length = source.total_tour_length;
    destination.to_visit = source.to_visit;
    destination.cost_objectives = source.cost_objectives;
    destination.added_empty_tour = source.added_empty_tour;

    if (destination.used_vehicles < source.used_vehicles) {
        for (int index = destination.used_vehicles; index < source.used_vehicles; ++index) {
            destination.tour_lengths.push_back(0.0);
            destination.tours.emplace_back();
            destination.current_quantity.push_back(0.0);
            destination.current_time.push_back(0.0);
        }
    }

    for (int i = 0; i < source.used_vehicles; ++i) {
        destination.tour_lengths[static_cast<std::size_t>(i)] = source.tour_lengths[static_cast<std::size_t>(i)];
        destination.current_quantity[static_cast<std::size_t>(i)] = source.current_quantity[static_cast<std::size_t>(i)];
        destination.current_time[static_cast<std::size_t>(i)] = source.current_time[static_cast<std::size_t>(i)];
        destination.tours[static_cast<std::size_t>(i)] = source.tours[static_cast<std::size_t>(i)];
    }

    destination.visited = source.visited;
    destination.used_vehicles = source.used_vehicles;
    destination.begin_service = source.begin_service;
    destination.longest_tour_length = source.longest_tour_length;
    destination.index_longest_tour = source.index_longest_tour;
}

std::vector<int> unrouted_customers(const AntSolution& ant, const InstanceData& vrp) {
    std::vector<int> customers;
    customers.reserve(static_cast<std::size_t>(std::max(0, ant.to_visit)));
    for (const auto city : vrp.available_request_ids) {
        if (city >= 0 && static_cast<std::size_t>(city) < ant.visited.size() &&
            !ant.visited[static_cast<std::size_t>(city)]) {
            customers.push_back(city);
            if (static_cast<int>(customers.size()) == ant.to_visit) {
                break;
            }
        }
    }
    return customers;
}

double compute_tours_amplitude(const AntSolution& ant) {
    if (ant.tour_lengths.empty()) {
        return 0.0;
    }
    auto min_v = ant.tour_lengths.front();
    auto max_v = ant.tour_lengths.front();
    for (const auto value : ant.tour_lengths) {
        if (value < min_v) {
            min_v = value;
        }
        if (value > max_v) {
            max_v = value;
        }
    }
    return max_v - min_v;
}

void global_update_pheromone(AntAlgorithmState& ants, const AntSolution& ant) {
    if (ant.total_tour_length <= 0.0) {
        return;
    }
    const auto d_tau = 1.0 / ant.total_tour_length;
    for (int i = 0; i < ant.used_vehicles; ++i) {
        const auto& tour = ant.tours[static_cast<std::size_t>(i)];
        for (std::size_t k = 0; k + 1 < tour.size(); ++k) {
            const auto j = tour[k] + 1;
            const auto h = tour[k + 1] + 1;
            ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)] += d_tau;
            ants.pheromone[static_cast<std::size_t>(h)][static_cast<std::size_t>(j)] =
                ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)];
        }
    }
}

void global_acs_pheromone_update(AntAlgorithmState& ants, const AntSolution& ant) {
    if (ant.total_tour_length <= 0.0) {
        return;
    }
    const auto d_tau = 1.0 / ant.total_tour_length;
    for (int i = 0; i < ant.used_vehicles; ++i) {
        const auto& tour = ant.tours[static_cast<std::size_t>(i)];
        for (std::size_t k = 0; k + 1 < tour.size(); ++k) {
            const auto j = tour[k] + 1;
            const auto h = tour[k + 1] + 1;
            ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)] =
                (1.0 - ants.rho) * ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)] + ants.rho * d_tau;
            ants.pheromone[static_cast<std::size_t>(h)][static_cast<std::size_t>(j)] =
                ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)];
        }
    }
}

void local_acs_pheromone_update(AntAlgorithmState& ants, const AntSolution& ant, const int index_salesman) {
    if (index_salesman < 0 || static_cast<std::size_t>(index_salesman) >= ant.tours.size()) {
        return;
    }
    const auto& tour = ant.tours[static_cast<std::size_t>(index_salesman)];
    if (tour.size() < 2) {
        return;
    }
    const auto j = tour.back() + 1;
    const auto h = tour[tour.size() - 2] + 1;
    ants.pheromone[static_cast<std::size_t>(h)][static_cast<std::size_t>(j)] =
        (1.0 - ants.local_rho) * ants.pheromone[static_cast<std::size_t>(h)][static_cast<std::size_t>(j)] +
        ants.local_rho * ants.trail_0;
    ants.pheromone[static_cast<std::size_t>(j)][static_cast<std::size_t>(h)] =
        ants.pheromone[static_cast<std::size_t>(h)][static_cast<std::size_t>(j)];
}

namespace {
void choose_closest_next(AntSolution& ant, const InstanceData& instance, const int index_salesman) {
    int next_city = static_cast<int>(instance.customer_count());
    const auto& req_list = instance.requests;

    const auto last_pos = ant.tours[static_cast<std::size_t>(index_salesman)].size() - 1;
    auto current_city = ant.tours[static_cast<std::size_t>(index_salesman)][last_pos];
    current_city++;

    auto min_value = std::numeric_limits<double>::max();
    auto best_begin_service = 0.0;
    for (const auto city : instance.available_request_ids) {
        if (ant.visited[static_cast<std::size_t>(city)]) {
            continue;
        }
        const auto distance = instance.problem.distance[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(city + 1)];
        const auto arrival_time = ant.current_time[static_cast<std::size_t>(index_salesman)] +
                                  req_list[static_cast<std::size_t>(current_city)].service_time + distance;
        const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(city + 1)].start_window);

        const auto distance_depot = instance.problem.distance[static_cast<std::size_t>(city + 1)][0];
        const auto arrival_time_depot = begin_service + req_list[static_cast<std::size_t>(city + 1)].service_time + distance_depot;
        const auto begin_service_depot = std::max(arrival_time_depot, req_list[0].start_window);

        if (!is_feasible(instance, ant, city, begin_service, begin_service_depot, index_salesman)) {
            continue;
        }
        const auto time_difference = begin_service - ant.begin_service[static_cast<std::size_t>(current_city)] -
                                     req_list[static_cast<std::size_t>(current_city)].service_time;
        const auto delivery_urgency = req_list[static_cast<std::size_t>(city + 1)].end_window -
                                      (ant.begin_service[static_cast<std::size_t>(current_city)] +
                                       req_list[static_cast<std::size_t>(current_city)].service_time + distance);
        const auto metric_value = distance + time_difference + delivery_urgency;
        if (metric_value < min_value) {
            next_city = city;
            min_value = metric_value;
            best_begin_service = begin_service;
        }
    }

    if (next_city == static_cast<int>(instance.customer_count())) {
        ant.used_vehicles++;
        ant.tours.emplace_back(std::vector<int>{-1});
        ant.tour_lengths.push_back(0.0);
        ant.current_quantity.push_back(0.0);
        ant.current_time.push_back(0.0);
        return;
    }

    ant.tours[static_cast<std::size_t>(index_salesman)].push_back(next_city);
    ant.visited[static_cast<std::size_t>(next_city)] = true;
    ant.to_visit--;
    ant.current_time[static_cast<std::size_t>(index_salesman)] = best_begin_service;
    ant.begin_service[static_cast<std::size_t>(next_city + 1)] = best_begin_service;
    ant.current_quantity[static_cast<std::size_t>(index_salesman)] += req_list[static_cast<std::size_t>(next_city + 1)].demand;
}
}  // namespace

double nn_tour(AntAlgorithmState& ants, const InstanceData& instance) {
    if (ants.ants.empty()) {
        ants.ants.push_back(AntSolution{});
    }
    auto& ant = ants.ants[0];
    ant_empty_memory(ant, instance);

    for (int i = 0; i < ant.used_vehicles; ++i) {
        ant.tours[static_cast<std::size_t>(i)].push_back(-1);
    }
    while (ant.to_visit > 0) {
        const auto salesman = ant.used_vehicles - 1;
        choose_closest_next(ant, instance, salesman);
    }

    auto sum = 0.0;
    for (int i = 0; i < ant.used_vehicles; ++i) {
        ant.tours[static_cast<std::size_t>(i)].push_back(-1);
        ant.tour_lengths[static_cast<std::size_t>(i)] =
            compute_tour_length_plus1_indexed(ant.tours[static_cast<std::size_t>(i)], instance.problem.distance);
        sum += ant.tour_lengths[static_cast<std::size_t>(i)];
    }
    ant.total_tour_length = sum;
    copy_from_to(ant, ants.best_so_far_ant, instance);
    ant_empty_memory(ant, instance);
    return sum;
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
