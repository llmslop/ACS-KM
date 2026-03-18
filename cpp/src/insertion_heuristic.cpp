#include "acs_km/insertion_heuristic.hpp"

#include <algorithm>
#include <limits>
#include <unordered_map>

namespace acs_km {

void compute_route_variables(AntSolution& ant, const InstanceData& vrp, const int start_tour_index) {
    const auto& req = vrp.requests;

    for (int index = start_tour_index; index < ant.used_vehicles; ++index) {
        const auto tour_length = ant.tours[static_cast<std::size_t>(index)].size();
        if (index >= static_cast<int>(ant.earliest_time.size())) {
            ant.earliest_time.resize(static_cast<std::size_t>(index + 1));
        }
        ant.earliest_time[static_cast<std::size_t>(index)].assign(tour_length, 0.0);
        for (std::size_t i = 0; i < tour_length; ++i) {
            const auto city = ant.tours[static_cast<std::size_t>(index)][i];
            if ((city + 1) == 0 && i == 0) {
                ant.earliest_time[static_cast<std::size_t>(index)][i] = req[0].start_window;
            } else {
                const auto previous_city = ant.tours[static_cast<std::size_t>(index)][i - 1];
                const auto value1 = req[static_cast<std::size_t>(city + 1)].start_window;
                const auto value2 = ant.earliest_time[static_cast<std::size_t>(index)][i - 1] +
                                    vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(city + 1)] +
                                    req[static_cast<std::size_t>(previous_city + 1)].service_time;
                ant.earliest_time[static_cast<std::size_t>(index)][i] = std::max(value1, value2);
            }
        }
    }

    for (int index = start_tour_index; index < ant.used_vehicles; ++index) {
        const auto tour_length = ant.tours[static_cast<std::size_t>(index)].size();
        if (index >= static_cast<int>(ant.latest_time.size())) {
            ant.latest_time.resize(static_cast<std::size_t>(index + 1));
        }
        ant.latest_time[static_cast<std::size_t>(index)].assign(tour_length, 0.0);
        for (int i = static_cast<int>(tour_length) - 1; i >= 0; --i) {
            const auto city = ant.tours[static_cast<std::size_t>(index)][static_cast<std::size_t>(i)];
            if (i == static_cast<int>(tour_length) - 1) {
                const auto value1 = req[static_cast<std::size_t>(city + 1)].end_window;
                const auto value2 = req[0].end_window -
                                    vrp.problem.distance[static_cast<std::size_t>(city + 1)][0] -
                                    req[static_cast<std::size_t>(city + 1)].service_time;
                ant.latest_time[static_cast<std::size_t>(index)][static_cast<std::size_t>(i)] = std::min(value1, value2);
            } else {
                const auto next_city = ant.tours[static_cast<std::size_t>(index)][static_cast<std::size_t>(i + 1)];
                const auto value1 = req[static_cast<std::size_t>(city + 1)].end_window;
                const auto value2 = ant.latest_time[static_cast<std::size_t>(index)][static_cast<std::size_t>(i + 1)] -
                                    vrp.problem.distance[static_cast<std::size_t>(city + 1)][static_cast<std::size_t>(next_city + 1)] -
                                    req[static_cast<std::size_t>(city + 1)].service_time;
                ant.latest_time[static_cast<std::size_t>(index)][static_cast<std::size_t>(i)] = std::min(value1, value2);
            }
        }
    }
}

void update_route_variables(AntSolution& ant, const InstanceData& vrp, const Insertion& insertion) {
    const auto& req = vrp.requests;
    const auto best_index_tour = insertion.index_tour;
    const auto best_pos = insertion.previous_node;

    for (int k = best_pos - 1; k >= 0; --k) {
        const auto previous_city = ant.tours[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(k)];
        const auto next_city = ant.tours[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(k + 1)];
        const auto old_latest = ant.latest_time[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(k)];
        const auto value1 = ant.latest_time[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(k + 1)] -
                            vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(next_city + 1)] -
                            req[static_cast<std::size_t>(previous_city + 1)].service_time;
        const auto new_latest = std::min(old_latest, value1);
        if (old_latest != new_latest) {
            ant.latest_time[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(k)] = new_latest;
        } else {
            break;
        }
    }

    for (std::size_t k = static_cast<std::size_t>(best_pos + 1);
         k < ant.tours[static_cast<std::size_t>(best_index_tour)].size();
         ++k) {
        const auto previous_city = ant.tours[static_cast<std::size_t>(best_index_tour)][k - 1];
        const auto next_city = ant.tours[static_cast<std::size_t>(best_index_tour)][k];
        const auto old_earliest = ant.earliest_time[static_cast<std::size_t>(best_index_tour)][k];
        const auto value2 = ant.earliest_time[static_cast<std::size_t>(best_index_tour)][k - 1] +
                            vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(next_city + 1)] +
                            req[static_cast<std::size_t>(previous_city + 1)].service_time;
        const auto new_earliest = std::max(old_earliest, value2);
        if (old_earliest != new_earliest) {
            ant.earliest_time[static_cast<std::size_t>(best_index_tour)][k] = new_earliest;
        } else {
            break;
        }
    }
}

bool check_is_feasible_tour(const AntSolution& ant,
                            const InstanceData& vrp,
                            const int customer,
                            const int index_tour,
                            const int /*previous_pos*/,
                            const int next_pos) {
    const auto& req = vrp.requests;
    auto current_quantity = req[0].demand;
    auto current_time = 0.0;

    for (int current_pos = 1; current_pos < next_pos; ++current_pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(current_pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(current_pos)];
        current_quantity += req[static_cast<std::size_t>(current_city + 1)].demand;
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        const auto begin_service = std::max(arrival_time, req[static_cast<std::size_t>(current_city + 1)].start_window);
        if (begin_service > req[static_cast<std::size_t>(current_city + 1)].end_window) {
            return false;
        }
        current_time = begin_service;
    }

    const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(next_pos - 1)];
    auto current_city = customer;
    current_quantity += req[static_cast<std::size_t>(current_city + 1)].demand;
    auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
    auto arrival_time = current_time + req[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
    auto begin_service = std::max(arrival_time, req[static_cast<std::size_t>(current_city + 1)].start_window);
    if (begin_service > req[static_cast<std::size_t>(current_city + 1)].end_window) {
        return false;
    }
    current_time = begin_service;

    current_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(next_pos)];
    current_quantity += req[static_cast<std::size_t>(current_city + 1)].demand;
    distance = vrp.problem.distance[static_cast<std::size_t>(customer + 1)][static_cast<std::size_t>(current_city + 1)];
    arrival_time = current_time + req[static_cast<std::size_t>(customer + 1)].service_time + distance;
    begin_service = std::max(arrival_time, req[static_cast<std::size_t>(current_city + 1)].start_window);
    if (begin_service > req[static_cast<std::size_t>(current_city + 1)].end_window) {
        return false;
    }

    return current_quantity <= static_cast<double>(vrp.problem.capacity);
}

bool is_feasible_insertion(const AntSolution& ant,
                           const InstanceData& vrp,
                           const int customer,
                           const int index_tour,
                           const int previous_pos,
                           const int next_pos) {
    const auto& req = vrp.requests;

    const auto previous_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(previous_pos)];
    const auto next_city = ant.tours[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(next_pos)];
    const auto current_quantity = ant.current_quantity[static_cast<std::size_t>(index_tour)] + req[static_cast<std::size_t>(customer + 1)].demand;
    const auto arrival_time =
        ant.begin_service[static_cast<std::size_t>(previous_city + 1)] +
        req[static_cast<std::size_t>(previous_city + 1)].service_time +
        vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(customer + 1)];
    const auto begin_service = std::max(arrival_time, req[static_cast<std::size_t>(customer + 1)].start_window);
    const auto earliest_time = std::max(
        req[static_cast<std::size_t>(customer + 1)].start_window,
        ant.earliest_time[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(previous_pos)] +
            vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(customer + 1)] +
            req[static_cast<std::size_t>(previous_city + 1)].service_time);
    const auto latest_time = std::min(
        req[static_cast<std::size_t>(customer + 1)].end_window,
        ant.latest_time[static_cast<std::size_t>(index_tour)][static_cast<std::size_t>(next_pos)] -
            vrp.problem.distance[static_cast<std::size_t>(customer + 1)][static_cast<std::size_t>(next_city + 1)] -
            req[static_cast<std::size_t>(customer + 1)].service_time);

    return (current_quantity <= vrp.problem.capacity) &&
           (earliest_time <= latest_time) &&
           (begin_service <= req[static_cast<std::size_t>(customer + 1)].end_window);
}

void insert_unrouted_customers(AntSolution& ant,
                               const InstanceData& vrp,
                               const std::vector<int>& unvisited_nodes,
                               const int start_index_tour,
                               const std::vector<int>& start_pos) {
    const auto& req = vrp.requests;
    auto ok = true;
    constexpr auto mu = 1.0;
    constexpr auto alfa1 = 0.1;
    constexpr auto alfa2 = 0.9;
    constexpr auto lambda = 2.0;
    std::unordered_map<int, bool> visited;
    for (const auto node : unvisited_nodes) {
        visited[node] = false;
    }

    compute_route_variables(ant, vrp, start_index_tour);

    while (ok) {
        std::vector<Insertion> best_insertions;

        for (const auto customer : unvisited_nodes) {
            if (visited[customer]) {
                continue;
            }
            auto best_c1_score = std::numeric_limits<double>::max();
            auto best_index_tour = 0;
            auto best_pos = 1;
            for (int index_tour = start_index_tour; index_tour < ant.used_vehicles; ++index_tour) {
                auto start_index = 1;
                if (index_tour <= static_cast<int>(start_pos.size()) - 1) {
                    start_index = start_pos[static_cast<std::size_t>(index_tour)] + 1;
                }
                for (std::size_t pos = static_cast<std::size_t>(start_index);
                     pos < ant.tours[static_cast<std::size_t>(index_tour)].size();
                     ++pos) {
                    if (is_feasible_insertion(ant, vrp, customer, index_tour, static_cast<int>(pos - 1), static_cast<int>(pos))) {
                        const auto previous_city = ant.tours[static_cast<std::size_t>(index_tour)][pos - 1];
                        const auto next_city = ant.tours[static_cast<std::size_t>(index_tour)][pos];
                        const auto c11 = vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(customer + 1)] +
                                         vrp.problem.distance[static_cast<std::size_t>(customer + 1)][static_cast<std::size_t>(next_city + 1)] -
                                         mu * vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(next_city + 1)];
                        const auto arrival_time = ant.begin_service[static_cast<std::size_t>(previous_city + 1)] +
                                                  req[static_cast<std::size_t>(previous_city + 1)].service_time +
                                                  vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(customer + 1)];
                        const auto begin_service = std::max(arrival_time, req[static_cast<std::size_t>(customer + 1)].start_window);
                        const auto new_arrival_time = begin_service + req[static_cast<std::size_t>(customer + 1)].service_time +
                                                      vrp.problem.distance[static_cast<std::size_t>(customer + 1)][static_cast<std::size_t>(next_city + 1)];
                        const auto new_begin_service = std::max(new_arrival_time, req[static_cast<std::size_t>(next_city + 1)].start_window);
                        const auto old_begin_service = ant.begin_service[static_cast<std::size_t>(next_city + 1)];
                        const auto c12 = new_begin_service - old_begin_service;
                        const auto c1 = alfa1 * c11 + alfa2 * c12;
                        if (c1 < best_c1_score) {
                            best_c1_score = c1;
                            best_index_tour = index_tour;
                            best_pos = static_cast<int>(pos);
                        }
                    }
                }
            }
            if (best_c1_score != std::numeric_limits<double>::max()) {
                best_insertions.push_back(Insertion{
                    .customer = customer,
                    .index_tour = best_index_tour,
                    .previous_node = best_pos,
                    .score = best_c1_score,
                });
            }
        }

        auto best_c2_score = std::numeric_limits<double>::max();
        auto best_index_insertion = -1;
        for (std::size_t i = 0; i < best_insertions.size(); ++i) {
            const auto& insert = best_insertions[i];
            const auto c2 = lambda * vrp.problem.distance[0][static_cast<std::size_t>(insert.customer + 1)] - insert.score;
            if (c2 < best_c2_score) {
                best_c2_score = c2;
                best_index_insertion = static_cast<int>(i);
            }
        }

        if (best_index_insertion < 0) {
            ok = false;
            continue;
        }

        const auto best_insertion = best_insertions[static_cast<std::size_t>(best_index_insertion)];
        const auto best_customer = best_insertion.customer;
        const auto best_index_tour = best_insertion.index_tour;
        const auto best_pos = best_insertion.previous_node;
        ant.tours[static_cast<std::size_t>(best_index_tour)].insert(
            ant.tours[static_cast<std::size_t>(best_index_tour)].begin() + best_pos,
            best_customer);

        ant.visited[static_cast<std::size_t>(best_customer)] = true;
        ant.to_visit -= 1;
        ant.current_quantity[static_cast<std::size_t>(best_index_tour)] += req[static_cast<std::size_t>(best_customer + 1)].demand;
        visited[best_customer] = true;

        const auto previous_city = ant.tours[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(best_pos - 1)];
        const auto next_city = ant.tours[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(best_pos + 1)];
        const auto earliest_time = std::max(
            req[static_cast<std::size_t>(best_customer + 1)].start_window,
            ant.earliest_time[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(best_pos - 1)] +
                vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(best_customer + 1)] +
                req[static_cast<std::size_t>(previous_city + 1)].service_time);
        const auto latest_time = std::min(
            req[static_cast<std::size_t>(best_customer + 1)].end_window,
            ant.latest_time[static_cast<std::size_t>(best_index_tour)][static_cast<std::size_t>(best_pos)] -
                vrp.problem.distance[static_cast<std::size_t>(best_customer + 1)][static_cast<std::size_t>(next_city + 1)] -
                req[static_cast<std::size_t>(best_customer + 1)].service_time);
        ant.earliest_time[static_cast<std::size_t>(best_index_tour)].insert(
            ant.earliest_time[static_cast<std::size_t>(best_index_tour)].begin() + best_pos,
            earliest_time);
        ant.latest_time[static_cast<std::size_t>(best_index_tour)].insert(
            ant.latest_time[static_cast<std::size_t>(best_index_tour)].begin() + best_pos,
            latest_time);

        update_route_variables(ant, vrp, best_insertion);

        auto begin_service = 0.0;
        for (std::size_t j = static_cast<std::size_t>(best_pos);
             j < ant.tours[static_cast<std::size_t>(best_index_tour)].size();
             ++j) {
            const auto previous = ant.tours[static_cast<std::size_t>(best_index_tour)][j - 1];
            const auto cust = ant.tours[static_cast<std::size_t>(best_index_tour)][j];
            const auto arrival = ant.begin_service[static_cast<std::size_t>(previous + 1)] +
                                 req[static_cast<std::size_t>(previous + 1)].service_time +
                                 vrp.problem.distance[static_cast<std::size_t>(previous + 1)][static_cast<std::size_t>(cust + 1)];
            begin_service = std::max(arrival, req[static_cast<std::size_t>(cust + 1)].start_window);
            ant.begin_service[static_cast<std::size_t>(cust + 1)] = begin_service;
        }
        ant.current_time[static_cast<std::size_t>(best_index_tour)] = begin_service;
    }
}

}  // namespace acs_km
