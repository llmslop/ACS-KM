#include "acs_km/vrptw_acs_core.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>

#include "acs_km/controller_utils.hpp"
#include "acs_km/insertion_heuristic.hpp"

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

int find_shortest_tour(const AntSolution& ant) {
    if (ant.tours.empty()) {
        return 0;
    }
    auto index_tour = 0;
    auto min_size = std::numeric_limits<std::size_t>::max();
    for (int i = 0; i < ant.used_vehicles && static_cast<std::size_t>(i) < ant.tours.size(); ++i) {
        if (ant.tours[static_cast<std::size_t>(i)].size() < min_size) {
            min_size = ant.tours[static_cast<std::size_t>(i)].size();
            index_tour = i;
        }
    }
    return index_tour;
}

int calc_tour_dist(const std::vector<int>& tour, const InstanceData& vrp) {
    if (tour.empty()) {
        return 0;
    }
    const auto& req_list = vrp.requests;
    auto current_time = 0.0;
    auto current_quantity = 0.0;
    auto total_distance = 0.0;

    for (std::size_t i = 1; i < tour.size(); ++i) {
        const auto prev_city = tour[i - 1];
        const auto current_city = tour[i];
        const auto prev_index = static_cast<std::size_t>(prev_city + 1);
        const auto curr_index = static_cast<std::size_t>(current_city + 1);
        const auto distance = vrp.problem.distance[prev_index][curr_index];
        current_quantity += req_list[curr_index].demand;
        if (current_quantity > static_cast<double>(vrp.problem.capacity)) {
            return -100000000;
        }
        const auto arrival_time = current_time + req_list[prev_index].service_time + distance;
        const auto begin_service = std::max(arrival_time, req_list[curr_index].start_window);
        if (begin_service > req_list[curr_index].end_window) {
            return -100000000;
        }
        current_time = begin_service;
        total_distance += distance;
    }

    const auto score = static_cast<int>(total_distance * 300.0);
    return -score;
}

bool is_better_solution(const AntSolution& candidate, const AntSolution& incumbent) {
    constexpr auto kTempNo = 1e10;
    const auto round1 = std::round(candidate.total_tour_length * kTempNo) / kTempNo;
    const auto round2 = std::round(incumbent.total_tour_length * kTempNo) / kTempNo;
    return (candidate.used_vehicles < incumbent.used_vehicles) ||
           ((candidate.used_vehicles == incumbent.used_vehicles) && (round1 < round2)) ||
           ((round1 < round2) && (incumbent.total_tour_length == std::numeric_limits<double>::max()));
}

namespace {

double random01() {
    static thread_local std::mt19937 generator(std::random_device{}());
    static thread_local std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(generator);
}

std::vector<int> committed_positions(const AntAlgorithmState& ants) {
    std::vector<int> committed;
    committed.reserve(static_cast<std::size_t>(ants.best_so_far_ant.used_vehicles));
    for (int index = 0; index < ants.best_so_far_ant.used_vehicles; ++index) {
        committed.push_back(get_last_committed_pos(ants, index));
    }
    return committed;
}

double selection_probability(const AntAlgorithmState& ants,
                             const InstanceData& instance,
                             const AntSolution& ant,
                             const int current_city,
                             const int city_plus_one,
                             const int index_salesman,
                             const double begin_service) {
    const auto& req_list = instance.requests;
    const auto distance =
        instance.problem.distance[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(city_plus_one)];
    const auto time_difference =
        begin_service - ant.begin_service[static_cast<std::size_t>(current_city)] -
        req_list[static_cast<std::size_t>(current_city)].service_time;
    const auto delivery_urgency =
        req_list[static_cast<std::size_t>(city_plus_one)].end_window -
        (ant.begin_service[static_cast<std::size_t>(current_city)] +
         req_list[static_cast<std::size_t>(current_city)].service_time + distance);

    auto w1 = ants.initial_weight1;
    auto w2 = ants.initial_weight2;
    auto w3 = ants.initial_weight3;
    if (w1 == 0.0 && w2 == 0.0 && w3 == 0.0) {
        w1 = 1.0;
        w2 = 1.0;
        w3 = 1.0;
    }
    const auto denominator = std::max(1e-12, w1 * distance + w2 * time_difference + w3 * delivery_urgency);
    auto help = 1.0 / denominator;
    help = std::pow(help, ants.beta);
    const auto pheromone = ants.pheromone.empty()
                               ? 1.0
                               : ants.pheromone[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(city_plus_one)];
    (void)index_salesman;
    return help * std::pow(pheromone, ants.alpha);
}

}  // namespace

void as_update(AntAlgorithmState& ants) {
    for (const auto& ant : ants.ants) {
        global_update_pheromone(ants, ant);
    }
}

void acs_global_update(AntAlgorithmState& ants) {
    global_acs_pheromone_update(ants, ants.best_so_far_ant);
}

void pheromone_trail_update(AntAlgorithmState& ants, const int customer_count) {
    if (ants.as_flag) {
        evaporation(ants, customer_count);
    }
    if (ants.as_flag) {
        as_update(ants);
    } else if (ants.acs_flag) {
        acs_global_update(ants);
    }
}

void init_try(const InstanceData& instance, AntAlgorithmState& ants, InOutState& inout) {
    inout.n_tours = 1;
    inout.iteration = 1;
    ants.best_so_far_ant.total_tour_length = std::numeric_limits<double>::max();
    ants.restart_best_ant.total_tour_length = std::numeric_limits<double>::max();
    inout.found_best = 0;
    inout.lambda = 0.05;

    if (!ants.acs_flag) {
        const auto nn = nn_tour(ants, instance);
        if (nn > 0.0 && ants.rho > 0.0) {
            ants.trail_0 = 1.0 / (ants.rho * nn);
        } else {
            ants.trail_0 = 1.0;
        }
        init_pheromone_trails(ants, static_cast<int>(instance.customer_count()), ants.trail_0);
    } else {
        const auto no_available_nodes = static_cast<int>(instance.available_request_ids.size());
        if (no_available_nodes == 0) {
            ants.trail_0 = 1.0;
        } else {
            const auto nn = nn_tour(ants, instance);
            if (nn > 0.0) {
                ants.trail_0 = 1.0 / (static_cast<double>(no_available_nodes + 1) * nn);
            } else {
                ants.trail_0 = 1.0;
            }
        }
        init_pheromone_trails(ants, static_cast<int>(instance.customer_count()), ants.trail_0);
    }
}

void update_statistics(const InstanceData& instance, AntAlgorithmState& ants, InOutState& inout) {
    const auto iteration_best_ant = find_best(ants.ants);
    if (iteration_best_ant < 0 || static_cast<std::size_t>(iteration_best_ant) >= ants.ants.size()) {
        return;
    }
    const auto& candidate = ants.ants[static_cast<std::size_t>(iteration_best_ant)];
    if (is_better_solution(candidate, ants.best_so_far_ant)) {
        copy_from_to(candidate, ants.best_so_far_ant, instance);
        inout.found_best = inout.iteration;
    }
    if (is_better_solution(candidate, ants.restart_best_ant)) {
        copy_from_to(candidate, ants.restart_best_ant, instance);
        inout.restart_found_best = inout.iteration;
    }
}

NextMove choose_best_next(AntSolution& ant, const InstanceData& instance, const AntAlgorithmState& ants) {
    const auto& req_list = instance.requests;
    auto next_city = static_cast<int>(instance.customer_count());
    auto salesman = 0;
    auto value_best = -1.0;
    auto best_begin_service = 0.0;

    auto start_index = 0;
    if (ant.added_empty_tour) {
        start_index = ant.used_vehicles - 1;
    }

    for (int index_salesman = start_index; index_salesman < ant.used_vehicles; ++index_salesman) {
        const auto last_pos = ant.tours[static_cast<std::size_t>(index_salesman)].size() - 1;
        auto current_city = ant.tours[static_cast<std::size_t>(index_salesman)][last_pos];
        current_city++;
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
            const auto help = selection_probability(ants, instance, ant, current_city, city + 1, index_salesman, begin_service);
            if (help > value_best) {
                value_best = help;
                next_city = city;
                salesman = index_salesman;
                best_begin_service = begin_service;
            }
        }
    }

    if (next_city == static_cast<int>(instance.customer_count())) {
        if ((ant.to_visit > 0) && (ant.to_visit <= 10)) {
            const auto unrouted = unrouted_customers(ant, instance);
            const auto starts = committed_positions(ants);
            insert_unrouted_customers(ant, instance, unrouted, 0, starts);
        }
        if (ant.to_visit > 0) {
            ant.used_vehicles++;
            const auto index_tour = ant.used_vehicles - 1;
            ant.tours.emplace_back(std::vector<int>{-1});
            ant.tour_lengths.push_back(0.0);
            ant.current_quantity.push_back(0.0);
            ant.current_time.push_back(0.0);
            ant.added_empty_tour = true;
            return {-1, index_tour};
        }
        return {-1, -1};
    }

    ant.tours[static_cast<std::size_t>(salesman)].push_back(next_city);
    ant.visited[static_cast<std::size_t>(next_city)] = true;
    ant.to_visit--;
    ant.current_time[static_cast<std::size_t>(salesman)] = best_begin_service;
    ant.begin_service[static_cast<std::size_t>(next_city + 1)] = best_begin_service;
    ant.current_quantity[static_cast<std::size_t>(salesman)] += req_list[static_cast<std::size_t>(next_city + 1)].demand;
    return {next_city, salesman};
}

NextMove neighbour_choose_best_next(AntSolution& ant, const InstanceData& instance, const AntAlgorithmState& ants) {
    const auto& req_list = instance.requests;
    const std::unordered_set<int> available(instance.available_request_ids.begin(), instance.available_request_ids.end());

    auto next_city = static_cast<int>(instance.customer_count());
    auto salesman = 0;
    auto value_best = -1.0;
    auto best_begin_service = 0.0;
    auto start_pos = ant.added_empty_tour ? ant.used_vehicles - 1 : 0;

    for (int index_salesman = start_pos; index_salesman < ant.used_vehicles; ++index_salesman) {
        const auto last_pos = ant.tours[static_cast<std::size_t>(index_salesman)].size() - 1;
        auto current_city = ant.tours[static_cast<std::size_t>(index_salesman)][last_pos];
        current_city++;
        const auto limit = std::min<int>(ants.nn_ants, static_cast<int>(instance.problem.nn_list[static_cast<std::size_t>(current_city)].size()));
        for (int i = 0; i < limit; ++i) {
            const auto help_city = instance.problem.nn_list[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(i)];
            const auto candidate = help_city - 1;
            if (available.find(candidate) == available.end() ||
                ant.visited[static_cast<std::size_t>(candidate)]) {
                continue;
            }
            const auto distance = instance.problem.distance[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(help_city)];
            const auto arrival_time = ant.current_time[static_cast<std::size_t>(index_salesman)] +
                                      req_list[static_cast<std::size_t>(current_city)].service_time + distance;
            const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(help_city)].start_window);
            const auto distance_depot = instance.problem.distance[static_cast<std::size_t>(help_city)][0];
            const auto arrival_time_depot = begin_service + req_list[static_cast<std::size_t>(help_city)].service_time + distance_depot;
            const auto begin_service_depot = std::max(arrival_time_depot, req_list[0].start_window);
            if (!is_feasible(instance, ant, candidate, begin_service, begin_service_depot, index_salesman)) {
                continue;
            }
            const auto help = selection_probability(ants, instance, ant, current_city, help_city, index_salesman, begin_service);
            if (help > value_best) {
                value_best = help;
                next_city = candidate;
                salesman = index_salesman;
                best_begin_service = begin_service;
            }
        }
    }

    if (next_city == static_cast<int>(instance.customer_count())) {
        return choose_best_next(ant, instance, ants);
    }

    ant.tours[static_cast<std::size_t>(salesman)].push_back(next_city);
    ant.visited[static_cast<std::size_t>(next_city)] = true;
    ant.to_visit--;
    ant.current_time[static_cast<std::size_t>(salesman)] = best_begin_service;
    ant.begin_service[static_cast<std::size_t>(next_city + 1)] = best_begin_service;
    ant.current_quantity[static_cast<std::size_t>(salesman)] += req_list[static_cast<std::size_t>(next_city + 1)].demand;
    return {next_city, salesman};
}

NextMove neighbour_choose_and_move_to_next(AntSolution& ant, const InstanceData& instance, AntAlgorithmState& ants) {
    if ((ants.q_0 > 0.0) && (random01() < ants.q_0)) {
        return neighbour_choose_best_next(ant, instance, ants);
    }

    struct Candidate {
        int salesman{};
        int current_city{};
        int city_plus_one{};
        double prob{};
        double begin_service{};
    };
    std::vector<Candidate> candidates;
    const auto& req_list = instance.requests;
    const std::unordered_set<int> available(instance.available_request_ids.begin(), instance.available_request_ids.end());

    for (int index_salesman = 0; index_salesman < ant.used_vehicles; ++index_salesman) {
        const auto last_pos = ant.tours[static_cast<std::size_t>(index_salesman)].size() - 1;
        auto current_city = ant.tours[static_cast<std::size_t>(index_salesman)][last_pos];
        current_city++;
        const auto limit = std::min<int>(ants.nn_ants, static_cast<int>(instance.problem.nn_list[static_cast<std::size_t>(current_city)].size()));
        for (int i = 0; i < limit; ++i) {
            const auto city_plus_one = instance.problem.nn_list[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(i)];
            const auto city = city_plus_one - 1;
            if (available.find(city) == available.end() || ant.visited[static_cast<std::size_t>(city)]) {
                continue;
            }
            const auto distance = instance.problem.distance[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(city_plus_one)];
            const auto arrival_time = ant.current_time[static_cast<std::size_t>(index_salesman)] +
                                      req_list[static_cast<std::size_t>(current_city)].service_time + distance;
            const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(city_plus_one)].start_window);
            const auto distance_depot = instance.problem.distance[static_cast<std::size_t>(city_plus_one)][0];
            const auto arrival_time_depot = begin_service + req_list[static_cast<std::size_t>(city_plus_one)].service_time + distance_depot;
            const auto begin_service_depot = std::max(arrival_time_depot, req_list[0].start_window);
            if (!is_feasible(instance, ant, city, begin_service, begin_service_depot, index_salesman)) {
                continue;
            }
            const auto prob = selection_probability(ants, instance, ant, current_city, city_plus_one, index_salesman, begin_service);
            if (prob > 0.0) {
                candidates.push_back(Candidate{
                    .salesman = index_salesman,
                    .current_city = current_city,
                    .city_plus_one = city_plus_one,
                    .prob = prob,
                    .begin_service = begin_service,
                });
            }
        }
    }

    if (candidates.empty()) {
        return choose_best_next(ant, instance, ants);
    }

    auto sum_prob = 0.0;
    for (const auto& c : candidates) {
        sum_prob += c.prob;
    }
    if (sum_prob <= 0.0) {
        return choose_best_next(ant, instance, ants);
    }

    auto rnd = random01() * sum_prob;
    const Candidate* chosen = nullptr;
    auto partial = 0.0;
    for (const auto& c : candidates) {
        partial += c.prob;
        if (partial >= rnd) {
            chosen = &c;
            break;
        }
    }
    if (chosen == nullptr) {
        chosen = &candidates.back();
    }

    const auto city = chosen->city_plus_one - 1;
    ant.tours[static_cast<std::size_t>(chosen->salesman)].push_back(city);
    ant.visited[static_cast<std::size_t>(city)] = true;
    ant.to_visit--;
    ant.current_time[static_cast<std::size_t>(chosen->salesman)] = chosen->begin_service;
    ant.begin_service[static_cast<std::size_t>(chosen->city_plus_one)] = chosen->begin_service;
    ant.current_quantity[static_cast<std::size_t>(chosen->salesman)] += req_list[static_cast<std::size_t>(chosen->city_plus_one)].demand;
    return {city, chosen->salesman};
}

void choose_closest_nn(AntSolution& ant, const int index_salesman, const InstanceData& instance, const AntAlgorithmState& ants) {
    const auto& req_list = instance.requests;
    while (ant.to_visit > 0) {
        auto next_city = static_cast<int>(instance.customer_count());
        const auto last_pos = ant.tours[static_cast<std::size_t>(index_salesman)].size() - 1;
        auto current_city = ant.tours[static_cast<std::size_t>(index_salesman)][last_pos];
        current_city++;

        auto min_value = std::numeric_limits<double>::max();
        auto best_begin_service = 0.0;
        const auto limit = std::min<int>(ants.nn_ants, static_cast<int>(instance.problem.nn_list[static_cast<std::size_t>(current_city)].size()));
        for (int i = 0; i < limit; ++i) {
            const auto city = instance.problem.nn_list[static_cast<std::size_t>(current_city)][static_cast<std::size_t>(i)] - 1;
            if (city < 0 || static_cast<std::size_t>(city) >= ant.visited.size() || ant.visited[static_cast<std::size_t>(city)]) {
                continue;
            }
            if (std::find(instance.available_request_ids.begin(), instance.available_request_ids.end(), city) ==
                instance.available_request_ids.end()) {
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
            auto w1 = ants.initial_weight1;
            auto w2 = ants.initial_weight2;
            auto w3 = ants.initial_weight3;
            if (w1 == 0.0 && w2 == 0.0 && w3 == 0.0) {
                w1 = 1.0;
                w2 = 1.0;
                w3 = 1.0;
            }
            const auto time_difference =
                begin_service - ant.begin_service[static_cast<std::size_t>(current_city)] -
                req_list[static_cast<std::size_t>(current_city)].service_time;
            const auto delivery_urgency =
                req_list[static_cast<std::size_t>(city + 1)].end_window -
                (ant.begin_service[static_cast<std::size_t>(current_city)] +
                 req_list[static_cast<std::size_t>(current_city)].service_time + distance);
            const auto metric = w1 * distance + w2 * time_difference + w3 * delivery_urgency;
            if (metric < min_value) {
                min_value = metric;
                next_city = city;
                best_begin_service = begin_service;
            }
        }

        if (next_city == static_cast<int>(instance.customer_count())) {
            break;
        }
        ant.tours[static_cast<std::size_t>(index_salesman)].push_back(next_city);
        ant.visited[static_cast<std::size_t>(next_city)] = true;
        ant.to_visit--;
        ant.current_time[static_cast<std::size_t>(index_salesman)] = best_begin_service;
        ant.begin_service[static_cast<std::size_t>(next_city + 1)] = best_begin_service;
        ant.current_quantity[static_cast<std::size_t>(index_salesman)] += req_list[static_cast<std::size_t>(next_city + 1)].demand;
    }
}

void construct_solutions(const InstanceData& instance, AntAlgorithmState& ants, InOutState& inout) {
    for (auto& ant : ants.ants) {
        ant_empty_memory(ant, instance);
    }

    for (auto& ant : ants.ants) {
        for (int i = 0; i < ant.used_vehicles; ++i) {
            ant.tours[static_cast<std::size_t>(i)].push_back(-1);
        }
    }

    if (check_committed_tours(ants)) {
        for (auto& ant : ants.ants) {
            add_committed_nodes(ant, ants, instance);
        }
    }

    while (!is_done(ants)) {
        for (auto& ant : ants.ants) {
            if (ant.to_visit > 0) {
                const auto move = neighbour_choose_and_move_to_next(ant, instance, ants);
                if (ants.acs_flag && move.salesman >= 0) {
                    local_acs_pheromone_update(ants, ant, move.salesman);
                }
            }
        }
    }

    for (auto& ant : ants.ants) {
        inout.no_solutions++;
        auto longest_tour_length = 0.0;
        auto id_longest_tour = 0;
        ant.total_tour_length = 0.0;
        for (int i = 0; i < ant.used_vehicles; ++i) {
            ant.tours[static_cast<std::size_t>(i)].push_back(-1);
            ant.tour_lengths[static_cast<std::size_t>(i)] =
                compute_tour_length_plus1_indexed(ant.tours[static_cast<std::size_t>(i)], instance.problem.distance);
            ant.total_tour_length += ant.tour_lengths[static_cast<std::size_t>(i)];
            if (ant.tour_lengths[static_cast<std::size_t>(i)] > longest_tour_length) {
                longest_tour_length = ant.tour_lengths[static_cast<std::size_t>(i)];
                id_longest_tour = i;
            }
            if (ants.acs_flag) {
                local_acs_pheromone_update(ants, ant, i);
            }
        }
        ant.longest_tour_length = longest_tour_length;
        ant.index_longest_tour = id_longest_tour;
        ant.cost_objectives = {ant.total_tour_length, compute_tours_amplitude(ant)};
    }
    if (!ants.ants.empty()) {
        inout.n_tours += ants.n_ants * ants.ants[0].used_vehicles;
    }
}

bool check_feasible_tour_relocation_multiple(const AntSolution& ant,
                                             const InstanceData& vrp,
                                             const int index_tour_source,
                                             const int index_tour_destination,
                                             const int i,
                                             const int j) {
    const auto& req_list = vrp.requests;

    const auto city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
    auto current_quantity = ant.current_quantity[static_cast<std::size_t>(index_tour_destination)] +
                            req_list[static_cast<std::size_t>(city + 1)].demand;
    if (current_quantity > vrp.problem.capacity) {
        return false;
    }

    auto current_time = 0.0;
    for (int pos = i + 1; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_source)].size()); ++pos) {
        auto prev_city = 0;
        auto current_city = 0;
        if (pos == (i + 1)) {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 2)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        } else {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 1)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        if (begin_service > req_list[static_cast<std::size_t>(current_city + 1)].end_window) {
            return false;
        }
        current_time = begin_service;
    }

    const auto previous_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j - 1)];
    const auto next_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)];
    auto arrival_time = ant.begin_service[static_cast<std::size_t>(previous_city + 1)] +
                        req_list[static_cast<std::size_t>(previous_city + 1)].service_time +
                        vrp.problem.distance[static_cast<std::size_t>(previous_city + 1)][static_cast<std::size_t>(city + 1)];
    auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(city + 1)].start_window);
    if (begin_service > req_list[static_cast<std::size_t>(city + 1)].end_window) {
        return false;
    }
    current_time = begin_service;

    arrival_time = current_time + req_list[static_cast<std::size_t>(city + 1)].service_time +
                   vrp.problem.distance[static_cast<std::size_t>(city + 1)][static_cast<std::size_t>(next_city + 1)];
    begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(next_city + 1)].start_window);
    if (begin_service > req_list[static_cast<std::size_t>(next_city + 1)].end_window) {
        return false;
    }
    current_time = begin_service;

    for (int pos = j + 1; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_destination)].size()); ++pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos)];
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        if (begin_service > req_list[static_cast<std::size_t>(current_city + 1)].end_window) {
            return false;
        }
        current_time = begin_service;
    }

    return true;
}

void update_begin_service_relocation_multiple(AntSolution& ant,
                                              const InstanceData& vrp,
                                              const int index_tour_source,
                                              const int index_tour_destination,
                                              const int i,
                                              const int j) {
    const auto& req_list = vrp.requests;
    auto current_time = 0.0;
    auto begin_service = 0.0;

    for (int pos = i; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_source)].size()) - 1; ++pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
        if (pos == i) {
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        current_time = begin_service;
        ant.begin_service[static_cast<std::size_t>(current_city + 1)] = begin_service;
    }
    ant.current_time[static_cast<std::size_t>(index_tour_source)] = begin_service;

    for (int pos = j; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_destination)].size()) - 1; ++pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos)];
        if (pos == j) {
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        current_time = begin_service;
        ant.begin_service[static_cast<std::size_t>(current_city + 1)] = begin_service;
    }
    ant.current_time[static_cast<std::size_t>(index_tour_destination)] = begin_service;
}

bool check_feasible_tour_exchange_multiple(const AntSolution& ant,
                                           const InstanceData& vrp,
                                           const int index_tour_source,
                                           const int index_tour_destination,
                                           const int i,
                                           const int j) {
    const auto& req_list = vrp.requests;
    const auto city1 = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
    const auto city2 = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)];
    auto current_quantity = ant.current_quantity[static_cast<std::size_t>(index_tour_source)] -
                            req_list[static_cast<std::size_t>(city1 + 1)].demand +
                            req_list[static_cast<std::size_t>(city2 + 1)].demand;
    if (current_quantity > vrp.problem.capacity) {
        return false;
    }
    current_quantity = ant.current_quantity[static_cast<std::size_t>(index_tour_destination)] -
                       req_list[static_cast<std::size_t>(city2 + 1)].demand +
                       req_list[static_cast<std::size_t>(city1 + 1)].demand;
    if (current_quantity > vrp.problem.capacity) {
        return false;
    }

    auto current_time = 0.0;
    for (int pos = i; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_source)].size()); ++pos) {
        auto prev_city = 0;
        auto current_city = 0;
        if (pos == i) {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 1)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)];
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        } else if (pos == (i + 1)) {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
        } else {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 1)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        if (begin_service > req_list[static_cast<std::size_t>(current_city + 1)].end_window) {
            return false;
        }
        current_time = begin_service;
    }

    for (int pos = j; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_destination)].size()); ++pos) {
        auto prev_city = 0;
        auto current_city = 0;
        if (pos == j) {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos - 1)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        } else if (pos == (j + 1)) {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos)];
        } else {
            prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos - 1)];
            current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        const auto begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        if (begin_service > req_list[static_cast<std::size_t>(current_city + 1)].end_window) {
            return false;
        }
        current_time = begin_service;
    }

    return true;
}

void update_begin_service_exchange_multiple(AntSolution& ant,
                                            const InstanceData& vrp,
                                            const int index_tour_source,
                                            const int index_tour_destination,
                                            const int i,
                                            const int j) {
    const auto& req_list = vrp.requests;
    auto current_time = 0.0;
    auto begin_service = 0.0;

    for (int pos = i; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_source)].size()) - 1; ++pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(pos)];
        if (pos == i) {
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        current_time = begin_service;
        ant.begin_service[static_cast<std::size_t>(current_city + 1)] = begin_service;
    }
    ant.current_time[static_cast<std::size_t>(index_tour_source)] = begin_service;

    for (int pos = j; pos < static_cast<int>(ant.tours[static_cast<std::size_t>(index_tour_destination)].size()) - 1; ++pos) {
        const auto prev_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos - 1)];
        const auto current_city = ant.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(pos)];
        if (pos == j) {
            current_time = ant.begin_service[static_cast<std::size_t>(prev_city + 1)];
        }
        const auto distance = vrp.problem.distance[static_cast<std::size_t>(prev_city + 1)][static_cast<std::size_t>(current_city + 1)];
        const auto arrival_time = current_time + req_list[static_cast<std::size_t>(prev_city + 1)].service_time + distance;
        begin_service = std::max(arrival_time, req_list[static_cast<std::size_t>(current_city + 1)].start_window);
        current_time = begin_service;
        ant.begin_service[static_cast<std::size_t>(current_city + 1)] = begin_service;
    }
    ant.current_time[static_cast<std::size_t>(index_tour_destination)] = begin_service;
}

AntSolution relocate_multiple_route(const AntSolution& ant,
                                    const InstanceData& instance,
                                    const AntAlgorithmState& ants) {
    auto best_improved = ant;
    auto temp = ant;
    const auto& req_list = instance.requests;
    const auto last_committed = committed_positions(ants);

    for (int index_tour_source = 0; index_tour_source < ant.used_vehicles; ++index_tour_source) {
        for (int index_tour_destination = 0; index_tour_destination < ant.used_vehicles; ++index_tour_destination) {
            if (index_tour_source == index_tour_destination) {
                continue;
            }
            const auto start_index_source =
                index_tour_source > static_cast<int>(last_committed.size()) - 1 ? 1 : last_committed[static_cast<std::size_t>(index_tour_source)] + 1;
            const auto start_index_destination =
                index_tour_destination > static_cast<int>(last_committed.size()) - 1 ? 1 : last_committed[static_cast<std::size_t>(index_tour_destination)] + 1;

            for (int i = start_index_source;
                 i < static_cast<int>(temp.tours[static_cast<std::size_t>(index_tour_source)].size()) - 1;
                 ++i) {
                for (int j = start_index_destination;
                     j < static_cast<int>(temp.tours[static_cast<std::size_t>(index_tour_destination)].size());
                     ++j) {
                    if (!check_feasible_tour_relocation_multiple(temp, instance, index_tour_source, index_tour_destination, i, j)) {
                        continue;
                    }

                    const auto city = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
                    temp.tours[static_cast<std::size_t>(index_tour_source)].erase(
                        temp.tours[static_cast<std::size_t>(index_tour_source)].begin() + i);
                    temp.tours[static_cast<std::size_t>(index_tour_destination)].insert(
                        temp.tours[static_cast<std::size_t>(index_tour_destination)].begin() + j,
                        city);

                    temp.current_quantity[static_cast<std::size_t>(index_tour_source)] -= req_list[static_cast<std::size_t>(city + 1)].demand;
                    temp.current_quantity[static_cast<std::size_t>(index_tour_destination)] += req_list[static_cast<std::size_t>(city + 1)].demand;

                    update_begin_service_relocation_multiple(temp, instance, index_tour_source, index_tour_destination, i, j);

                    const auto source_prev_city = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i - 1)];
                    const auto source_next_city = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
                    const auto new_distance1 =
                        temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] -
                        instance.problem.distance[static_cast<std::size_t>(source_prev_city + 1)][static_cast<std::size_t>(city + 1)] -
                        instance.problem.distance[static_cast<std::size_t>(city + 1)][static_cast<std::size_t>(source_next_city + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(source_prev_city + 1)][static_cast<std::size_t>(source_next_city + 1)];

                    const auto destination_prev_city = temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j - 1)];
                    const auto destination_next_city = temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j + 1)];
                    const auto new_distance2 =
                        temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] -
                        instance.problem.distance[static_cast<std::size_t>(destination_prev_city + 1)][static_cast<std::size_t>(destination_next_city + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(destination_prev_city + 1)][static_cast<std::size_t>(city + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(city + 1)][static_cast<std::size_t>(destination_next_city + 1)];

                    temp.total_tour_length = temp.total_tour_length -
                                             temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] -
                                             temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] +
                                             new_distance1 + new_distance2;
                    temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] = new_distance1;
                    temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] = new_distance2;

                    if (temp.tours[static_cast<std::size_t>(index_tour_source)].size() == 2 &&
                        temp.tours[static_cast<std::size_t>(index_tour_source)][0] == -1 &&
                        temp.tours[static_cast<std::size_t>(index_tour_source)][1] == -1) {
                        temp.tours.erase(temp.tours.begin() + index_tour_source);
                        temp.tour_lengths.erase(temp.tour_lengths.begin() + index_tour_source);
                        temp.current_quantity.erase(temp.current_quantity.begin() + index_tour_source);
                        temp.current_time.erase(temp.current_time.begin() + index_tour_source);
                        temp.used_vehicles--;
                    }

                    if ((temp.total_tour_length < best_improved.total_tour_length) ||
                        (temp.used_vehicles < best_improved.used_vehicles)) {
                        best_improved = temp;
                    }
                    temp = ant;
                }
            }
        }
    }
    return best_improved;
}

AntSolution relocate_multiple_route_iterated(const AntSolution& ant,
                                             const InstanceData& instance,
                                             const AntAlgorithmState& ants) {
    auto improved = ant;
    auto found_improvement = true;
    constexpr auto kTempNo = 1e10;
    while (found_improvement) {
        found_improvement = false;
        const auto candidate = relocate_multiple_route(improved, instance, ants);
        const auto round_candidate = std::round(candidate.total_tour_length * kTempNo) / kTempNo;
        const auto round_current = std::round(improved.total_tour_length * kTempNo) / kTempNo;
        if (((round_candidate < round_current) && (candidate.used_vehicles == improved.used_vehicles)) ||
            (candidate.used_vehicles < improved.used_vehicles)) {
            improved = candidate;
            found_improvement = true;
        }
    }
    return improved;
}

AntSolution exchange_multiple_route(const AntSolution& ant,
                                    const InstanceData& instance,
                                    const AntAlgorithmState& ants) {
    auto best_improved = ant;
    auto temp = ant;
    const auto& req_list = instance.requests;
    const auto last_committed = committed_positions(ants);

    for (int index_tour_source = 0; index_tour_source < ant.used_vehicles - 1; ++index_tour_source) {
        for (int index_tour_destination = index_tour_source + 1; index_tour_destination < ant.used_vehicles; ++index_tour_destination) {
            const auto start_index_source =
                index_tour_source > static_cast<int>(last_committed.size()) - 1 ? 1 : last_committed[static_cast<std::size_t>(index_tour_source)] + 1;
            const auto start_index_destination =
                index_tour_destination > static_cast<int>(last_committed.size()) - 1 ? 1 : last_committed[static_cast<std::size_t>(index_tour_destination)] + 1;

            for (int i = start_index_source;
                 i < static_cast<int>(temp.tours[static_cast<std::size_t>(index_tour_source)].size()) - 1;
                 ++i) {
                for (int j = start_index_destination;
                     j < static_cast<int>(temp.tours[static_cast<std::size_t>(index_tour_destination)].size()) - 1;
                     ++j) {
                    if (!check_feasible_tour_exchange_multiple(temp, instance, index_tour_source, index_tour_destination, i, j)) {
                        continue;
                    }
                    const auto city1 = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)];
                    const auto city2 = temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)];
                    temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i)] = city2;
                    temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j)] = city1;

                    temp.current_quantity[static_cast<std::size_t>(index_tour_source)] =
                        temp.current_quantity[static_cast<std::size_t>(index_tour_source)] -
                        req_list[static_cast<std::size_t>(city1 + 1)].demand +
                        req_list[static_cast<std::size_t>(city2 + 1)].demand;
                    temp.current_quantity[static_cast<std::size_t>(index_tour_destination)] =
                        temp.current_quantity[static_cast<std::size_t>(index_tour_destination)] -
                        req_list[static_cast<std::size_t>(city2 + 1)].demand +
                        req_list[static_cast<std::size_t>(city1 + 1)].demand;

                    update_begin_service_exchange_multiple(temp, instance, index_tour_source, index_tour_destination, i, j);

                    const auto source_prev_city = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i - 1)];
                    const auto source_next_city = temp.tours[static_cast<std::size_t>(index_tour_source)][static_cast<std::size_t>(i + 1)];
                    const auto new_distance1 =
                        temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] -
                        instance.problem.distance[static_cast<std::size_t>(source_prev_city + 1)][static_cast<std::size_t>(city1 + 1)] -
                        instance.problem.distance[static_cast<std::size_t>(city1 + 1)][static_cast<std::size_t>(source_next_city + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(source_prev_city + 1)][static_cast<std::size_t>(city2 + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(city2 + 1)][static_cast<std::size_t>(source_next_city + 1)];

                    const auto destination_prev_city = temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j - 1)];
                    const auto destination_next_city = temp.tours[static_cast<std::size_t>(index_tour_destination)][static_cast<std::size_t>(j + 1)];
                    const auto new_distance2 =
                        temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] -
                        instance.problem.distance[static_cast<std::size_t>(destination_prev_city + 1)][static_cast<std::size_t>(city2 + 1)] -
                        instance.problem.distance[static_cast<std::size_t>(city2 + 1)][static_cast<std::size_t>(destination_next_city + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(destination_prev_city + 1)][static_cast<std::size_t>(city1 + 1)] +
                        instance.problem.distance[static_cast<std::size_t>(city1 + 1)][static_cast<std::size_t>(destination_next_city + 1)];

                    temp.total_tour_length = temp.total_tour_length -
                                             temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] -
                                             temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] +
                                             new_distance1 + new_distance2;
                    temp.tour_lengths[static_cast<std::size_t>(index_tour_source)] = new_distance1;
                    temp.tour_lengths[static_cast<std::size_t>(index_tour_destination)] = new_distance2;

                    if (temp.total_tour_length < best_improved.total_tour_length) {
                        best_improved = temp;
                    }
                    temp = ant;
                }
            }
        }
    }
    return best_improved;
}

AntSolution exchange_multiple_route_iterated(const AntSolution& ant,
                                             const InstanceData& instance,
                                             const AntAlgorithmState& ants) {
    auto improved = ant;
    auto found_improvement = true;
    constexpr auto kTempNo = 1e10;
    while (found_improvement) {
        found_improvement = false;
        const auto candidate = exchange_multiple_route(improved, instance, ants);
        const auto round_candidate = std::round(candidate.total_tour_length * kTempNo) / kTempNo;
        const auto round_current = std::round(improved.total_tour_length * kTempNo) / kTempNo;
        if (round_candidate < round_current) {
            improved = candidate;
            found_improvement = true;
        }
    }
    return improved;
}

AntSolution local_search(const AntSolution& ant,
                         const InstanceData& instance,
                         const AntAlgorithmState& ants) {
    auto improved = relocate_multiple_route_iterated(ant, instance, ants);
    improved = exchange_multiple_route_iterated(improved, instance, ants);
    return improved;
}

void apply_local_search(std::vector<AntSolution>& population,
                        const InstanceData& instance,
                        const AntAlgorithmState& ants) {
    for (auto& ant : population) {
        ant = local_search(ant, instance, ants);
    }
}

void run_colony_iterations(const InstanceData& instance,
                           AntAlgorithmState& ants,
                           InOutState& inout,
                           const int max_iterations) {
    auto counter = 0;
    while (counter < max_iterations) {
        construct_solutions(instance, ants, inout);
        inout.no_evaluations++;
        apply_local_search(ants.ants, instance, ants);
        update_statistics(instance, ants, inout);
        pheromone_trail_update(ants, static_cast<int>(instance.customer_count()));
        ++inout.iteration;
        ++counter;
    }
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
