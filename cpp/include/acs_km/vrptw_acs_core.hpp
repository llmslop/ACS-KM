#pragma once

#include "acs_km/ants_state.hpp"
#include "acs_km/ants_core.hpp"
#include "acs_km/inout.hpp"
#include "acs_km/problem.hpp"
#include "acs_km/utilities_core.hpp"

namespace acs_km {

[[nodiscard]] bool termination_condition(double elapsed_seconds, double max_time_seconds);

[[nodiscard]] bool is_feasible(const InstanceData& vrp,
                               const AntSolution& ant,
                               int city,
                               double begin_service,
                               double begin_service_depot,
                               int index_salesman);

void generate_initial_weights(AntAlgorithmState& ants, UtilitiesCore& random_source);

[[nodiscard]] bool is_done(const AntAlgorithmState& ants);
[[nodiscard]] bool check_committed_tours(const AntAlgorithmState& ants);
[[nodiscard]] int find_shortest_tour(const AntSolution& ant);
[[nodiscard]] int calc_tour_dist(const std::vector<int>& tour, const InstanceData& vrp);
[[nodiscard]] bool is_better_solution(const AntSolution& candidate, const AntSolution& incumbent);
void as_update(AntAlgorithmState& ants);
void acs_global_update(AntAlgorithmState& ants);
void pheromone_trail_update(AntAlgorithmState& ants, int customer_count);
void init_try(const InstanceData& instance, AntAlgorithmState& ants, InOutState& inout);
void update_statistics(const InstanceData& instance, AntAlgorithmState& ants, InOutState& inout);
[[nodiscard]] bool check_feasible_tour_relocation_multiple(const AntSolution& ant,
                                                           const InstanceData& vrp,
                                                           int index_tour_source,
                                                           int index_tour_destination,
                                                           int i,
                                                           int j);
void update_begin_service_relocation_multiple(AntSolution& ant,
                                              const InstanceData& vrp,
                                              int index_tour_source,
                                              int index_tour_destination,
                                              int i,
                                              int j);
[[nodiscard]] bool check_feasible_tour_exchange_multiple(const AntSolution& ant,
                                                         const InstanceData& vrp,
                                                         int index_tour_source,
                                                         int index_tour_destination,
                                                         int i,
                                                         int j);
void update_begin_service_exchange_multiple(AntSolution& ant,
                                            const InstanceData& vrp,
                                            int index_tour_source,
                                            int index_tour_destination,
                                            int i,
                                            int j);

void add_committed_nodes(AntSolution& ant,
                         const AntAlgorithmState& ants,
                         const InstanceData& instance);

}  // namespace acs_km
