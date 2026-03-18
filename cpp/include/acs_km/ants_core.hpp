#pragma once

#include "acs_km/ants_state.hpp"
#include "acs_km/problem.hpp"

namespace acs_km {

[[nodiscard]] double heuristic(const Problem& problem, int m, int n);

void allocate_ants(AntAlgorithmState& ants, const InstanceData& instance);
[[nodiscard]] int find_best(const std::vector<AntSolution>& population);
void ant_empty_memory(AntSolution& ant, const InstanceData& instance);
void copy_from_to(const AntSolution& source, AntSolution& destination, const InstanceData& instance);
[[nodiscard]] std::vector<int> unrouted_customers(const AntSolution& ant, const InstanceData& vrp);
[[nodiscard]] double compute_tours_amplitude(const AntSolution& ant);
void choose_closest_next(AntSolution& ant, const InstanceData& instance, int index_salesman);
void global_update_pheromone(AntAlgorithmState& ants, const AntSolution& ant);
void global_acs_pheromone_update(AntAlgorithmState& ants, const AntSolution& ant);
void local_acs_pheromone_update(AntAlgorithmState& ants, const AntSolution& ant, int index_salesman);
[[nodiscard]] double nn_tour(AntAlgorithmState& ants, const InstanceData& instance);

void init_pheromone_trails(AntAlgorithmState& ants, int customer_count, double initial_trail);
void preserve_pheromones(AntAlgorithmState& ants,
                         const InstanceData& vrp,
                         double pheromone_preservation);
void evaporation(AntAlgorithmState& ants, int customer_count);

}  // namespace acs_km
