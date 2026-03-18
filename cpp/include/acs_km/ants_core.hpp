#pragma once

#include "acs_km/ants_state.hpp"
#include "acs_km/problem.hpp"

namespace acs_km {

[[nodiscard]] double heuristic(const Problem& problem, int m, int n);

void allocate_ants(AntAlgorithmState& ants, const InstanceData& instance);
[[nodiscard]] int find_best(const std::vector<AntSolution>& population);

void init_pheromone_trails(AntAlgorithmState& ants, int customer_count, double initial_trail);
void preserve_pheromones(AntAlgorithmState& ants,
                         const InstanceData& vrp,
                         double pheromone_preservation);
void evaporation(AntAlgorithmState& ants, int customer_count);

}  // namespace acs_km
