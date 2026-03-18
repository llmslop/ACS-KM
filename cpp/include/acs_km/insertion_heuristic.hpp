#pragma once

#include <vector>

#include "acs_km/ants_state.hpp"
#include "acs_km/problem.hpp"

namespace acs_km {

struct Insertion {
    int customer{};
    int index_tour{};
    int previous_node{};
    double score{};
};

void compute_route_variables(AntSolution& ant, const InstanceData& vrp, int start_tour_index);
void update_route_variables(AntSolution& ant, const InstanceData& vrp, const Insertion& insertion);

[[nodiscard]] bool check_is_feasible_tour(const AntSolution& ant,
                                          const InstanceData& vrp,
                                          int customer,
                                          int index_tour,
                                          int previous_pos,
                                          int next_pos);

[[nodiscard]] bool is_feasible_insertion(const AntSolution& ant,
                                         const InstanceData& vrp,
                                         int customer,
                                         int index_tour,
                                         int previous_pos,
                                         int next_pos);

void insert_unrouted_customers(AntSolution& ant,
                               const InstanceData& vrp,
                               const std::vector<int>& unvisited_nodes,
                               int start_index_tour,
                               const std::vector<int>& start_pos);

}  // namespace acs_km
