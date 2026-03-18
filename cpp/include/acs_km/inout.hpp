#pragma once

#include <string>
#include <vector>

#include "acs_km/ants_state.hpp"
#include "acs_km/problem.hpp"
#include "acs_km/vrptw_utils.hpp"

namespace acs_km {

struct InOutState {
    DistanceType distance_type{DistanceType::euc_2d};
    int max_tries{10};
    int max_tours{200};
    int max_iterations{3000};
    double max_time{100.0};
    int optimal{1};
    double branch_fac{1.00001};
    double pheromone_preservation{0.3};
    int no_evaluations{0};
    int no_solutions{0};
    int found_best{0};
    int restart_found_best{0};
    int n_tours{0};
    int iteration{0};
    double time_used{0.0};
    double time_passed{0.0};
    double lambda{0.05};
};

void set_default_as_parameters(AntAlgorithmState& ants);
void set_default_acs_parameters(AntAlgorithmState& ants);
void set_default_parameters(AntAlgorithmState& ants, InOutState& inout);

[[nodiscard]] double node_branching(const AntAlgorithmState& ants,
                                    const Problem& problem,
                                    double lambda);

[[nodiscard]] float average(const std::vector<int>& values);
[[nodiscard]] float variance(const std::vector<int>& values);

void init_program(const std::vector<std::string>& args,
                  int run_number,
                  Problem& problem,
                  double scaling_value,
                  AntAlgorithmState& ants,
                  InOutState& inout);

}  // namespace acs_km
