#pragma once

#include <cstddef>
#include <vector>

namespace acs_km {

struct AntSolution {
    std::vector<std::vector<int>> tours;
    std::vector<bool> visited;
    std::vector<double> tour_lengths;
    std::vector<double> begin_service;
    int used_vehicles{1};
    std::vector<double> current_time;
    std::vector<std::vector<double>> earliest_time;
    std::vector<std::vector<double>> latest_time;
    std::vector<double> current_quantity;
    double total_tour_length{0.0};
    double longest_tour_length{0.0};
    int index_longest_tour{-1};
    int to_visit{0};
    std::vector<double> cost_objectives;
    bool added_empty_tour{false};
};

struct AntAlgorithmState {
    int n_ants{-1};
    int nn_ants{20};
    double rho{0.5};
    double local_rho{0.9};
    double alpha{1.0};
    double beta{2.0};
    double q_0{0.0};
    bool as_flag{false};
    bool acs_flag{false};
    bool acs_km_flag{false};
    double initial_weight1{0.0};
    double initial_weight2{0.0};
    double initial_weight3{0.0};
    int u_gb{2147483647};
    double trail_0{0.0};

    std::vector<std::vector<double>> pheromone;
    std::vector<bool> committed_nodes;
    std::vector<AntSolution> ants;
    AntSolution best_so_far_ant;
};

}  // namespace acs_km
