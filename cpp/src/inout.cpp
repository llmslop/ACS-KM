#include "acs_km/inout.hpp"

#include <numeric>
#include <stdexcept>

#include "acs_km/parse.hpp"

namespace acs_km {

void set_default_as_parameters(AntAlgorithmState& ants) {
    ants.n_ants = -1;
    ants.nn_ants = 20;
    ants.alpha = 1.0;
    ants.beta = 2.0;
    ants.rho = 0.5;
    ants.q_0 = 0.0;
}

void set_default_acs_parameters(AntAlgorithmState& ants) {
    ants.n_ants = 10;
    ants.nn_ants = 20;
    ants.alpha = 1.0;
    ants.beta = 1.0;
    ants.rho = 0.9;
    ants.local_rho = 0.9;
    ants.q_0 = 0.9;
}

void set_default_parameters(AntAlgorithmState& ants, InOutState& inout) {
    ants.n_ants = 25;
    ants.nn_ants = 20;
    ants.alpha = 1.0;
    ants.beta = 2.0;
    ants.rho = 0.5;
    ants.q_0 = 0.0;
    ants.u_gb = 2147483647;
    ants.acs_flag = false;
    ants.as_flag = false;
    ants.acs_km_flag = false;

    inout.max_tries = 10;
    inout.max_tours = 200;
    inout.max_time = 100.0;
    inout.max_iterations = 3000;
    inout.optimal = 1;
    inout.branch_fac = 1.00001;
    inout.distance_type = DistanceType::euc_2d;
    inout.pheromone_preservation = 0.3;
}

double node_branching(const AntAlgorithmState& ants, const Problem& problem, const double lambda) {
    if (problem.nn_list.empty() || ants.pheromone.empty()) {
        return 0.0;
    }

    const auto n = problem.nn_list.size();
    auto avg = 0.0;

    for (std::size_t m = 0; m < n; ++m) {
        if (problem.nn_list[m].empty()) {
            continue;
        }
        auto min_pheromone = ants.pheromone[m][static_cast<std::size_t>(problem.nn_list[m][0])];
        auto max_pheromone = min_pheromone;
        for (std::size_t i = 1; i < static_cast<std::size_t>(ants.nn_ants) && i < problem.nn_list[m].size(); ++i) {
            const auto value = ants.pheromone[m][static_cast<std::size_t>(problem.nn_list[m][i])];
            if (value > max_pheromone) {
                max_pheromone = value;
            }
            if (value < min_pheromone) {
                min_pheromone = value;
            }
        }
        const auto cutoff = min_pheromone + lambda * (max_pheromone - min_pheromone);
        auto branches = 0.0;
        for (std::size_t i = 0; i < static_cast<std::size_t>(ants.nn_ants) && i < problem.nn_list[m].size(); ++i) {
            if (ants.pheromone[m][static_cast<std::size_t>(problem.nn_list[m][i])] > cutoff) {
                branches += 1.0;
            }
        }
        avg += branches;
    }

    return avg / static_cast<double>(n * 2);
}

float average(const std::vector<int>& values) {
    if (values.empty()) {
        return 0.0F;
    }
    const auto sum = std::accumulate(values.begin(), values.end(), 0);
    return static_cast<float>(sum) / static_cast<float>(values.size());
}

float variance(const std::vector<int>& values) {
    if (values.empty()) {
        return 0.0F;
    }
    const auto mean = average(values);
    auto variance_value = 0.0;
    for (const auto value : values) {
        const auto delta = static_cast<double>(value) - static_cast<double>(mean);
        variance_value += delta * delta;
    }
    return static_cast<float>(variance_value / static_cast<double>(values.size()));
}

void init_program(const std::vector<std::string>& args,
                  const int run_number,
                  Problem& problem,
                  const double scaling_value,
                  AntAlgorithmState& ants,
                  InOutState& inout) {
    set_default_parameters(ants, inout);
    parse_commandline(args, run_number, ants, inout);

    if (ants.n_ants < 0) {
        ants.n_ants = static_cast<int>(problem.nodes.size()) - 1;
    }
    problem.distance = compute_distances(problem.nodes, inout.distance_type, scaling_value);
}

}  // namespace acs_km
