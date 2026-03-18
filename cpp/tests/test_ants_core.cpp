#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "acs_km/ants_core.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

acs_km::InstanceData make_instance() {
    acs_km::InstanceData data;
    data.problem.distance = {
        {0, 1, 2},
        {1, 0, 1},
        {2, 1, 0},
    };
    data.requests = {
        {.id = 0},
        {.id = 1},
        {.id = 2},
    };
    data.available_request_ids = {0, 1};
    return data;
}

void test_allocate_and_pheromone_ops() {
    auto data = make_instance();
    acs_km::AntAlgorithmState ants;
    ants.trail_0 = 0.5;
    ants.rho = 0.1;

    acs_km::allocate_ants(ants, data);
    expect(ants.committed_nodes.size() == 2, "Expected committed node array for customers");

    acs_km::init_pheromone_trails(ants, 2, 1.0);
    expect(ants.pheromone[0][1] == 1.0, "Expected initialized pheromone");

    acs_km::preserve_pheromones(ants, data, 0.3);
    expect(ants.pheromone[0][1] > 0.0, "Expected positive pheromone after preserve");

    const auto previous = ants.pheromone[0][1];
    acs_km::evaporation(ants, 2);
    expect(ants.pheromone[0][1] < previous, "Expected evaporation to lower pheromone");
}

void test_find_best() {
    std::vector<acs_km::AntSolution> pop(3);
    pop[0].used_vehicles = 2;
    pop[0].total_tour_length = 50.0;
    pop[1].used_vehicles = 1;
    pop[1].total_tour_length = 70.0;
    pop[2].used_vehicles = 1;
    pop[2].total_tour_length = 60.0;

    const auto best = acs_km::find_best(pop);
    expect(best == 2, "Expected best ant index with min vehicles then min distance");
}

}  // namespace

int main() {
    try {
        test_allocate_and_pheromone_ops();
        test_find_best();
        std::cout << "All ants core tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
