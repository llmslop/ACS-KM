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

void test_ant_memory_copy_and_lists() {
    auto data = make_instance();
    data.problem.capacity = 10;
    data.requests = {
        {.id = 0, .demand = 0, .start_window = 0, .end_window = 100, .service_time = 0},
        {.id = 1, .demand = 2, .start_window = 0, .end_window = 100, .service_time = 1},
        {.id = 2, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 1},
    };
    acs_km::AntAlgorithmState ants;
    ants.ants = {acs_km::AntSolution{}};

    acs_km::ant_empty_memory(ants.ants[0], data);
    expect(ants.ants[0].used_vehicles == 1, "Expected one vehicle after empty memory");
    expect(ants.ants[0].to_visit == 2, "Expected to_visit from available request ids");

    ants.ants[0].tours[0] = {-1, 0, -1};
    ants.ants[0].visited = {true, false};
    ants.ants[0].tour_lengths[0] = 10.0;
    ants.ants[0].total_tour_length = 10.0;
    ants.ants[0].to_visit = 1;
    ants.ants[0].begin_service = {0.0, 1.0, 2.0};
    ants.ants[0].current_time[0] = 2.0;
    ants.ants[0].current_quantity[0] = 2.0;

    acs_km::AntSolution dst;
    acs_km::copy_from_to(ants.ants[0], dst, data);
    expect(dst.tours[0].size() == 3, "Expected copied tour size");
    expect(dst.visited[0], "Expected visited copied");

    const auto unrouted = acs_km::unrouted_customers(dst, data);
    expect(unrouted.size() == 1 && unrouted[0] == 1, "Expected unrouted customer list");
    expect(acs_km::compute_tours_amplitude(dst) == 0.0, "Expected zero amplitude with one tour");
}

void test_pheromone_updates_and_nn_tour() {
    auto data = make_instance();
    data.problem.capacity = 10;
    data.problem.distance = {
        {0, 2, 3},
        {2, 0, 1},
        {3, 1, 0},
    };
    data.requests = {
        {.id = 0, .demand = 0, .start_window = 0, .end_window = 100, .service_time = 0},
        {.id = 1, .demand = 2, .start_window = 0, .end_window = 100, .service_time = 1},
        {.id = 2, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 1},
    };

    acs_km::AntAlgorithmState ants;
    ants.rho = 0.1;
    ants.local_rho = 0.2;
    ants.trail_0 = 0.5;
    ants.ants = {acs_km::AntSolution{}};
    acs_km::init_pheromone_trails(ants, 2, 1.0);

    acs_km::AntSolution ant;
    ant.used_vehicles = 1;
    ant.tours = {{-1, 0, -1}};
    ant.total_tour_length = 4.0;
    acs_km::global_update_pheromone(ants, ant);
    expect(ants.pheromone[0][1] > 1.0, "Expected pheromone increase on used arc");

    const auto after_global = ants.pheromone[0][1];
    acs_km::global_acs_pheromone_update(ants, ant);
    expect(ants.pheromone[0][1] != after_global, "Expected ACS global update change");

    acs_km::local_acs_pheromone_update(ants, ant, 0);
    expect(ants.pheromone[1][0] == ants.pheromone[0][1], "Expected symmetric pheromone matrix");

    const auto nn = acs_km::nn_tour(ants, data);
    expect(nn > 0.0, "Expected positive nearest-neighbor total length");
}

}  // namespace

int main() {
    try {
        test_allocate_and_pheromone_ops();
        test_find_best();
        test_ant_memory_copy_and_lists();
        test_pheromone_updates_and_nn_tour();
        std::cout << "All ants core tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
