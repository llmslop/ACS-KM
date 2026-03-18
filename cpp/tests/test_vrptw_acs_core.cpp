#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "acs_km/vrptw_acs_core.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

acs_km::InstanceData make_instance() {
    acs_km::InstanceData data;
    data.problem.capacity = 10;
    data.problem.distance = {
        {0, 2, 3},
        {2, 0, 1},
        {3, 1, 0},
    };
    data.requests = {
        {.id = 0, .demand = 0, .start_window = 0, .end_window = 100, .service_time = 0},
        {.id = 1, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 1},
        {.id = 2, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 1},
    };
    return data;
}

void test_termination_and_weights() {
    expect(!acs_km::termination_condition(4.0, 5.0), "Expected no termination yet");
    expect(acs_km::termination_condition(5.0, 5.0), "Expected termination at max time");

    acs_km::AntAlgorithmState ants;
    acs_km::UtilitiesCore random(7U);
    acs_km::generate_initial_weights(ants, random);
    expect(ants.initial_weight1 > 0.0, "Expected first initial weight positive");
    expect(ants.initial_weight2 >= 0.0, "Expected second initial weight non-negative");
    expect(ants.initial_weight3 >= 0.0, "Expected third initial weight non-negative");
    expect((ants.initial_weight1 + ants.initial_weight2 + ants.initial_weight3) > 0.999999 &&
               (ants.initial_weight1 + ants.initial_weight2 + ants.initial_weight3) < 1.000001,
           "Expected weights to sum to 1");
}

void test_is_done_and_feasible() {
    acs_km::AntAlgorithmState ants;
    ants.ants = {acs_km::AntSolution{.to_visit = 0}, acs_km::AntSolution{.to_visit = 1}};
    expect(!acs_km::is_done(ants), "Expected colony not done with pending visits");
    ants.ants[1].to_visit = 0;
    expect(acs_km::is_done(ants), "Expected colony done");

    auto instance = make_instance();
    acs_km::AntSolution ant;
    ant.current_quantity = {2.0};
    expect(acs_km::is_feasible(instance, ant, 0, 10.0, 15.0, 0), "Expected feasible insertion");
    expect(!acs_km::is_feasible(instance, ant, 0, 200.0, 15.0, 0), "Expected infeasible by end-window");
}

void test_committed_tours_and_add_nodes() {
    auto instance = make_instance();
    acs_km::AntAlgorithmState ants;
    ants.committed_nodes = {true, false};
    ants.best_so_far_ant.used_vehicles = 1;
    ants.best_so_far_ant.tours = {{-1, 0, 1, -1}};

    expect(acs_km::check_committed_tours(ants), "Expected committed tours to exist");

    acs_km::AntSolution ant;
    ant.used_vehicles = 1;
    ant.tours = {{-1}};
    ant.tour_lengths = {0.0};
    ant.current_quantity = {0.0};
    ant.current_time = {0.0};
    ant.visited = {false, false};
    ant.to_visit = 2;
    ant.begin_service = {0.0, 0.0, 0.0};

    acs_km::add_committed_nodes(ant, ants, instance);
    expect(ant.tours[0].size() == 2, "Expected one committed node inserted");
    expect(ant.tours[0][1] == 0, "Expected committed node id inserted");
    expect(ant.visited[0], "Expected inserted committed node marked visited");
}

void test_shortest_tour_and_calc_dist_and_compare() {
    auto instance = make_instance();

    acs_km::AntSolution ant;
    ant.used_vehicles = 2;
    ant.tours = {{-1, 0, 1, -1}, {-1, 1, -1}};
    expect(acs_km::find_shortest_tour(ant) == 1, "Expected second tour shortest");

    const auto score = acs_km::calc_tour_dist({-1, 0, -1}, instance);
    expect(score < 0, "Expected negative finite score for feasible tour");

    auto tight = instance;
    tight.requests[1].end_window = 0.0;
    expect(acs_km::calc_tour_dist({-1, 0, -1}, tight) == -100000000, "Expected hard penalty for infeasible tour");

    acs_km::AntSolution incumbent;
    incumbent.used_vehicles = 2;
    incumbent.total_tour_length = 100.0;
    acs_km::AntSolution candidate;
    candidate.used_vehicles = 1;
    candidate.total_tour_length = 120.0;
    expect(acs_km::is_better_solution(candidate, incumbent), "Expected fewer vehicles to dominate");
}

}  // namespace

int main() {
    try {
        test_termination_and_weights();
        test_is_done_and_feasible();
        test_committed_tours_and_add_nodes();
        test_shortest_tour_and_calc_dist_and_compare();
        std::cout << "All vrptw_acs core tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
