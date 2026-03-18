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

void test_relocation_and_exchange_helpers() {
    auto instance = make_instance();
    acs_km::AntSolution ant;
    ant.tours = {{-1, 0, -1}, {-1, 1, -1}};
    ant.used_vehicles = 2;
    ant.current_quantity = {3.0, 3.0};
    ant.current_time = {0.0, 0.0};
    ant.begin_service = {0.0, 2.0, 3.0};

    expect(acs_km::check_feasible_tour_relocation_multiple(ant, instance, 0, 1, 1, 1),
           "Expected relocation feasibility");
    expect(acs_km::check_feasible_tour_exchange_multiple(ant, instance, 0, 1, 1, 1),
           "Expected exchange feasibility");

    acs_km::update_begin_service_relocation_multiple(ant, instance, 0, 1, 1, 1);
    expect(ant.current_time[0] >= 0.0 && ant.current_time[1] >= 0.0, "Expected relocation begin-service update");

    acs_km::update_begin_service_exchange_multiple(ant, instance, 0, 1, 1, 1);
    expect(ant.current_time[0] >= 0.0 && ant.current_time[1] >= 0.0, "Expected exchange begin-service update");
}

void test_pheromone_trail_update_orchestration() {
    acs_km::AntAlgorithmState ants;
    ants.rho = 0.1;
    ants.trail_0 = 1.0;
    ants.as_flag = true;
    acs_km::AntSolution one_ant;
    one_ant.used_vehicles = 1;
    one_ant.tours = {{-1, 0, -1}};
    one_ant.total_tour_length = 4.0;
    ants.ants = {one_ant};
    acs_km::init_pheromone_trails(ants, 1, 1.0);
    acs_km::pheromone_trail_update(ants, 1);
    expect(ants.pheromone[0][1] > 0.0, "Expected AS trail update to keep positive pheromone");

    ants.as_flag = false;
    ants.acs_flag = true;
    ants.best_so_far_ant = ants.ants[0];
    const auto before = ants.pheromone[0][1];
    acs_km::pheromone_trail_update(ants, 1);
    expect(ants.pheromone[0][1] != before, "Expected ACS trail update to modify pheromone");
}

void test_init_try_and_update_statistics() {
    auto instance = make_instance();
    instance.available_request_ids = {0, 1};

    acs_km::AntAlgorithmState ants;
    ants.ants = {acs_km::AntSolution{}, acs_km::AntSolution{}};
    ants.rho = 0.5;
    ants.acs_flag = false;
    acs_km::InOutState inout;

    acs_km::init_try(instance, ants, inout);
    expect(inout.iteration == 1, "Expected init_try to set iteration");
    expect(ants.trail_0 > 0.0, "Expected positive initial trail");
    expect(!ants.pheromone.empty(), "Expected initialized pheromone matrix");

    ants.ants[0].used_vehicles = 2;
    ants.ants[0].total_tour_length = 100.0;
    ants.ants[0].cost_objectives = {100.0, 0.0};
    ants.ants[1].used_vehicles = 0;
    ants.ants[1].total_tour_length = 90.0;
    ants.ants[1].cost_objectives = {90.0, 0.0};
    ants.ants[1].tours = {{-1, 0, -1}};
    ants.ants[1].tour_lengths = {90.0};
    ants.ants[1].current_quantity = {0.0};
    ants.ants[1].current_time = {0.0};
    ants.ants[1].visited = {true, false};
    ants.ants[1].begin_service = {0.0, 1.0, 2.0};
    ants.ants[1].to_visit = 1;
    inout.iteration = 3;

    acs_km::update_statistics(instance, ants, inout);
    expect(ants.best_so_far_ant.total_tour_length == 90.0, "Expected best-so-far updated from better ant");
    expect(inout.found_best == 3, "Expected found_best recorded from current iteration");
}

void test_construct_solutions() {
    auto instance = make_instance();
    instance.available_request_ids = {0, 1};
    acs_km::AntAlgorithmState ants;
    ants.n_ants = 1;
    ants.ants = {acs_km::AntSolution{}};
    ants.acs_flag = false;
    ants.best_so_far_ant.used_vehicles = 1;
    ants.best_so_far_ant.tours = {{-1, -1}};
    ants.committed_nodes = {false, false};
    acs_km::InOutState inout;

    acs_km::construct_solutions(instance, ants, inout);
    expect(ants.ants[0].to_visit == 0, "Expected constructed solution to visit all available nodes");
    expect(ants.ants[0].total_tour_length > 0.0, "Expected positive total length for constructed solution");
}

}  // namespace

int main() {
    try {
        test_termination_and_weights();
        test_is_done_and_feasible();
        test_committed_tours_and_add_nodes();
        test_shortest_tour_and_calc_dist_and_compare();
        test_relocation_and_exchange_helpers();
        test_pheromone_trail_update_orchestration();
        test_init_try_and_update_statistics();
        test_construct_solutions();
        std::cout << "All vrptw_acs core tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
