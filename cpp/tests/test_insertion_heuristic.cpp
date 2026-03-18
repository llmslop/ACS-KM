#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "acs_km/insertion_heuristic.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

acs_km::InstanceData make_small_instance() {
    acs_km::InstanceData data;
    data.problem.capacity = 10;
    data.problem.distance = {
        {0, 1, 2},
        {1, 0, 1},
        {2, 1, 0},
    };
    data.requests = {
        {.id = 0, .demand = 0, .start_window = 0, .end_window = 100, .service_time = 0, .available_time = 0},
        {.id = 1, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 0, .available_time = 0},
        {.id = 2, .demand = 3, .start_window = 0, .end_window = 100, .service_time = 0, .available_time = 0},
    };
    return data;
}

void test_feasible_insertion_and_insert() {
    auto data = make_small_instance();
    acs_km::AntSolution ant;
    ant.used_vehicles = 1;
    ant.tours = {{-1, -1}};
    ant.visited = {false, false};
    ant.to_visit = 2;
    ant.current_quantity = {0.0};
    ant.current_time = {0.0};
    ant.begin_service = {0.0, 0.0, 0.0};
    ant.earliest_time = {{}};
    ant.latest_time = {{}};

    acs_km::compute_route_variables(ant, data, 0);
    expect(acs_km::is_feasible_insertion(ant, data, 0, 0, 0, 1), "Expected customer 0 feasible insertion");

    acs_km::insert_unrouted_customers(ant, data, {0, 1}, 0, {0});

    expect(ant.to_visit == 0, "Expected all customers inserted");
    expect(ant.visited[0] && ant.visited[1], "Expected both customers marked visited");
    expect(ant.tours[0].size() == 4, "Expected depot + 2 customers + depot");
}

}  // namespace

int main() {
    try {
        test_feasible_insertion_and_insert();
        std::cout << "All insertion heuristic tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
