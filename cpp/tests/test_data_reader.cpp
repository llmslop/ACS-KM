#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "acs_km/controller_utils.hpp"
#include "acs_km/data_reader.hpp"

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_static_instance() {
    const std::filesystem::path input = std::filesystem::path(ACS_KM_SOURCE_DIR) / "input" / "r101-0.0.txt";
    const acs_km::DataReader reader(input);
    const auto data = reader.read();

    expect(data.problem.vehicle_count == 25, "Expected 25 vehicles");
    expect(data.problem.capacity == 200, "Expected vehicle capacity 200");
    expect(!data.requests.empty(), "Expected parsed requests");
    expect(data.requests.front().id == 0, "Expected depot request id 0");
    expect(data.requests.front().x_coord == 35 && data.requests.front().y_coord == 35,
           "Unexpected depot coordinates");
    expect(data.dynamic_requests.empty(), "Expected no dynamic requests for dynamic level 0.0");
    expect(data.available_request_ids.size() + 1 == data.requests.size(),
           "Expected all non-depot requests available from start");
}


void test_collect_newly_available_nodes() {
    const std::filesystem::path input = std::filesystem::path(ACS_KM_SOURCE_DIR) / "input" / "r101-1.0.txt";
    const acs_km::DataReader reader(input);
    const auto data = reader.read();

    std::size_t cursor = 0;
    const auto early_nodes = acs_km::collect_newly_available_nodes(data.dynamic_requests, 10.0, cursor);
    const auto later_nodes = acs_km::collect_newly_available_nodes(data.dynamic_requests, 120.0, cursor);

    expect(!early_nodes.empty(), "Expected at least one node to become available by t=10");
    expect(!later_nodes.empty(), "Expected additional nodes to become available by t=120");

    const auto all_remaining_nodes = acs_km::collect_newly_available_nodes(data.dynamic_requests, 1e9, cursor);
    const auto total_returned = early_nodes.size() + later_nodes.size() + all_remaining_nodes.size();
    expect(total_returned == data.dynamic_requests.size(),
           "Expected utility to emit each dynamic request exactly once");
}

void test_dynamic_instance() {
    const std::filesystem::path input = std::filesystem::path(ACS_KM_SOURCE_DIR) / "input" / "r101-1.0.txt";
    const acs_km::DataReader reader(input);
    const auto data = reader.read();

    expect(data.problem.vehicle_count == 25, "Expected 25 vehicles");
    expect(data.problem.capacity == 200, "Expected vehicle capacity 200");
    expect(!data.dynamic_requests.empty(), "Expected dynamic requests for dynamic level 1.0");
    expect(!data.available_request_ids.empty(), "Expected at least some a-priori requests");
    expect(data.dynamic_requests.size() + data.available_request_ids.size() + 1 == data.requests.size(),
           "Expected dynamic + available + depot partition to match total requests");
}

}  // namespace

int main() {
    try {
        test_static_instance();
        test_dynamic_instance();
        test_collect_newly_available_nodes();
        std::cout << "All data reader tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
