#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

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
        std::cout << "All data reader tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
