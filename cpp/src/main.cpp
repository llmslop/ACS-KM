#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "acs_km/data_reader.hpp"
#include "acs_km/timer.hpp"

namespace {

std::filesystem::path resolve_input_path(int argc, char** argv) {
    if (argc == 1) {
        return std::filesystem::path("input") / "rc203-0.5.txt";
    }

    if (argc == 3 && std::string(argv[1]) == "--input") {
        return std::filesystem::path(argv[2]);
    }

    throw std::runtime_error("Usage: acs_km_cli [--input /absolute/or/relative/path/to/file.txt]");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        acs_km::Timer timer;
        timer.start();

        const auto input_path = resolve_input_path(argc, argv);
        const acs_km::DataReader reader(input_path);
        const auto data = reader.read();

        std::cout << "Loaded instance: " << data.problem.name << '\n';
        std::cout << "Vehicles: " << data.problem.vehicle_count << " capacity: " << data.problem.capacity << '\n';
        std::cout << "Total requests (including depot): " << data.requests.size() << '\n';
        std::cout << "Customer count: " << data.customer_count() << '\n';
        std::cout << "Dynamic requests: " << data.dynamic_requests.size() << '\n';
        std::cout << "A-priori available requests: " << data.available_request_ids.size() << '\n';
        std::cout << "Elapsed seconds: " << timer.elapsed_seconds() << '\n';

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
