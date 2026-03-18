#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "acs_km/data_reader.hpp"
#include "acs_km/execution_flow.hpp"
#include "acs_km/timer.hpp"

namespace {

struct CliOptions {
    std::filesystem::path input_path{std::filesystem::path("input") / "rc203-0.5.txt"};
    bool simulate_slices{false};
};

CliOptions parse_options(int argc, char** argv) {
    CliOptions options;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--input") {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for --input");
            }
            options.input_path = std::filesystem::path(argv[++index]);
            continue;
        }
        if (arg == "--simulate-slices") {
            options.simulate_slices = true;
            continue;
        }

        throw std::runtime_error(
            "Usage: acs_km_cli [--input /absolute/or/relative/path/to/file.txt] [--simulate-slices]");
    }

    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        acs_km::Timer timer;
        timer.start();

        const auto options = parse_options(argc, argv);
        const acs_km::DataReader reader(options.input_path);
        const auto data = reader.read();

        std::cout << "Loaded instance: " << data.problem.name << '\n';
        std::cout << "Vehicles: " << data.problem.vehicle_count << " capacity: " << data.problem.capacity << '\n';
        std::cout << "Total requests (including depot): " << data.requests.size() << '\n';
        std::cout << "Customer count: " << data.customer_count() << '\n';
        std::cout << "Dynamic requests: " << data.dynamic_requests.size() << '\n';
        std::cout << "A-priori available requests: " << data.available_request_ids.size() << '\n';
        if (options.simulate_slices) {
            const auto simulation = acs_km::simulate_dynamic_release(data);
            std::cout << "Scaling value: " << simulation.scaling_value << '\n';
            std::cout << "Time slice length: " << simulation.slice_length_seconds << '\n';
            std::cout << "Slices with new nodes: " << simulation.events.size() << '\n';
            std::cout << "Total newly available nodes observed: " << simulation.total_newly_available << '\n';
        }

        std::cout << "Elapsed seconds: " << timer.elapsed_seconds() << '\n';

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
