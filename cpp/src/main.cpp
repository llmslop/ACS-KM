#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "acs_km/data_reader.hpp"
#include "acs_km/execution_flow.hpp"
#include "acs_km/timer.hpp"

namespace {

std::filesystem::path default_input_path() {
    return std::filesystem::path("input") / "rc203-0.5.txt";
}

struct CliOptions {
    std::filesystem::path input_path{default_input_path()};
    bool show_help{false};
    bool simulate_slices{false};
    double working_day_seconds{100.0};
    int time_slices{50};
};

std::string usage_message() {
    return "Load an ACS-KM instance and optionally simulate dynamic request release.\n"
           "Default input: " + default_input_path().string() + "\n"
           "Usage: acs_km_cli [--input /absolute/or/relative/path/to/file.txt] [--simulate-slices] "
           "[--working-day positive_seconds] [--time-slices positive_count] [--help|-h]";
}

double parse_positive_double(const std::string& value, const std::string& option_name) {
    try {
        std::size_t parsed_chars = 0;
        const auto parsed = std::stod(value, &parsed_chars);
        if (parsed_chars != value.size() || parsed <= 0.0) {
            throw std::runtime_error(option_name + " must be greater than 0");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error(option_name + " must be greater than 0");
    }
}

int parse_positive_int(const std::string& value, const std::string& option_name) {
    try {
        std::size_t parsed_chars = 0;
        const auto parsed = std::stoi(value, &parsed_chars);
        if (parsed_chars != value.size() || parsed <= 0) {
            throw std::runtime_error(option_name + " must be greater than 0");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error(option_name + " must be greater than 0");
    }
}

CliOptions parse_options(int argc, char** argv) {
    CliOptions options;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
            continue;
        }
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
        if (arg == "--working-day") {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for --working-day");
            }
            options.working_day_seconds = parse_positive_double(argv[++index], "--working-day");
            continue;
        }
        if (arg == "--time-slices") {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for --time-slices");
            }
            options.time_slices = parse_positive_int(argv[++index], "--time-slices");
            continue;
        }

        throw std::runtime_error("Unrecognized option: " + arg + "\n" + usage_message());
    }

    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        acs_km::Timer timer;
        timer.start();

        const auto options = parse_options(argc, argv);
        if (options.show_help) {
            std::cout << usage_message() << '\n';
            return 0;
        }
        const acs_km::DataReader reader(options.input_path);
        const auto data = reader.read();

        std::cout << "Loaded instance: " << data.problem.name << '\n';
        std::cout << "Vehicles: " << data.problem.vehicle_count << " capacity: " << data.problem.capacity << '\n';
        std::cout << "Total requests (including depot): " << data.requests.size() << '\n';
        std::cout << "Customer count: " << data.customer_count() << '\n';
        std::cout << "Dynamic requests: " << data.dynamic_requests.size() << '\n';
        std::cout << "A-priori available requests: " << data.available_request_ids.size() << '\n';
        if (options.simulate_slices) {
            const auto simulation = acs_km::simulate_dynamic_release(
                data,
                acs_km::SimulationConfig{
                    .working_day_seconds = options.working_day_seconds,
                    .time_slices = options.time_slices,
                });
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
