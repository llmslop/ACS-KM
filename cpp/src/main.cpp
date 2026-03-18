#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "acs_km/ants_core.hpp"
#include "acs_km/data_reader.hpp"
#include "acs_km/execution_flow.hpp"
#include "acs_km/inout.hpp"
#include "acs_km/insertion_heuristic.hpp"
#include "acs_km/timer.hpp"
#include "acs_km/vrptw_acs_core.hpp"
#include "acs_km/vrptw_utils.hpp"

namespace {

std::filesystem::path default_input_path() {
    return std::filesystem::path("input") / "rc203-0.5.txt";
}

struct CliOptions {
    std::filesystem::path input_path{default_input_path()};
    bool show_help{false};
    bool simulate_slices{false};
    bool solve_dynamic{false};
    double working_day_seconds{100.0};
    int time_slices{50};
    int max_iterations_per_slice{300};
};

std::string usage_message() {
    return "Load an ACS-KM instance and optionally simulate/solve dynamic request release.\n"
           "Default input: " + default_input_path().string() + "\n"
           "Usage: acs_km_cli [--input /absolute/or/relative/path/to/file.txt] [--simulate-slices] [--solve-dynamic] "
           "[--max-iterations-per-slice positive_count] [--working-day positive_seconds] [--time-slices positive_count] [--help|-h]";
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
        if (arg == "--solve-dynamic") {
            options.solve_dynamic = true;
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
        if (arg == "--max-iterations-per-slice") {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for --max-iterations-per-slice");
            }
            options.max_iterations_per_slice = parse_positive_int(argv[++index], "--max-iterations-per-slice");
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
        auto data = reader.read();

        std::cout << "Loaded instance: " << data.problem.name << '\n';
        std::cout << "Vehicles: " << data.problem.vehicle_count << " capacity: " << data.problem.capacity << '\n';
        std::cout << "Total requests (including depot): " << data.requests.size() << '\n';
        std::cout << "Customer count: " << data.customer_count() << '\n';
        std::cout << "Dynamic requests: " << data.dynamic_requests.size() << '\n';
        std::cout << "A-priori available requests: " << data.available_request_ids.size() << '\n';

        const auto simulation = acs_km::simulate_dynamic_release(
            data,
            acs_km::SimulationConfig{
                .working_day_seconds = options.working_day_seconds,
                .time_slices = options.time_slices,
            });
        if (options.simulate_slices || options.solve_dynamic) {
            std::cout << "Scaling value: " << simulation.scaling_value << '\n';
            std::cout << "Time slice length: " << simulation.slice_length_seconds << '\n';
            std::cout << "Slices with new nodes: " << simulation.events.size() << '\n';
            std::cout << "Total newly available nodes observed: " << simulation.total_newly_available << '\n';
        }

        if (options.solve_dynamic) {
            for (auto& req : data.requests) {
                req.start_window *= simulation.scaling_value;
                req.end_window *= simulation.scaling_value;
                req.service_time *= simulation.scaling_value;
                req.available_time *= simulation.scaling_value;
            }
            for (auto& req : data.dynamic_requests) {
                req.available_time *= simulation.scaling_value;
            }

            data.problem.distance = acs_km::compute_distances(data.problem.nodes, acs_km::DistanceType::euc_2d, simulation.scaling_value);
            const auto nn = acs_km::compute_nn_lists(data.problem.distance, 20);
            data.problem.nn_list = nn.excluding_depot;
            data.problem.nn_list_all = nn.including_depot;

            acs_km::AntAlgorithmState ants;
            acs_km::InOutState inout;
            acs_km::set_default_acs_parameters(ants);
            ants.acs_flag = true;
            ants.n_ants = 1;
            ants.ants.assign(static_cast<std::size_t>(ants.n_ants), acs_km::AntSolution{});
            acs_km::allocate_ants(ants, data);
            acs_km::UtilitiesCore random_source;
            acs_km::generate_initial_weights(ants, random_source);
            acs_km::init_try(data, ants, inout);

            std::unordered_map<int, std::vector<int>> events_by_slice;
            for (const auto& event : simulation.events) {
                events_by_slice[event.slice_index] = event.newly_available_node_ids;
            }

            for (int slice = 1; slice <= options.time_slices; ++slice) {
                const auto found = events_by_slice.find(slice);
                if (found != events_by_slice.end() && !found->second.empty()) {
                    for (const auto node : found->second) {
                        if (std::find(data.available_request_ids.begin(), data.available_request_ids.end(), node) ==
                            data.available_request_ids.end()) {
                            data.available_request_ids.push_back(node);
                            ants.best_so_far_ant.visited[static_cast<std::size_t>(node)] = false;
                            ants.best_so_far_ant.to_visit += 1;
                        }
                    }

                    auto unrouted = acs_km::unrouted_customers(ants.best_so_far_ant, data);
                    std::vector<int> committed_starts;
                    committed_starts.reserve(static_cast<std::size_t>(ants.best_so_far_ant.used_vehicles));
                    for (int index = 0; index < ants.best_so_far_ant.used_vehicles; ++index) {
                        committed_starts.push_back(acs_km::get_last_committed_pos(ants, index));
                    }
                    acs_km::insert_unrouted_customers(ants.best_so_far_ant, data, unrouted, 0, committed_starts);

                    while (ants.best_so_far_ant.to_visit > 0) {
                        ants.best_so_far_ant.used_vehicles++;
                        const auto index_tour = ants.best_so_far_ant.used_vehicles - 1;
                        ants.best_so_far_ant.tours.emplace_back(std::vector<int>{-1});
                        ants.best_so_far_ant.tour_lengths.push_back(0.0);
                        ants.best_so_far_ant.current_quantity.push_back(0.0);
                        ants.best_so_far_ant.current_time.push_back(0.0);
                        acs_km::choose_closest_nn(ants.best_so_far_ant, index_tour, data, ants);
                        if (ants.best_so_far_ant.to_visit > 0) {
                            unrouted = acs_km::unrouted_customers(ants.best_so_far_ant, data);
                            committed_starts.clear();
                            for (int index = 0; index < ants.best_so_far_ant.used_vehicles; ++index) {
                                committed_starts.push_back(acs_km::get_last_committed_pos(ants, index));
                            }
                            acs_km::insert_unrouted_customers(ants.best_so_far_ant, data, unrouted, index_tour, committed_starts);
                        }
                        ants.best_so_far_ant.tours[static_cast<std::size_t>(index_tour)].push_back(-1);
                    }
                    ants.best_so_far_ant.total_tour_length = 0.0;
                    for (int i = 0; i < ants.best_so_far_ant.used_vehicles; ++i) {
                        ants.best_so_far_ant.tour_lengths[static_cast<std::size_t>(i)] = acs_km::compute_tour_length_plus1_indexed(
                            ants.best_so_far_ant.tours[static_cast<std::size_t>(i)], data.problem.distance);
                        ants.best_so_far_ant.total_tour_length += ants.best_so_far_ant.tour_lengths[static_cast<std::size_t>(i)];
                    }
                }

                if (acs_km::check_new_committed_nodes(ants, ants.best_so_far_ant, slice, simulation.slice_length_seconds)) {
                    acs_km::commit_nodes(ants, ants.best_so_far_ant, slice, simulation.slice_length_seconds);
                }

                ants.trail_0 = 1.0 / (static_cast<double>(data.available_request_ids.size() + 1) *
                                      std::max(1e-12, ants.best_so_far_ant.total_tour_length));
                acs_km::preserve_pheromones(ants, data, inout.pheromone_preservation);
                acs_km::run_colony_iterations(data, ants, inout, options.max_iterations_per_slice);
            }

            std::cout << "Final best solution >> vehicles=" << ants.best_so_far_ant.used_vehicles
                      << " total tours length=" << ants.best_so_far_ant.total_tour_length << '\n';
            std::cout << "Total number of evaluations: " << inout.no_evaluations << '\n';
            std::cout << "Total number of feasible solutions: " << inout.no_solutions << '\n';
        }

        std::cout << "Elapsed seconds: " << timer.elapsed_seconds() << '\n';
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
