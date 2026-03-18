#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "acs_km/controller_utils.hpp"
#include "acs_km/problem.hpp"

namespace acs_km {

struct SimulationConfig {
    double working_day_seconds{100.0};
    int time_slices{50};
};

struct SliceEvent {
    int slice_index{};
    double trigger_time{};
    std::vector<int> newly_available_node_ids;
};

struct SliceSimulationResult {
    double scaling_value{};
    double slice_length_seconds{};
    std::vector<SliceEvent> events;
    std::size_t total_newly_available{};
};

inline SliceSimulationResult simulate_dynamic_release(const InstanceData& data,
                                                      const SimulationConfig& config = {}) {
    if (data.requests.empty()) {
        throw std::runtime_error("Cannot simulate dynamic release without requests");
    }
    if (config.time_slices <= 0 || config.working_day_seconds <= 0.0) {
        throw std::runtime_error("Invalid simulation configuration");
    }

    const auto& depot = data.requests.front();
    const auto time_span = depot.end_window - depot.start_window;
    if (time_span <= 0.0) {
        throw std::runtime_error("Invalid depot time window span");
    }

    SliceSimulationResult result;
    result.scaling_value = config.working_day_seconds / time_span;
    result.slice_length_seconds = config.working_day_seconds / static_cast<double>(config.time_slices);

    std::vector<Request> scaled_dynamic_requests = data.dynamic_requests;
    for (auto& request : scaled_dynamic_requests) {
        request.available_time *= result.scaling_value;
    }
    std::sort(scaled_dynamic_requests.begin(), scaled_dynamic_requests.end());

    std::size_t last_known_dynamic_index = 0;
    for (int current_slice = 1; current_slice <= config.time_slices; ++current_slice) {
        const auto threshold = static_cast<double>(current_slice) * result.slice_length_seconds;
        auto newly_available = collect_newly_available_nodes(
            scaled_dynamic_requests,
            threshold,
            last_known_dynamic_index);

        if (!newly_available.empty()) {
            result.total_newly_available += newly_available.size();
            result.events.push_back(SliceEvent{
                .slice_index = current_slice,
                .trigger_time = threshold,
                .newly_available_node_ids = std::move(newly_available),
            });
        }
    }

    return result;
}

}  // namespace acs_km
