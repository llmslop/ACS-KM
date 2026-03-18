#pragma once

#include <cstddef>
#include <vector>

#include "acs_km/problem.hpp"

namespace acs_km {

enum class DistanceType {
    att,
    ceil_2d,
    euc_2d,
    geo,
};

struct NearestNeighborLists {
    int used_depth{};
    std::vector<std::vector<int>> excluding_depot;
    std::vector<std::vector<int>> including_depot;
};

[[nodiscard]] double dtrunc(double value);

[[nodiscard]] double euclidean_distance(const std::vector<Point>& nodes, int i, int j);
[[nodiscard]] int ceil_distance(const std::vector<Point>& nodes, int i, int j);
[[nodiscard]] int geo_distance(const std::vector<Point>& nodes, int i, int j);
[[nodiscard]] int att_distance(const std::vector<Point>& nodes, int i, int j);

[[nodiscard]] std::vector<std::vector<double>> compute_distances(
    const std::vector<Point>& nodes,
    DistanceType distance_type,
    double scaling_value = 0.0);

[[nodiscard]] NearestNeighborLists compute_nn_lists(
    const std::vector<std::vector<double>>& distance_matrix,
    int nn_depth);

[[nodiscard]] double compute_tour_length_plus1_indexed(
    const std::vector<int>& tour,
    const std::vector<std::vector<double>>& distance_matrix);

[[nodiscard]] double compute_tour_length_direct_indexed(
    const std::vector<int>& tour,
    const std::vector<std::vector<double>>& distance_matrix);

[[nodiscard]] double compute_tour_length_with_depot_edges(
    const std::vector<int>& tour,
    const std::vector<std::vector<double>>& distance_matrix);

}  // namespace acs_km
