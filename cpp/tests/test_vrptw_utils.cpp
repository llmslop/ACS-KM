#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "acs_km/vrptw_utils.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_close(const double lhs, const double rhs, const double tolerance, const std::string& message) {
    if (std::abs(lhs - rhs) > tolerance) {
        throw std::runtime_error(message);
    }
}

void test_compute_distances_euclidean_scaled() {
    const std::vector<acs_km::Point> nodes{
        {.x = 0, .y = 0},
        {.x = 3, .y = 4},
        {.x = 6, .y = 8},
    };
    const auto matrix = acs_km::compute_distances(nodes, acs_km::DistanceType::euc_2d, 2.0);

    expect_close(matrix[0][1], 10.0, 1e-9, "Expected scaled euclidean distance 0->1 to be 10");
    expect_close(matrix[1][2], 10.0, 1e-9, "Expected scaled euclidean distance 1->2 to be 10");
    expect_close(matrix[0][2], 20.0, 1e-9, "Expected scaled euclidean distance 0->2 to be 20");
}

void test_compute_nn_lists_excluding_and_including_depot() {
    const std::vector<std::vector<double>> matrix{
        {0, 1, 2, 3},
        {1, 0, 3, 2},
        {2, 3, 0, 1},
        {3, 2, 1, 0},
    };

    const auto nn = acs_km::compute_nn_lists(matrix, 2);

    expect(nn.used_depth == 2, "Expected NN depth 2");
    expect(nn.including_depot.size() == 4, "Expected NN rows per node");
    expect(nn.excluding_depot.size() == 4, "Expected NN rows per node");

    expect(nn.including_depot[1][0] == 0, "Expected depot to appear in include-depot list");
    expect(nn.excluding_depot[1][0] != 0, "Expected depot to be excluded from exclude-depot list");
}

void test_compute_tour_lengths() {
    const std::vector<std::vector<double>> matrix{
        {0, 10, 20, 30},
        {10, 0, 11, 12},
        {20, 11, 0, 13},
        {30, 12, 13, 0},
    };

    const std::vector<int> plus1_tour{-1, 0, 1, -1};
    const auto plus1_length = acs_km::compute_tour_length_plus1_indexed(plus1_tour, matrix);
    expect_close(plus1_length, 10 + 11 + 20, 1e-9, "Unexpected +1 indexed tour length");

    const std::vector<int> direct_tour{0, 1, 2, 0};
    const auto direct_length = acs_km::compute_tour_length_direct_indexed(direct_tour, matrix);
    expect_close(direct_length, 10 + 11 + 20, 1e-9, "Unexpected direct-indexed tour length");

    const std::vector<int> depot_edge_tour{-1, 0, 1, -1};
    const auto depot_edge_length = acs_km::compute_tour_length_with_depot_edges(depot_edge_tour, matrix);
    expect_close(depot_edge_length, 10 + 11 + 20, 1e-9, "Unexpected depot-edge tour length");
}

}  // namespace

int main() {
    try {
        test_compute_distances_euclidean_scaled();
        test_compute_nn_lists_excluding_and_including_depot();
        test_compute_tour_lengths();
        std::cout << "All VRPTW utility tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
