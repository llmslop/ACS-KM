#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "acs_km/km_match.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

int brute_force_max_assignment(const std::vector<std::vector<int>>& weights) {
    const auto n = weights.size();
    if (n == 0) {
        return 0;
    }
    std::vector<std::size_t> permutation(n);
    std::iota(permutation.begin(), permutation.end(), 0U);

    auto best = std::numeric_limits<int>::min();
    do {
        auto score = 0;
        for (std::size_t right = 0; right < n; ++right) {
            score += weights[permutation[right]][right];
        }
        best = std::max(best, score);
    } while (std::next_permutation(permutation.begin(), permutation.end()));

    return best;
}

void test_empty_matrix() {
    acs_km::KMMatch km;
    km.resize(0);
    expect(km.solve_max() == 0, "Expected 0 for empty assignment problem");
}

void test_known_case() {
    acs_km::KMMatch km;
    km.resize(3);
    auto& w = km.weights();
    w[0][0] = 1;
    w[0][1] = 3;
    w[0][2] = 2;
    w[1][0] = 2;
    w[1][1] = 1;
    w[1][2] = 3;
    w[2][0] = 3;
    w[2][1] = 2;
    w[2][2] = 1;

    const auto expected = brute_force_max_assignment(w);
    const auto got = km.solve_max();
    expect(got == expected, "Expected known matrix score to match brute-force optimum");
}

void test_negative_weights_for_minimization_via_negation() {
    acs_km::KMMatch km;
    km.resize(4);
    auto& w = km.weights();

    // If we negate costs, max-weight matching corresponds to min-cost matching.
    const std::vector<std::vector<int>> costs{
        {9, 2, 7, 8},
        {6, 4, 3, 7},
        {5, 8, 1, 8},
        {7, 6, 9, 4},
    };
    for (std::size_t i = 0; i < costs.size(); ++i) {
        for (std::size_t j = 0; j < costs.size(); ++j) {
            w[i][j] = -costs[i][j];
        }
    }

    const auto expected = brute_force_max_assignment(w);
    const auto got = km.solve_max();
    expect(got == expected, "Expected negated-cost matrix score to match brute-force optimum");
}

}  // namespace

int main() {
    try {
        test_empty_matrix();
        test_known_case();
        test_negative_weights_for_minimization_via_negation();
        std::cout << "All KMMatch tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
