#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "acs_km/utilities_core.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_sort2() {
    std::vector<double> values{3.0, 1.0, 2.0};
    std::vector<int> paired{30, 10, 20};
    acs_km::UtilitiesCore::sort2(values, paired);

    expect(values[0] == 1.0 && values[1] == 2.0 && values[2] == 3.0, "Expected ascending order");
    expect(paired[0] == 10 && paired[1] == 20 && paired[2] == 30, "Expected paired permutation preserved");
}

void test_random01_bounds() {
    acs_km::UtilitiesCore utils(123U);
    for (auto i = 0; i < 100; ++i) {
        const auto x = utils.random01();
        expect(x >= 0.0 && x <= 1.0, "Expected random01 output in [0, 1]");
    }
}

}  // namespace

int main() {
    try {
        test_sort2();
        test_random01_bounds();
        std::cout << "All utilities core tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
