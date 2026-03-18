#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "acs_km/inout.hpp"
#include "acs_km/parse.hpp"

namespace {

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_parse_as() {
    acs_km::AntAlgorithmState ants;
    acs_km::InOutState inout;
    acs_km::parse_commandline({"--as"}, 0, ants, inout);

    expect(ants.as_flag, "Expected AS flag");
    expect(!ants.acs_flag, "Expected ACS flag off for --as");
    expect(!ants.acs_km_flag, "Expected ACS-KM flag off for --as");
}

void test_parse_acs_km() {
    acs_km::AntAlgorithmState ants;
    acs_km::InOutState inout;
    acs_km::parse_commandline({"--acs-km"}, 1, ants, inout);

    expect(ants.acs_flag, "Expected ACS flag on for --acs-km");
    expect(ants.acs_km_flag, "Expected ACS-KM flag on for --acs-km");
    expect(!ants.as_flag, "Expected AS flag off for --acs-km");
}

void test_parse_rejects_multiple_algorithms() {
    acs_km::AntAlgorithmState ants;
    acs_km::InOutState inout;
    auto threw = false;
    try {
        acs_km::parse_commandline({"--as", "--acs"}, 0, ants, inout);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    expect(threw, "Expected parser to reject multiple algorithm options");
}

void test_inout_statistics() {
    const std::vector<int> values{1, 2, 3, 4};
    expect(acs_km::average(values) == 2.5F, "Expected arithmetic mean 2.5");
    expect(acs_km::variance(values) == 1.25F, "Expected variance 1.25");
}

}  // namespace

int main() {
    try {
        test_parse_as();
        test_parse_acs_km();
        test_parse_rejects_multiple_algorithms();
        test_inout_statistics();
        std::cout << "All parse/inout tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Test failure: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
