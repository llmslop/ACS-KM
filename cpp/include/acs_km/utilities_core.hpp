#pragma once

#include <cstddef>
#include <random>
#include <vector>

namespace acs_km {

class UtilitiesCore {
public:
    explicit UtilitiesCore(unsigned int seed = std::random_device{}());

    [[nodiscard]] double random01();

    static void sort2(std::vector<double>& values, std::vector<int>& paired_indices);
    static void sort2_boxed(std::vector<double>& values, std::vector<int>& paired_indices);

private:
    static void sort2_impl(std::vector<double>& values, std::vector<int>& paired_indices, int left, int right);
    static void swap2(std::vector<double>& values, std::vector<int>& paired_indices, int i, int j);

    std::mt19937 generator_;
    std::uniform_real_distribution<double> distribution_{0.0, 1.0};
};

}  // namespace acs_km
