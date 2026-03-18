#include "acs_km/utilities_core.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace acs_km {

UtilitiesCore::UtilitiesCore(const unsigned int seed) : generator_(seed) {}

double UtilitiesCore::random01() {
    return distribution_(generator_);
}

void UtilitiesCore::sort2(std::vector<double>& values, std::vector<int>& paired_indices) {
    if (values.size() != paired_indices.size()) {
        throw std::runtime_error("sort2 expects equal vector sizes");
    }
    if (values.empty()) {
        return;
    }

    std::vector<std::pair<double, int>> paired;
    paired.reserve(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
        paired.emplace_back(values[i], paired_indices[i]);
    }
    std::sort(paired.begin(), paired.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });
    for (std::size_t i = 0; i < paired.size(); ++i) {
        values[i] = paired[i].first;
        paired_indices[i] = paired[i].second;
    }
}

void UtilitiesCore::sort2_boxed(std::vector<double>& values, std::vector<int>& paired_indices) {
    sort2(values, paired_indices);
}

}  // namespace acs_km
