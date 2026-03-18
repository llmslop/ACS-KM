#include "acs_km/utilities_core.hpp"

#include <stdexcept>

namespace acs_km {

UtilitiesCore::UtilitiesCore(const unsigned int seed) : generator_(seed) {}

double UtilitiesCore::random01() {
    return distribution_(generator_);
}

void UtilitiesCore::swap2(std::vector<double>& values,
                          std::vector<int>& paired_indices,
                          const int i,
                          const int j) {
    std::swap(values[static_cast<std::size_t>(i)], values[static_cast<std::size_t>(j)]);
    std::swap(paired_indices[static_cast<std::size_t>(i)], paired_indices[static_cast<std::size_t>(j)]);
}

void UtilitiesCore::sort2_impl(std::vector<double>& values,
                               std::vector<int>& paired_indices,
                               const int left,
                               const int right) {
    if (left >= right) {
        return;
    }
    swap2(values, paired_indices, left, (left + right) / 2);
    auto last = left;
    for (auto k = left + 1; k <= right; ++k) {
        if (values[static_cast<std::size_t>(k)] < values[static_cast<std::size_t>(left)]) {
            swap2(values, paired_indices, ++last, k);
        }
    }
    swap2(values, paired_indices, left, last);
    sort2_impl(values, paired_indices, left, last);
    sort2_impl(values, paired_indices, last + 1, right);
}

void UtilitiesCore::sort2(std::vector<double>& values, std::vector<int>& paired_indices) {
    if (values.size() != paired_indices.size()) {
        throw std::runtime_error("sort2 expects equal vector sizes");
    }
    if (values.empty()) {
        return;
    }
    sort2_impl(values, paired_indices, 0, static_cast<int>(values.size() - 1));
}

void UtilitiesCore::sort2_boxed(std::vector<double>& values, std::vector<int>& paired_indices) {
    sort2(values, paired_indices);
}

}  // namespace acs_km
