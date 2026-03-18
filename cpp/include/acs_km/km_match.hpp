#pragma once

#include <cstddef>
#include <vector>

namespace acs_km {

class KMMatch {
public:
    void resize(std::size_t n);

    [[nodiscard]] std::size_t size() const noexcept {
        return pop_;
    }

    [[nodiscard]] std::vector<std::vector<int>>& weights() noexcept {
        return w_;
    }

    [[nodiscard]] const std::vector<std::vector<int>>& weights() const noexcept {
        return w_;
    }

    [[nodiscard]] const std::vector<int>& matched_left_for_right() const noexcept {
        return son_y_;
    }

    [[nodiscard]] int solve_max();

private:
    void adjust(std::size_t v);
    [[nodiscard]] bool find(std::size_t v);

    static constexpr int inf_ = 1000000000;

    std::size_t pop_{0};
    std::vector<std::vector<int>> w_;
    std::vector<int> x_;
    std::vector<int> y_;
    std::vector<int> prev_x_;
    std::vector<int> prev_y_;
    std::vector<int> son_y_;
    std::vector<int> slack_;
    std::vector<int> par_;
};

}  // namespace acs_km
