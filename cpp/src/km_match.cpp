#include "acs_km/km_match.hpp"

#include <algorithm>
#include <stdexcept>

namespace acs_km {

void KMMatch::resize(const std::size_t n) {
    pop_ = n;
    w_.assign(pop_, std::vector<int>(pop_, 0));
    x_.assign(pop_, 0);
    y_.assign(pop_, 0);
    prev_x_.assign(pop_, 0);
    prev_y_.assign(pop_, 0);
    son_y_.assign(pop_, 0);
    slack_.assign(pop_, 0);
    par_.assign(pop_, 0);
}

void KMMatch::adjust(const std::size_t v) {
    son_y_[v] = prev_y_[v];
    if (prev_x_[static_cast<std::size_t>(son_y_[v])] != -2) {
        adjust(static_cast<std::size_t>(prev_x_[static_cast<std::size_t>(son_y_[v])]));
    }
}

bool KMMatch::find(const std::size_t v) {
    for (std::size_t i = 0; i < pop_; ++i) {
        if (prev_y_[i] == -1) {
            const auto reduced_cost = x_[v] + y_[i] - w_[v][i];
            if (slack_[i] > reduced_cost) {
                slack_[i] = reduced_cost;
                par_[i] = static_cast<int>(v);
            }
            if (x_[v] + y_[i] == w_[v][i]) {
                prev_y_[i] = static_cast<int>(v);
                if (son_y_[i] == -1) {
                    adjust(i);
                    return true;
                }
                if (prev_x_[static_cast<std::size_t>(son_y_[i])] != -1) {
                    continue;
                }
                prev_x_[static_cast<std::size_t>(son_y_[i])] = static_cast<int>(i);
                if (find(static_cast<std::size_t>(son_y_[i]))) {
                    return true;
                }
            }
        }
    }

    return false;
}

int KMMatch::solve_max() {
    if (pop_ == 0) {
        return 0;
    }
    if (w_.size() != pop_) {
        throw std::runtime_error("KMMatch weight matrix row count does not match size");
    }
    for (const auto& row : w_) {
        if (row.size() != pop_) {
            throw std::runtime_error("KMMatch weight matrix must be square");
        }
    }

    std::fill(son_y_.begin(), son_y_.end(), -1);
    std::fill(y_.begin(), y_.end(), 0);

    for (std::size_t i = 0; i < pop_; ++i) {
        x_[i] = 0;
        for (std::size_t j = 0; j < pop_; ++j) {
            x_[i] = std::max(x_[i], w_[i][j]);
        }
    }

    for (std::size_t i = 0; i < pop_; ++i) {
        std::fill(prev_x_.begin(), prev_x_.end(), -1);
        std::fill(prev_y_.begin(), prev_y_.end(), -1);
        std::fill(slack_.begin(), slack_.end(), inf_);
        prev_x_[i] = -2;
        if (find(i)) {
            continue;
        }

        bool matched = false;
        while (!matched) {
            auto minimum_slack = inf_;
            for (std::size_t j = 0; j < pop_; ++j) {
                if (prev_y_[j] == -1) {
                    minimum_slack = std::min(minimum_slack, slack_[j]);
                }
            }

            for (std::size_t j = 0; j < pop_; ++j) {
                if (prev_x_[j] != -1) {
                    x_[j] -= minimum_slack;
                }
                if (prev_y_[j] != -1) {
                    y_[j] += minimum_slack;
                } else {
                    slack_[j] -= minimum_slack;
                }
            }

            for (std::size_t j = 0; j < pop_; ++j) {
                if (prev_y_[j] == -1 && slack_[j] == 0) {
                    prev_y_[j] = par_[j];
                    if (son_y_[j] == -1) {
                        adjust(j);
                        matched = true;
                        break;
                    }
                    prev_x_[static_cast<std::size_t>(son_y_[j])] = static_cast<int>(j);
                    if (find(static_cast<std::size_t>(son_y_[j]))) {
                        matched = true;
                        break;
                    }
                }
            }
        }
    }

    auto answer = 0;
    for (std::size_t right = 0; right < pop_; ++right) {
        answer += w_[static_cast<std::size_t>(son_y_[right])][right];
    }
    return answer;
}

}  // namespace acs_km
