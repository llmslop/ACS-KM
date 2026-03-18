#pragma once

#include <chrono>

namespace acs_km {

class Timer {
public:
    void start() {
        start_time_ = std::chrono::steady_clock::now();
    }

    [[nodiscard]] double elapsed_seconds() const {
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - start_time_).count();
    }

private:
    std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
};

}  // namespace acs_km
