#pragma once

namespace acs_km {

struct Request {
    int id{};
    int x_coord{};
    int y_coord{};
    int demand{};
    double start_window{};
    double end_window{};
    double service_time{};
    double available_time{};

    [[nodiscard]] bool operator<(const Request& other) const {
        return available_time < other.available_time;
    }
};

}  // namespace acs_km
