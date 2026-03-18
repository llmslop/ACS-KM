#include "acs_km/vrptw_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace acs_km {

double dtrunc(const double value) {
    return static_cast<double>(static_cast<int>(value));
}

double euclidian_distance(const std::vector<Point>& nodes, const int i, const int j) {
    const auto xd = static_cast<double>(nodes[i].x - nodes[j].x);
    const auto yd = static_cast<double>(nodes[i].y - nodes[j].y);
    return std::sqrt(xd * xd + yd * yd);
}

int ceil_distance(const std::vector<Point>& nodes, const int i, const int j) {
    return static_cast<int>(std::ceil(euclidian_distance(nodes, i, j)));
}

int geo_distance(const std::vector<Point>& nodes, const int i, const int j) {
    const auto x1 = static_cast<double>(nodes[i].x);
    const auto x2 = static_cast<double>(nodes[j].x);
    const auto y1 = static_cast<double>(nodes[i].y);
    const auto y2 = static_cast<double>(nodes[j].y);

    auto deg = dtrunc(x1);
    auto min = x1 - deg;
    const auto lati = std::numbers::pi * (deg + 5.0 * min / 3.0) / 180.0;
    deg = dtrunc(x2);
    min = x2 - deg;
    const auto latj = std::numbers::pi * (deg + 5.0 * min / 3.0) / 180.0;

    deg = dtrunc(y1);
    min = y1 - deg;
    const auto longi = std::numbers::pi * (deg + 5.0 * min / 3.0) / 180.0;
    deg = dtrunc(y2);
    min = y2 - deg;
    const auto longj = std::numbers::pi * (deg + 5.0 * min / 3.0) / 180.0;

    const auto q1 = std::cos(longi - longj);
    const auto q2 = std::cos(lati - latj);
    const auto q3 = std::cos(lati + latj);
    return static_cast<int>(6378.388 * std::acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
}

int att_distance(const std::vector<Point>& nodes, const int i, const int j) {
    const auto xd = static_cast<double>(nodes[i].x - nodes[j].x);
    const auto yd = static_cast<double>(nodes[i].y - nodes[j].y);
    const auto rij = std::sqrt((xd * xd + yd * yd) / 10.0);
    const auto tij = dtrunc(rij);
    if (tij < rij) {
        return static_cast<int>(tij) + 1;
    }
    return static_cast<int>(tij);
}

std::vector<std::vector<double>> compute_distances(const std::vector<Point>& nodes,
                                                   const DistanceType distance_type,
                                                   const double scaling_value) {
    const auto size = nodes.size();
    std::vector<std::vector<double>> matrix(size, std::vector<double>(size, 0.0));

    for (std::size_t i = 0; i < size; ++i) {
        for (std::size_t j = 0; j < size; ++j) {
            switch (distance_type) {
                case DistanceType::att:
                    matrix[i][j] = static_cast<double>(att_distance(
                        nodes,
                        static_cast<int>(i),
                        static_cast<int>(j)));
                    break;
                case DistanceType::ceil_2d:
                    matrix[i][j] = static_cast<double>(ceil_distance(
                        nodes,
                        static_cast<int>(i),
                        static_cast<int>(j)));
                    break;
                case DistanceType::euc_2d:
                    matrix[i][j] = euclidian_distance(
                        nodes,
                        static_cast<int>(i),
                        static_cast<int>(j));
                    if (scaling_value != 0.0) {
                        matrix[i][j] *= scaling_value;
                    }
                    break;
                case DistanceType::geo:
                    matrix[i][j] = static_cast<double>(geo_distance(
                        nodes,
                        static_cast<int>(i),
                        static_cast<int>(j)));
                    break;
                default:
                    throw std::runtime_error("Unsupported distance type");
            }
        }
    }

    return matrix;
}

NearestNeighborLists compute_nn_lists(const std::vector<std::vector<double>>& distance_matrix,
                                      int nn_depth) {
    const auto node_count = distance_matrix.size();
    if (node_count <= 1) {
        return NearestNeighborLists{
            .used_depth = 0,
            .excluding_depot = {},
            .including_depot = {},
        };
    }

    const auto max_depth = static_cast<int>(node_count) - 2;
    if (nn_depth >= static_cast<int>(node_count)) {
        nn_depth = max_depth;
    }
    if (nn_depth < 0) {
        nn_depth = 0;
    }

    std::vector<std::vector<int>> excluding_depot(node_count, std::vector<int>(nn_depth, 0));
    std::vector<std::vector<int>> including_depot(node_count, std::vector<int>(nn_depth, 0));

    for (std::size_t node = 0; node < node_count; ++node) {
        std::vector<std::pair<double, int>> order;
        order.reserve(node_count);
        for (std::size_t i = 0; i < node_count; ++i) {
            if (i == node) {
                continue;
            }
            order.emplace_back(distance_matrix[node][i], static_cast<int>(i));
        }
        std::sort(order.begin(), order.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
            }
            return lhs.second < rhs.second;
        });

        auto idx = 0;
        for (const auto& entry : order) {
            if (idx >= nn_depth) {
                break;
            }
            including_depot[node][idx] = entry.second;
            ++idx;
        }

        idx = 0;
        for (const auto& entry : order) {
            if (idx >= nn_depth) {
                break;
            }
            const auto candidate = entry.second;
            if (candidate == 0) {
                continue;
            }
            excluding_depot[node][idx] = candidate;
            ++idx;
        }
    }

    return NearestNeighborLists{
        .used_depth = nn_depth,
        .excluding_depot = std::move(excluding_depot),
        .including_depot = std::move(including_depot),
    };
}

double compute_tour_length_plus1_indexed(const std::vector<int>& tour,
                                         const std::vector<std::vector<double>>& distance_matrix) {
    auto sum = 0.0;
    if (tour.size() <= 1) {
        return sum;
    }
    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        sum += distance_matrix[static_cast<std::size_t>(tour[i] + 1)][static_cast<std::size_t>(tour[i + 1] + 1)];
    }
    return sum;
}

double compute_tour_length_direct_indexed(const std::vector<int>& tour,
                                          const std::vector<std::vector<double>>& distance_matrix) {
    auto sum = 0.0;
    if (tour.size() <= 1) {
        return sum;
    }
    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        sum += distance_matrix[static_cast<std::size_t>(tour[i])][static_cast<std::size_t>(tour[i + 1])];
    }
    return sum;
}

double compute_tour_length_with_depot_edges(const std::vector<int>& tour,
                                            const std::vector<std::vector<double>>& distance_matrix) {
    auto sum = 0.0;
    if (tour.size() <= 1) {
        return sum;
    }

    sum += distance_matrix[0][static_cast<std::size_t>(tour[1] + 1)];
    for (std::size_t i = 1; i + 2 < tour.size(); ++i) {
        sum += distance_matrix[static_cast<std::size_t>(tour[i] + 1)][static_cast<std::size_t>(tour[i + 1] + 1)];
    }
    sum += distance_matrix[static_cast<std::size_t>(tour[tour.size() - 2] + 1)][0];
    return sum;
}

}  // namespace acs_km
