#pragma once

#include <filesystem>

#include "acs_km/problem.hpp"

namespace acs_km {

class DataReader {
public:
    explicit DataReader(std::filesystem::path file_path);

    [[nodiscard]] InstanceData read() const;

private:
    std::filesystem::path file_path_;
};

}  // namespace acs_km
