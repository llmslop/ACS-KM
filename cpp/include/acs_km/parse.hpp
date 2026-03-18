#pragma once

#include <string>
#include <vector>

#include "acs_km/ants_state.hpp"
#include "acs_km/inout.hpp"

namespace acs_km {

void parse_commandline(const std::vector<std::string>& args,
                       int run_number,
                       AntAlgorithmState& ants,
                       InOutState& inout);

}  // namespace acs_km
