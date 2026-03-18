#include "acs_km/parse.hpp"

#include <iostream>
#include <stdexcept>

namespace acs_km {

void parse_commandline(const std::vector<std::string>& args,
                       const int run_number,
                       AntAlgorithmState& ants,
                       InOutState&) {
    if (args.empty()) {
        throw std::runtime_error("No options are specified. Try `--help' for more information.");
    }

    const auto has_option = [&args](const std::string& short_opt, const std::string& long_opt) {
        for (const auto& arg : args) {
            if (arg == short_opt || arg == long_opt) {
                return true;
            }
        }
        return false;
    };

    const auto use_as = has_option("-u", "--as");
    const auto use_acs = has_option("-z", "--acs");
    const auto use_acs_km = has_option("-l", "--acs-km");

    const auto algorithm_count = static_cast<int>(use_as) + static_cast<int>(use_acs) + static_cast<int>(use_acs_km);
    if (algorithm_count > 1) {
        throw std::runtime_error("Error: More than one ACO algorithm enabled in the command line.");
    }
    if (algorithm_count == 1) {
        ants.as_flag = false;
        ants.acs_flag = false;
        ants.acs_km_flag = false;
    }

    if (use_as) {
        ants.as_flag = true;
        set_default_as_parameters(ants);
        std::cout << "\nRun basic Ant System #" << (run_number + 1) << '\n';
    }
    if (use_acs || use_acs_km) {
        ants.acs_flag = true;
        set_default_acs_parameters(ants);
        std::cout << "\nRun Ant Colony System #" << (run_number + 1) << '\n';
    }
    if (use_acs_km) {
        ants.acs_km_flag = true;
    }
}

}  // namespace acs_km
