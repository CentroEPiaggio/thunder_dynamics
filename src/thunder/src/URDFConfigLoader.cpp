#include "URDFConfigLoader.h"
#include <iostream>

namespace thunder_ns {

    Config URDFConfigLoader::load(const std::string& file) const {
        Config config;
        // Implement URDF parsing logic here
        std::cout << "Loading URDF config from " << file << std::endl;
        // Populate the config object
        return config;
    }

}