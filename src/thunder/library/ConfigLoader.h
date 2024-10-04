#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>
#include "utils.h"

namespace thunder_ns {

    class ConfigLoader {
    public:
        virtual ~ConfigLoader() = default;
        virtual Config load(const std::string& file) const = 0;
    };

}

#endif // CONFIG_LOADER_H