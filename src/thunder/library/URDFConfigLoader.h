#ifndef URDF_CONFIG_LOADER_H
#define URDF_CONFIG_LOADER_H

#include "ConfigLoader.h"

namespace thunder_ns {

    class URDFConfigLoader : public ConfigLoader {
    public:
        Config load(const std::string& file) const override;
    };

}

#endif // URDF_CONFIG_LOADER_H