#ifndef DH_CONFIG_LOADER_H
#define DH_CONFIG_LOADER_H

#include "ConfigLoader.h"

namespace thunder_ns
{

    class DHConfigLoader : public ConfigLoader
    {
    public:
        Config load(const std::string &file) const override;
        casadi::SX DHMatrix(const Eigen::MatrixXd &rowDHTable) const;
        
    };

}

#endif // DH_CONFIG_LOADER_H