
#pragma once
#include "plugin_interfaces.h"
#include <map>
#include <string>
#include <memory>

// TODO: ADD ALL NEW PLUGIN HEADERS HERE
#include "plugins/dh_loader.h"
#include "plugins/kinematic_plugin.h"
#include "plugins/cpp_generator.h"


namespace thunder_ns {

/**
 * @brief A static registry for all Loader plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseLoader>> LOADERS = {
    // TODO: ADD NEW LOADERS HERE
    {"dh_loader", std::make_shared<DHLoader>()},
};

/**
 * @brief A static registry for all Builder plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseBuilder>> POPULATORS = {
    // TODO: ADD NEW POPULATORS HERE
    {"kin", std::make_shared<KinBuilder>()},
};

/**
 * @brief A static registry for all Generator plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseGenerator>> GENERATORS = {
    // TODO: ADD NEW GENERATORS HERE
    {"cpp", std::make_shared<CPPGenerator>()},
};






/**
 * @brief Helper function to find a loader by name.
 * It searches all three maps and returns the first match.
 *
 * @param plugin_type The name of the loader (e.g., "dh_loader").
 * @return A shared_ptr to the BaseLoader, or nullptr if not found.
 */
inline std::shared_ptr<BaseLoader> find_loader(const std::string& plugin_type) {
    if (LOADERS.count(plugin_type)) {
        return LOADERS.at(plugin_type);
    }
    return nullptr;
}


/**
 * @brief Helper function to find a builder by name.
 * It searches all three maps and returns the first match.
 *
 * @param plugin_type The name of the builder.
 * @return A shared_ptr to the Basebuilder, or nullptr if not found.
 */
inline std::shared_ptr<BaseBuilder> find_builder(const std::string& plugin_type) {
    if (POPULATORS.count(plugin_type)) {
        return POPULATORS.at(plugin_type);
    }
    return nullptr;
}

/**
 * @brief Helper function to find a generator by name.
 * It searches all three maps and returns the first match.
 *
 * @param plugin_type The name of the generator (e.g., "dh_generator").
 * @return A shared_ptr to the BaseGenerator, or nullptr if not found.
 */
inline std::shared_ptr<BaseGenerator> find_generator(const std::string& plugin_type) {
    if (GENERATORS.count(plugin_type)) {
        return GENERATORS.at(plugin_type);
    }
    return nullptr;
}


} // namespace thunder_ns