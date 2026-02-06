#ifndef PLUGINREGISTRY_H
#define PLUGINREGISTRY_H

#include "plugin_interfaces.h"
#include <map>
#include <string>
#include <memory>

//---------------------------------------//
// TODO: ADD ALL NEW PLUGIN HEADERS HERE //
//---------------------------------------//
// - Loaders - //
#include "plugins/loaders/dh_loader.h"
#include "plugins/loaders/kin_loader.h"
#include "plugins/loaders/dyn_loader.h"
#include "plugins/loaders/soft_loader.h"
#include "plugins/loaders/urdf_loader.h"
#include "plugins/loaders/kinTree_loader.h"
#include "plugins/loaders/dynTree_loader.h"

// - Builders - //
#include "plugins/builders/kin_builder.h"
#include "plugins/builders/dyn_builder.h"
#include "plugins/builders/reg_builder.h"
#include "plugins/builders/soft_builder.h"
#include "plugins/builders/example_builder.h"
#include "plugins/builders/kinTree_builder.h"
#include "plugins/builders/dynTree_builder.h"

// - Generators - //
#include "plugins/generators/robot_generator.h"


namespace thunder_ns {

/**
 * @brief A static registry for all Loader plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseLoader>> LOADERS = {
	// TODO: ADD NEW LOADERS HERE
	{"dh_loader", std::make_shared<DHLoader>()},
	{"kin_loader", std::make_shared<KinLoader>()},
	{"dyn_loader", std::make_shared<DynLoader>()},
	{"soft_loader", std::make_shared<SoftLoader>()},
	{"urdf_loader", std::make_shared<UrdfLoader>()},
	{"kinTree_loader", std::make_shared<KinTreeLoader>()},
	{"dynTree_loader", std::make_shared<DynTreeLoader>()},
};

/**
 * @brief A static registry for all Builder plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseBuilder>> POPULATORS = {
	// TODO: ADD NEW POPULATORS HERE
	{"kin_builder", std::make_shared<KinBuilder>()},
	{"dyn_builder", std::make_shared<DynBuilder>()},
	{"reg_builder", std::make_shared<RegBuilder>()},
	{"soft_builder", std::make_shared<SoftBuilder>()},
	{"example_builder", std::make_shared<ExampleBuilder>()},
	{"kinTree_builder", std::make_shared<KinTreeBuilder>()},
	{"dynTree_builder", std::make_shared<DynTreeBuilder>()},
};

/**
 * @brief A static registry for all Generator plugins.
 */
inline const std::map<std::string, std::shared_ptr<BaseGenerator>> GENERATORS = {
	// TODO: ADD NEW GENERATORS HERE
	{"robot_generator", std::make_shared<RobotGenerator>()},
	
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

#endif