#ifndef FLOATING_LOADER_H
#define FLOATING_LOADER_H

#include "plugin_interfaces.h"

namespace thunder_ns {

	/**
	 * Declares a floating base: the root of the structure loaded before (urdf_loader, kin_loader, dh_loader, ...)
	 * moves freely in the world. Adds the variables pose_base, twist_base, acc_base, twist_base_r, acc_base_r,
	 * jerk_base and the property floating = true, read by the builders. See floating_base.h for the conventions.
	 *
	 *   pipeline:
	 *     loaders: [urdf_loader, floating_loader]
	 *   floating_loader:
	 *     base: base_Link      # optional, the root of the structure; only the root can float for now
	 *     twist: world         # world (default): twist_base = [v; w] in world axes, body: in base axes
	 */
	class FloatingLoader : public BaseLoader {
		public:
			FloatingLoader() : BaseLoader("Floating base Loader", "Make the root of the structure a floating base.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot) override;
	};

} // namespace thunder_ns

#endif // FLOATING_LOADER_H
