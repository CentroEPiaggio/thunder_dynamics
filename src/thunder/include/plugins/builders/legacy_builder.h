#ifndef LEGACY_BUILDER_H
#define LEGACY_BUILDER_H

#include "plugin_interfaces.h"
#include "plugins/builders/common/kinematics.h"
#include "plugins/builders/common/dynamics.h"
#include "plugins/builders/common/regressors.h"
#include "plugins/builders/common/userDefined.h"


namespace thunder_ns {

	class LegacyBuilder : public BaseBuilder {

		public:
			LegacyBuilder() : BaseBuilder("Legacy Builder", "Build everythink like the old times.") {}
		
			void build(std::shared_ptr<Robot> robot) override;

	};

	
} // namespace thunder_ns

#endif // LEGACY_BUILDER_H