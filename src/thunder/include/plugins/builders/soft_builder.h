#ifndef SOFT_BUILDER_H
#define SOFT_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class SoftBuilder : public BaseBuilder {

		public:
			SoftBuilder() : BaseBuilder("Soft-Robots Builder", "Build elastic joints functions.") {}

			int compute_elastic(std::shared_ptr<Robot> robot);
		
			void build(std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // SOFT_BUILDER_H