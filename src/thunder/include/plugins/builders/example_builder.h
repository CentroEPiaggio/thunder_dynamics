#ifndef EXAMPLE_BUILDER_H
#define EXAMPLE_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class ExampleBuilder : public BaseBuilder {

		public:
			ExampleBuilder() : BaseBuilder("Example Builder", "Examle of a builder plugin. It can embody a userDefined function to test.") {}

			int compute_userDefined(std::shared_ptr<Robot> robot);
			int compute_example_fun(std::shared_ptr<Robot> robot);

			void build(std::shared_ptr<Robot> robot) override;
			
	};

	
} // namespace thunder_ns

#endif // EXAMPLE_BUILDER_H