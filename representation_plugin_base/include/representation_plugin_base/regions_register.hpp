#ifndef REPRESENTATION_PLUGINS__REGIONS_REGISTER_HPP
#define REPRESENTATION_PLUGINS__REGIONS_REGISTER_HPP

#include <memory>
#include <string>
#include <map>
#include <vector>

namespace representation_plugins{
	class RegionsRegister{
		protected:
			std::map<std::vector<std::string>, int> areas_;
		public:
			RegionsRegister();
			~RegionsRegister();

			int addArea(const std::vector<std::string> & regs);
			int findRegions(const std::vector<std::string> & regs) const;
			std::vector<std::string> findRegionsById(const int & id) const;
			void clear();

			// Debugging functions
			int getRegionsNumber() const;
			void print() const;
	};
}  // representation_plugins
#endif