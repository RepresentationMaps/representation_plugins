#ifndef REPRESENTATION_PLUGINS__REGIONS_REGISTER_HPP
#define REPRESENTATION_PLUGINS__REGIONS_REGISTER_HPP

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <mutex>

namespace representation_plugins{
	class RegionsRegister{
		protected:
			bool threaded_;
			std::recursive_mutex register_mutex_;
			std::map<std::vector<std::string>, int> areas_;
			int id_;
		public:
			RegionsRegister(const bool & threaded);
			~RegionsRegister();

			int addArea(const std::vector<std::string> & regs);
			std::map<int, int> removeRegion(const std::string & reg);
			int findRegions(const std::vector<std::string> & regs) const;
			std::vector<std::string> findRegionsById(const int & id) const;
			void clear();

			// Debugging functions
			int getRegionsNumber() const;
			void print() const;
			int getId() const;
	};
}  // representation_plugins
#endif