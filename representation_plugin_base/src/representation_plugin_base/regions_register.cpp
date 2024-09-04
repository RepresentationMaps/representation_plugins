#include <iostream>
#include <algorithm>

#include "representation_plugin_base/regions_register.hpp"

namespace representation_plugins{
	RegionsRegister::RegionsRegister(const bool & threaded){
		threaded_ = threaded;
	}

	RegionsRegister::~RegionsRegister(){}

	int RegionsRegister::addArea(const std::vector<std::string> & regs){
		// adds a new area assigned to the set of regions regs
		int id;
		std::lock_guard<std::recursive_mutex> guard(register_mutex_);
		if (areas_.empty()){
			id = 0;
		} else {
			auto last = areas_.rbegin();
			int last_id = last->second;
			id = last_id+1;
		}
		areas_[regs] = id;
		return id;
	}

	std::map<int, int> RegionsRegister::removeRegion(const std::string & reg){
		// remove an entire region from the register
		std::vector<std::vector<std::string>> areas_to_remove;
		std::map<std::vector<std::string>, int> areas_to_add;
		std::map<int, int> ids_to_update;
		for (const auto & area: areas_){
			auto regs =  area.first;
			auto reg_elem = std::find(regs.begin(), regs.end(), reg); 
			if (reg_elem == regs.end()){
				continue;
			}
			// At this point, we know that the region we are removing
			// is contained inside of this area;
			// Therefore, we add this area to those to remove
			areas_to_remove.push_back(regs);
			regs.erase(reg_elem);
			if (regs.size() == 0){
				// We just found the register entrance
				// cotaining only the region we are deleting
				continue;
			}
			int old_id = area.second;
			int new_id = findRegions(regs);
			if (new_id == -1){
				areas_to_add.insert(std::pair(regs, old_id));
			} else {
				ids_to_update.insert(std::pair<int, int>(old_id, new_id));
			}
		}
		// We proceed removing the elements from the areas, according
		// to the previous iteration
		for (const auto & area: areas_to_remove){
			areas_.erase(area);
		}
		// We complete the updated by adding those new areas
		// created by the removal of this RegionOfSpace
		for (const auto & area: areas_to_add){
			areas_.insert(area);
		}
		return ids_to_update;
	}

	int RegionsRegister::findRegions(const std::vector<std::string> & regs) const {
		auto elem = areas_.find(regs);
		if (elem == areas_.end()){
			return -1;
		} else {
			return elem->second;
		}
	}

	std::vector<std::string> RegionsRegister::findRegionsById(const int & id) const {
		std::vector<std::string> regs;
		for (auto elem : areas_){
			if (elem.second == id){
				regs = elem.first;
				break;
			}
		}
		return regs;
	}

	void RegionsRegister::clear(){
		areas_.clear();
	}

	int RegionsRegister::getRegionsNumber() const {
		return areas_.size();
	}

	void RegionsRegister::print() const {
		for (auto elem : areas_){
			std::cout << "Area " << elem.second << ": ";
			for (auto reg : elem.first){
				std::cout << reg << " ";
			}
			std::cout << std::endl;
		}
	}
} // representation_plugins
