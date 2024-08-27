#include <iostream>

#include "representation_plugin_base/regions_register.hpp"

namespace representation_plugins{
	RegionsRegister::RegionsRegister(){}

	RegionsRegister::~RegionsRegister(){}

	int RegionsRegister::addArea(const std::vector<std::string> & regs){
		// adds a new area assigned to the set of regions regs
		int id;
		if (areas_.empty()){
			id = 0;
		} else {
			auto last = areas_.rbegin();
			int last_id = last->second;
			std::cout<<"Last id: "<<last_id<<std::endl;
			id = last_id+1;
			std::cout<<"New id: "<<id<<std::endl;
		}
		areas_[regs] = id;
		return id;
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
