#ifndef MAP_HANLDER__SEMANTIC_MAP_HANDLER_HPP
#define MAP_HANDLER__SEMANTIC_MAP_HANDLER_HPP

#include <iostream>
#include <cmath>
#include <type_traits> 
#include <algorithm>
#include <chrono>
#include <ctime> 
#include <limits>

#include <openvdb/openvdb.h>
#include <openvdb/tree/ValueAccessor.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/Statistics.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/blocked_range2d.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "representation_plugin_base/regions_register.hpp"

namespace map_handler
{
	class SemanticMapHandler
	{
		private:
			std::shared_ptr<openvdb::Int32Grid> grid_;
			
			bool threaded_;

			float voxel_size_;
			bool vertex_centered_;
			openvdb::math::Vec3d offset_;
			openvdb::math::Transform::Ptr initial_transformation_;	

			std::string fixed_frame_;
			std::string map_frame_; // If these two are the same, then no rotation applied to the map

			using GridAccessorType = typename openvdb::Int32Grid::Accessor;

			static float computeDistance(const openvdb::Vec3d& a, const openvdb::Vec3d& b)
			{
				return std::sqrt(std::pow(a[0] - b[0],2) + std::pow(a[1] - b[1],2) + std::pow(a[2] - b[2],2));
			}

			int getAreaId(
				const GridAccessorType & accessor,
				const openvdb::Coord & ijk,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register){
				int id;
				if (!accessor.isValueOn(ijk)){
					std::vector<std::string> reg_vec({reg});
					id = reg_register.findRegions(reg_vec);
					if (id == -1){
						id = reg_register.addArea(reg_vec);
					}
				} else{
					id = accessor.getValue(ijk);
					auto regs = reg_register.findRegionsById(id);
					if (std::find(regs.begin(), regs.end(), reg) == regs.end()){
						regs.push_back(reg);
						id = reg_register.findRegions(regs);
						if (id == -1){
							id = reg_register.addArea(regs);
						}
					} else {
						// No need for intervention, as this specific region
						// is already mapped to this voxel
						return -2;
					}
				}
				return id;
			}

			void setVoxelId(
				GridAccessorType & accessor,
				const int & idx_i,
				const int & idx_j,
				const int & idx_k,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register)
			{
				openvdb::Coord ijk(idx_i,idx_j,idx_k);
				int id = getAreaId(
					accessor,
					ijk,
					reg,
					reg_register);
				if (id == -2){
					return;
				}
				accessor.setValue(ijk, id);
			}

			void boxSemanticCore(
				openvdb::Int32Tree& tree, 
				const int& idx_i, 
				const openvdb::Coord& ijk_min, 
				const openvdb::Coord& ijk_max,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register)
			{
				GridAccessorType accessor(tree);

				for(int idx_j = ijk_min[1]; idx_j <= ijk_max[1]; ++idx_j)
				{
					for(int idx_k = ijk_min[2]; idx_k <= ijk_max[2]; ++idx_k)
					{
						setVoxelId(
							accessor,
							idx_i,
							idx_j,
							idx_k,
							reg,
							reg_register);
					}
				}
			}

			void sphereSemanticCore(
				openvdb::Int32Tree& tree,
				const int& idx_i,
				const openvdb::Coord& ijk_min,
				const openvdb::Coord& ijk_max,
				const float& radius,
				const openvdb::Vec3d& centre,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register)
			{
				GridAccessorType accessor(tree);
				
				for(int idx_j = ijk_min[1]; idx_j <= ijk_max[1]; ++idx_j)
				{
					for(int idx_k = ijk_min[2]; idx_k <= ijk_max[2]; ++idx_k)
					{
						openvdb::Coord ijk(idx_i,idx_j,idx_k);
						openvdb::Vec3d current_point = initial_transformation_->indexToWorld(ijk);
						float distance = computeDistance(centre,current_point);
						if(distance <= radius)
						{	
							setVoxelId(
								accessor,
								idx_i,
								idx_j,
								idx_k,
								reg,
								reg_register);
						}
					}
				}
			}

			void coneSemanticCore(
				openvdb::Int32Tree& tree,
				const int & i,
				int & j,
				int & k,
				const double & theta,
				const openvdb::Vec3d & idx_space_origin,
				const openvdb::math::Quatd & rotation_quat,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register)
			{
				GridAccessorType accessor(tree);

				openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i,0,0));
				int j_start = -(static_cast<int>(std::tan(theta)*x.length()/voxel_size_));
				int k_start = -(static_cast<int>(std::tan(theta)*x.length()/voxel_size_));
				
				for(j = j_start; j <= -j_start; ++j){
					for(k = k_start; k <= -k_start; ++k)
					{
						openvdb::Vec3d yz = grid_->indexToWorld(openvdb::Coord(0.0,j,k));
						float distance = computeDistance(openvdb::Vec3d(0.0,0.0,0.0),yz);
						if(distance <= (std::tan(theta)*x.length()))
						{
							openvdb::Vec3d current_point(i,j,k);
							current_point = rotation_quat.rotateVector(current_point);
							current_point = current_point + idx_space_origin;

							int idx_i = static_cast<int>(std::round(current_point[0]));
							int idx_j = static_cast<int>(std::round(current_point[1]));
							int idx_k = static_cast<int>(std::round(current_point[2]));

							setVoxelId(
								accessor,
								idx_i,
								idx_j,
								idx_k,
								reg,
								reg_register);
						}
					}
				}
			}

			void pyramidSemanticCore(
				openvdb::Int32Tree& tree,
				const int & i,
				int & j, 
				int & k,
				const double & theta_h,
				const double & theta_v,
				const openvdb::Vec3d & idx_space_origin,
				const openvdb::math::Quatd & rotation_quat,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register)
			{
				GridAccessorType accessor(tree);
				
				openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i,0,0));
				int j_start = -(static_cast<int>(std::tan(theta_h)*x.length()/voxel_size_));
				int k_start = -(static_cast<int>(std::tan(theta_v)*x.length()/voxel_size_));

				for (j = j_start; j <= -j_start; ++j){
					for (k = k_start; k <= -k_start; ++k)
					{
						openvdb::Vec3d y = grid_->indexToWorld(openvdb::Coord(0.0, j, 0.0));
						float y_distance = computeDistance(openvdb::Vec3d(0.0, 0.0, 0.0), y);

						openvdb::Vec3d z = grid_->indexToWorld(openvdb::Coord(0.0, 0.0, k));
						float z_distance = computeDistance(openvdb::Vec3d(0.0, 0.0, 0.0), z);

						if (y_distance <= (std::tan(theta_h) * x.length()) && z_distance <= (std::tan(theta_v) * x.length()))
						{
							openvdb::Vec3d current_point(i, j, k);
							current_point = rotation_quat.rotateVector(current_point);
							current_point = current_point + idx_space_origin;

							int idx_i = static_cast<int>(std::round(current_point[0]));
							int idx_j = static_cast<int>(std::round(current_point[1]));
							int idx_k = static_cast<int>(std::round(current_point[2]));

							setVoxelId(
								accessor,
								idx_i,
								idx_j,
								idx_k,
								reg,
								reg_register);
						}
					}
				}
			}

		public:

			SemanticMapHandler(
				const bool& threaded,
				const float& voxel_size,
				const bool& vertex_centered = true):
			threaded_(threaded),
			voxel_size_(voxel_size),
			vertex_centered_(vertex_centered)
			{
				grid_ = openvdb::Int32Grid::create();

				if(vertex_centered){
					offset_ = openvdb::math::Vec3d(0.0,0.0,0.0);
				}
				else{
					offset_ = openvdb::math::Vec3d(voxel_size_/2.0,voxel_size_/2.0,voxel_size_/2.0);
				}

				initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
				initial_transformation_->postTranslate(offset_);

				grid_->setTransform(initial_transformation_);
			}

			SemanticMapHandler(
				std::shared_ptr<openvdb::Int32Grid> grid,
				const bool& threaded,
				const float& voxel_size,
				const bool& vertex_centered = true):
			grid_(grid),
			threaded_(threaded),
			voxel_size_(voxel_size),
			vertex_centered_(vertex_centered)
			{
				if(vertex_centered){
					offset_ = openvdb::math::Vec3d(0.0,0.0,0.0);
				}
				else{
					offset_ = openvdb::math::Vec3d(voxel_size_/2.0,voxel_size_/2.0,voxel_size_/2.0);
				}
				initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
				initial_transformation_->postTranslate(offset_);

				grid_->setTransform(initial_transformation_);		
			}

			~SemanticMapHandler(){
				grid_.reset();
			}

			void insertSemanticSphere(
				const float& radius,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register,
				const openvdb::Vec3d& centre = openvdb::Vec3d(0.0,0.0,0.0))
			{
				if (radius <= 0.0)
					std::cerr << "[AddSphere]: The radius cannot be less than or equal to zero! Aborting!" << std::endl;
				else
				{

					openvdb::Vec3d xyz_min_value_ws = centre - radius;
					openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
					openvdb::Coord ijk_min(static_cast<int>(xyz_min_value_is[0]),static_cast<int>(xyz_min_value_is[1]),static_cast<int>(xyz_min_value_is[2]));

					openvdb::Vec3d xyz_max_value = centre + radius;
					openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
					openvdb::Coord ijk_max(static_cast<int>(xyz_max_value_is[0]),static_cast<int>(xyz_max_value_is[1]),static_cast<int>(xyz_max_value_is[2]));
					
					tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(ijk_min[0],ijk_max[0],sizeof(ijk_min[0]));

					auto kernel = [&](const tbb::blocked_range<int>& iteration_range)
					{
						int idx_i = 0;
						openvdb::Int32Tree &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for(idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							sphereSemanticCore(
								tree,
								idx_i,
								ijk_min,
								ijk_max,
								radius,
								centre,
								reg,
								reg_register);
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<openvdb::Int32Tree>::iterator>;
						struct Op {
				            const bool mDelete;
				            openvdb::Int32Tree *mTree;
				            Op(openvdb::Int32Tree &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new openvdb::Int32Tree(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(openvdb::Int32Tree &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
						tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			void insertSemanticBox(
				const float& amplitude_h,
				const float& amplitude_v,
				const float& length,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register,
				const openvdb::Vec3d& centre = openvdb::Vec3d(0.0, 0.0, 0.0)
				)
			{
				if(amplitude_h < 0 || amplitude_v < 0 || length < 0){
					std::cerr << "[AddBox]: The Box length/amplitude cannot be less than zero! Aborting! Aborting!" << std::endl;
				} else {

					// Compute the maximum and minimum value of i,j, and k
					openvdb::Vec3d half_sizes = 0.5*openvdb::Vec3d(length, amplitude_h, amplitude_v);

					openvdb::Vec3d xyz_min_value_ws = centre - half_sizes;
					openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
					openvdb::Coord ijk_min(
						static_cast<int>(xyz_min_value_is[0]),
						static_cast<int>(xyz_min_value_is[1]),
						static_cast<int>(xyz_min_value_is[2]));

					openvdb::Vec3d xyz_max_value = centre + half_sizes;
					openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
					openvdb::Coord ijk_max(
						static_cast<int>(xyz_max_value_is[0]),
						static_cast<int>(xyz_max_value_is[1]),
						static_cast<int>(xyz_max_value_is[2]));

					tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(ijk_min[0],ijk_max[0],sizeof(ijk_max[0]));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						openvdb::Int32Tree &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							boxSemanticCore(
								tree,
								idx_i,
								ijk_min,
								ijk_max,
								reg,
								reg_register);
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<openvdb::Int32Tree>::iterator>;
						struct Op {
				            const bool mDelete;
				            openvdb::Int32Tree *mTree;
				            Op(openvdb::Int32Tree &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new openvdb::Int32Tree(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(openvdb::Int32Tree &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
				        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			void performIteration(
						tbb::enumerable_thread_specific<openvdb::Int32Tree> & tbb_thread_pool,
						const tbb::blocked_range<int> & tbb_iteration_range,
						std::function<void(const tbb::blocked_range<int>&)> kernel,
						const bool & threaded)
			{
				if(threaded_){
					tbb::parallel_for(tbb_iteration_range, kernel);
					using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<openvdb::Int32Tree>::iterator>;
					struct Op {
			            const bool mDelete;
			            openvdb::Int32Tree *mTree;
			            Op(openvdb::Int32Tree &tree) : mDelete(false), mTree(&tree) {}
			            Op(const Op& other, tbb::split) : mDelete(true), mTree(new openvdb::Int32Tree(other.mTree->background())){}
			            ~Op() { if (mDelete) delete mTree; }
			            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
			            void join(Op &other) { this->merge(*(other.mTree)); }
			            void merge(openvdb::Int32Tree &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
			        } op(grid_->tree());
			        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
				}
				else
					kernel(tbb_iteration_range);
			}

			void insertSemanticCone(
				const float & amplitude,
				const float & length,
				const openvdb::Vec3d & direction,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register,
				const openvdb::Vec3d& origin = openvdb::Vec3d(0.0, 0.0, 0.0)
				)
			{
				if (length <= 0.0){
					std::cerr << "[AddCone]: The cone length cannot be less than or equal to zero! Aborting!" << std::endl;
				}
				else {
					float theta = amplitude / 2.0;
					int i_end = std::ceil(length/voxel_size_);

					openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

					openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);
					openvdb::Vec3d rotation_axis = unit_vector_x.cross(direction);
					double rotation_angle = std::atan2(rotation_axis.length(), unit_vector_x.dot(direction));
					rotation_axis.normalize();
					openvdb::math::Quatd rotation_quat(rotation_axis, rotation_angle);

					tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(0, i_end, sizeof(i_end));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						int idx_j = 0;
						int idx_k = 0;
						openvdb::Int32Tree &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							coneSemanticCore(
								tree,
								idx_i,
								idx_j,
								idx_k,
								theta, 
								idx_space_origin,
								rotation_quat,
								reg,
								reg_register);
					};

					performIteration(tbb_thread_pool, tbb_iteration_range, kernel, threaded_);
				}
			}

			void insertSemanticPyramid(
				const float & amplitude_h,
				const float & amplitude_v,
				const float & length,
				const openvdb::Vec3d & direction,
				const std::string & reg,
				representation_plugins::RegionsRegister & reg_register,
				const openvdb::Vec3d & origin = openvdb::Vec3d(0.0, 0.0, 0.0))
			{
				if (length <= 0.0)
					std::cerr << "[AddPyramid]: The cone length cannot be less than or equal to zero! Aborting!" << std::endl;
				else
				{
					float theta_h = amplitude_h/2.0;
					float theta_v = amplitude_v/2.0;
					int i_end = std::ceil(length/voxel_size_);

					openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

					openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);
					openvdb::Vec3d rotation_axis = unit_vector_x.cross(direction);
					double rotation_angle = std::atan2(rotation_axis.length(), unit_vector_x.dot(direction));
					rotation_axis.normalize();
					openvdb::math::Quatd rotation_quat(rotation_axis, rotation_angle);

					tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(0, i_end, sizeof(i_end));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						int idx_j = 0;
						int idx_k = 0;
						openvdb::Int32Tree &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i){
							pyramidSemanticCore(
								tree,
								idx_i,
								idx_j,
								idx_k,
								theta_h,
								theta_v,
								idx_space_origin,
								rotation_quat,
								reg,
								reg_register);
						}
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<openvdb::Int32Tree>::iterator>;
						struct Op {
				            const bool mDelete;
				            openvdb::Int32Tree *mTree;
				            Op(openvdb::Int32Tree &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new openvdb::Int32Tree(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(openvdb::Int32Tree &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
				        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			bool removeRegion(const std::string & reg,
							  representation_plugins::RegionsRegister & reg_register){
				using ValueIter = typename openvdb::Int32Grid::ValueOnIter;

				// First we find the ID associated with the region-only area
				std::vector<std::string> reg_area = {reg};
				auto reg_id = reg_register.findRegions(reg_area);
				auto ids_to_udpate = reg_register.removeRegion(reg);
				if ((reg_id == -1) && (ids_to_udpate.size() == 0)){
					return false;
				}

			    struct IdUpdater {
			        int reg_id_;
			        std::map<int, int> ids_to_update_;
			        IdUpdater(
			        	const int & reg_id, 
			        	const std::map<int, int> & ids_to_udpate):
			        reg_id_(reg_id), ids_to_update_(ids_to_udpate) {}
			        inline void operator()(const ValueIter& iter) const {
			        	if (!iter.isVoxelValue())
			        		return;
			        	if (*iter == reg_id_){
			        		iter.setValueOff();
			        		return;
			        	}
			        	auto ids_it = ids_to_update_.find(*iter);
			        	if (ids_it == ids_to_update_.end()){
			        		return;
			        	}
			        	iter.setValue(ids_it->second);
			        }
			    };
				// As we are directly modifying the topology
				// of the tree associated with the grid,
				// it is not possible to have a multithreaded
				// foreach
			    openvdb::tools::foreach(grid_->beginValueOn(),
				        IdUpdater(reg_id, ids_to_udpate), false);

			    return true;
			}

			void clear()
			{
				grid_.reset();
				grid_ = openvdb::Int32Grid::create();
				grid_->setTransform(initial_transformation_);
			}

			void setFixedFrame(const std::string fixed_frame);
			void setMapFrame(const std::string map_frame);
			void rotateMap(); // perform the grid rotation from map frame to fixed frame
			void flushFoV(); // removes everything inside the FoV; to be performed as a TopologyDifference

			std::shared_ptr<openvdb::Int32Grid> getGridPtr()
			{
				return grid_;
			}

			void printHistogram(const int & max_id){
				auto histogram = openvdb::tools::histogram(
					grid_->beginValueOn(),
					0.0,
					static_cast<double>(max_id+1),
					10,
					false);
				histogram.print();
			}
	};
}  // namespace map_handler

#endif