#ifndef MAP_HANLDER__MAP_HANDLER_HPP
#define MAP_HANDLER__MAP_HANDLER_HPP

/* -------------------------------------------------------------------------- */
/*                                Header Files                                */
/* -------------------------------------------------------------------------- */
#include <iostream>
#include <cmath>
#include <type_traits> 
#include <algorithm>

#include <openvdb/openvdb.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "map_handler/fov.hpp"

#include <chrono>
#include <ctime> 
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                  Namespace                                 */
/* -------------------------------------------------------------------------- */
namespace map_handler
{
	/* -------------------------------------------------------------------------- */
	/*                              Class Definition                              */
	/* -------------------------------------------------------------------------- */
	template<class GridT>
	class MapHandler
	{
		/* --------------------------------- Private -------------------------------- */
		private:
			/* ------------------------------- Attributes ------------------------------- */
			bool threaded_;

			std::shared_ptr<GridT> grid_;

			float voxel_size_;
			bool vertex_centered_;
			openvdb::math::Vec3d offset_;
			openvdb::math::Transform::Ptr initial_transformation_;	

			// std::shared_ptr<fov::FOV> fov_{nullptr}; // yet to be implemented

			std::string fixed_frame_;
			std::string map_frame_; // If these two are the same, then no rotation applied to the map
			/* -------------------------------------------------------------------------- */

			/* -------------------------------- Utilities ------------------------------- */
			static float computeDistance(const openvdb::Vec3d& a, const openvdb::Vec3d& b)
			{
				return std::sqrt(std::pow(a[0] - b[0],2) + std::pow(a[1] - b[1],2) + std::pow(a[2] - b[2],2));
			}

			static bool compareVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
			{
					return a[0] < b[0];
			}

			void createFromPointCloud(const sensor_msgs::msg::PointCloud2& pointCloud2_msg)
			{
				auto start = std::chrono::system_clock::now();
				using GridAccessorType = typename GridT::Accessor;
				using CurrentTreeType = typename GridT::TreeType;

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

				pcl::fromROSMsg(pointCloud2_msg, *cloud);

				pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approx_voxel_grid;
				approx_voxel_grid.setInputCloud(cloud);
				approx_voxel_grid.setLeafSize(voxel_size_,voxel_size_,voxel_size_);
				approx_voxel_grid.filter(*cloud_filtered);

				// sensor_msgs::msg::PointCloud2 cloud_filtered_msg;
  				// pcl::toROSMsg(*cloud_filtered,cloud_filtered_msg);
				// sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(cloud_filtered_msg, "x");
				// sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(cloud_filtered_msg, "y");
				// sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(cloud_filtered_msg, "z");

				// std::vector<Eigen::Vector3d> points;

				// for(size_t i = 0; i < (cloud_filtered->height * cloud_filtered->width); ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
				// {
				//    // Reject NAN points
				//   if (std::isnan(*ros_pc2_x) || std::isnan(*ros_pc2_y) || std::isnan(*ros_pc2_z))
				//     continue;
				//   else
				//     points.push_back(Eigen::Vector3d(*ros_pc2_x,*ros_pc2_y,*ros_pc2_z));
				// }

				// Sort the vector of Eigen vectors based on the Euclidean norm
    			// std::sort(points.begin(), points.end(), compareVectors);
				// std::vector<std::vector<Eigen::Vector3d>> points_blocks;

				// points_blocks.push_back(std::vector<Eigen::Vector3d>());
				// points_blocks[0].push_back(points[0]);

				// int idx = 0;
				// for(int i = 1; i < static_cast<int>(points.size()); i++)
				// {
				// 	if(points[i-1][0] != points[i][0])
				// 	{
				// 		idx++;
				// 		points_blocks.push_back(std::vector<Eigen::Vector3d>());
				// 	}

				// 	points_blocks[idx].push_back(points[i]);
				// }

				tbb::enumerable_thread_specific<CurrentTreeType> tbb_thread_pool(grid_->tree());
				tbb::blocked_range<std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>::iterator> tbb_iteration_range(cloud_filtered->points.begin(),cloud_filtered->points.end(),sizeof(cloud_filtered->points.begin()));

				auto kernel = [&](const tbb::blocked_range<std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>::iterator> &iteration_range)
				{
					CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();
					GridAccessorType accessor(tree);
					int value = 0;
					for(auto it_ = iteration_range.begin(); it_ != iteration_range.end(); ++it_)
					{
						++value;
						openvdb::Vec3d coordinates = grid_->worldToIndex(openvdb::Vec3d(it_->x,it_->y,it_->z));
						openvdb::Coord ijk(static_cast<int>(round(coordinates[0])), static_cast<int>(round(coordinates[1])), static_cast<int>(round(coordinates[2])));
						accessor.setValue(ijk, 1);
					}
					std::cout << value << std::endl;
				};

				// tbb::blocked_range<int> tbb_iteration_range(0,static_cast<int>(points_blocks.size()));

				// auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
				// {
				// 	int idx_x = 0;

				// 	CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();
				// 	GridAccessorType accessor(tree);

				// 	int value = 0;

				// 	for(idx_x = iteration_range.begin(); idx_x != iteration_range.end(); ++idx_x)
				// 	{
						
				// 		++value;
				// 		for(int j = 0; j <= static_cast<int>(points_blocks[idx_x].size()); ++j)
				// 		{
				// 			openvdb::Vec3d coordinates = grid_->worldToIndex(openvdb::Vec3d(points_blocks[idx_x][j][0],points_blocks[idx_x][j][1],points_blocks[idx_x][j][2]));
				// 			openvdb::Coord ijk(static_cast<int>(round(coordinates[0])), static_cast<int>(round(coordinates[1])), static_cast<int>(round(coordinates[2])));
				// 			accessor.setValue(ijk, 1);
				// 		}
				// 	}
				// 	std::cout << value << std::endl;
				// };

				if(threaded_)
				{
					tbb::parallel_for(tbb_iteration_range,kernel);
					using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<CurrentTreeType>::iterator>;
					struct Op {
				        const bool mDelete;
				        CurrentTreeType *mTree;
				        Op(CurrentTreeType &tree) : mDelete(false), mTree(&tree) {}
				        Op(const Op& other, tbb::split) : mDelete(true), mTree(new CurrentTreeType(other.mTree->background())){}
				        ~Op() { if (mDelete) delete mTree; }
				        void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				        void join(Op &other) { this->merge(*(other.mTree)); }
				        void merge(CurrentTreeType &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				    } op(grid_->tree());
				    tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
				}
				else
					kernel(tbb_iteration_range);

				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> elapsed_seconds = end-start;
				std::cout << "elapsed time: " << elapsed_seconds.count() << "s" << std::endl;
			}
			/* -------------------------------------------------------------------------- */

			/* ---------------------------------- Cores --------------------------------- */
			template<class TreeT, class U>
			void coneCore(TreeT& tree, const int& i, int& j, int& k, const double& theta, const openvdb::Vec3d& idx_space_origin, const openvdb::math::Quatd& rotation_quat, const U& intensity)
			{
				using GridAccessorType = typename openvdb::Grid<TreeT>::Accessor;
				GridAccessorType accessor(tree);

				openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i,0,0));
				int j_start = -(static_cast<int>(std::tan(theta)*x.length()/voxel_size_));
				int k_start = -(static_cast<int>(std::tan(theta)*x.length()/voxel_size_));
				
				for(j = j_start; j <= -j_start; ++j)
					for(k = k_start; k <= -k_start; ++k)
					{
						openvdb::Vec3d yz = grid_->indexToWorld(openvdb::Coord(0.0,j,k));
						float distance = computeDistance(openvdb::Vec3d(0.0,0.0,0.0),yz);
						if(distance <= (std::tan(theta)*x.length()))
						{
							openvdb::Vec3d current_point(i,j,k);
							current_point = rotation_quat.rotateVector(current_point);
							current_point = current_point + idx_space_origin;

							openvdb::Coord ijk (static_cast<int>(std::round(current_point[0])),static_cast<int>(std::round(current_point[1])),static_cast<int>(std::round(current_point[2])));

							if constexpr(std::is_invocable_v<U,const openvdb::Vec3d&>)
								accessor.setValue(ijk,std::invoke(intensity,grid_->indexToWorld(ijk)));
							else
								accessor.setValue(ijk,intensity);
						}
					}
			}

			template<class TreeT, class U>
			void sphereCore(TreeT& tree, const int& idx_i, const openvdb::Coord& ijk_min, const openvdb::Coord& ijk_max, const float& radius, const openvdb::Vec3d& centre, const U& intensity)
			{
				using GridAccessorType = typename openvdb::Grid<TreeT>::Accessor;
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
							if constexpr(std::is_invocable_v<U,const openvdb::Vec3d&>)
								accessor.setValue(ijk,std::invoke(intensity,current_point));
							else
								accessor.setValue(ijk,intensity);
						}
					}
				}
			}
			
			template<class TreeT, class U>
			void pyramidCore(TreeT& tree, const int& i, int& j, int& k, const double& theta_h, const double& theta_v, const openvdb::Vec3d& idx_space_origin, const openvdb::math::Quatd& rotation_quat, const U& intensity)
			{
				using GridAccessorType = typename openvdb::Grid<TreeT>::Accessor;
				GridAccessorType accessor(tree);

				openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i,0,0));
				int j_start = -(static_cast<int>(std::tan(theta_h)*x.length()/voxel_size_));
				int k_start = -(static_cast<int>(std::tan(theta_v)*x.length()/voxel_size_));

				for (j = j_start; j <= -j_start; ++j)
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

							openvdb::Coord ijk (static_cast<int>(std::round(current_point[0])),static_cast<int>(std::round(current_point[1])),static_cast<int>(std::round(current_point[2])));

							if constexpr(std::is_invocable_v<U,const openvdb::Vec3d&>)
								accessor.setValue(ijk,std::invoke(intensity,grid_->indexToWorld(ijk)));
							else
								accessor.setValue(ijk,intensity);
						}
					}
			}

			template<class TreeT, class U>
			void boxCore(TreeT& tree, const int& idx_i, const openvdb::Coord& ijk_min, const openvdb::Coord& ijk_max, const U& intensity)
			{
				using GridAccessorType = typename openvdb::Grid<TreeT>::Accessor;
				GridAccessorType accessor(tree);

				for(int idx_j = ijk_min[1]; idx_j <= ijk_max[1]; ++idx_j)
					for(int idx_k = ijk_min[2]; idx_k <= ijk_max[2]; ++idx_k)
					{
						openvdb::Coord ijk(idx_i,idx_j,idx_k);

						if constexpr(std::is_invocable_v<U,const openvdb::Vec3d&>)
								accessor.setValue(ijk,std::invoke(intensity,grid_->indexToWorld(ijk)));
							else
								accessor.setValue(ijk,intensity);
					}
						
			}
		/* -------------------------------------------------------------------------- */

		/* --------------------------------- Public --------------------------------- */
		public:

			/* ------------------------------- Constructor ------------------------------ */
			MapHandler(const bool& threaded, const float& voxel_size, const bool& vertex_centered = true):threaded_(threaded),voxel_size_(voxel_size),vertex_centered_(vertex_centered)
			{
				constexpr bool is_base = std::is_base_of_v<openvdb::GridBase,GridT>;
				constexpr bool is_same = std::is_same_v<openvdb::GridBase,GridT>;

				if constexpr(!is_base)
				{
					std::cerr << "[MapHandler]: You are trying to operate on an object not derived from GridBase! Aborting!" << std::endl;
					return;
				}

				if constexpr(is_same)
				{
					std::cerr << "[MapHandler]: You are trying to operate on a GridBase object! Aborting!" << std::endl;
					return;
				}

				if(is_base && !is_same)
				{
					grid_ = GridT::create();

					if(vertex_centered)
					offset_ = openvdb::math::Vec3d(0.0,0.0,0.0);
					else
						offset_ = openvdb::math::Vec3d(voxel_size_/2.0,voxel_size_/2.0,voxel_size_/2.0);

					initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
					initial_transformation_->postTranslate(offset_);

					grid_->setTransform(initial_transformation_);
				}			
			}

			MapHandler(std::shared_ptr<GridT> grid, const bool& threaded, const float& voxel_size, const bool& vertex_centered = true):grid_(grid),threaded_(threaded),voxel_size_(voxel_size),vertex_centered_(vertex_centered)
			{
				constexpr bool is_base = std::is_base_of_v<openvdb::GridBase,GridT>;
				constexpr bool is_same = std::is_same_v<openvdb::GridBase,GridT>;

				if constexpr(!is_base)
				{
					std::cerr << "[MapHandler]: You are trying to operate on an object not derived from GridBase! Aborting!" << std::endl;
					return;
				}

				if constexpr(is_same)
				{
					std::cerr << "[MapHandler]: You are trying to operate on a GridBase object! Aborting!" << std::endl;
					return;
				}

				if(is_base && !is_same)
				{
					if(vertex_centered)
					offset_ = openvdb::math::Vec3d(0.0,0.0,0.0);
					else
						offset_ = openvdb::math::Vec3d(voxel_size_/2.0,voxel_size_/2.0,voxel_size_/2.0);

					initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
					initial_transformation_->postTranslate(offset_);

					grid_->setTransform(initial_transformation_);
				}			
			}
			/* -------------------------------------------------------------------------- */

			/* ------------------------------- Destructor ------------------------------- */
			~MapHandler(){}
			/* -------------------------------------------------------------------------- */
			
			/* --------------------------------- Insert --------------------------------- */
			template <class U>
			void insertCone(const float& amplitude, const float& length, const openvdb::Vec3d& direction, const U &intensity, const openvdb::Vec3d& origin = openvdb::Vec3d(0.0, 0.0, 0.0))
			{
				if (length <= 0.0)
					std::cerr << "[AddCone]: The cone length cannot be less than or equal to zero! Aborting!" << std::endl;
				else
				{
					using CurrentTreeType = typename GridT::TreeType;

					float theta = amplitude / 2.0;
					int i_end = std::ceil(length/voxel_size_);

					openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

					openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);
					openvdb::Vec3d rotation_axis = unit_vector_x.cross(direction);
					double rotation_angle = std::atan2(rotation_axis.length(), unit_vector_x.dot(direction));
					rotation_axis.normalize();
					openvdb::math::Quatd rotation_quat(rotation_axis, rotation_angle);

					tbb::enumerable_thread_specific<CurrentTreeType> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(0, i_end, sizeof(i_end));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						int idx_j = 0;
						int idx_k = 0;
						CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							coneCore(tree,idx_i, idx_j, idx_k, theta, idx_space_origin, rotation_quat, intensity);
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<CurrentTreeType>::iterator>;
						struct Op {
				            const bool mDelete;
				            CurrentTreeType *mTree;
				            Op(CurrentTreeType &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new CurrentTreeType(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(CurrentTreeType &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
				        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			template <class U>
			void insertSphere(const float& radius, const U &intensity, const openvdb::Vec3d& centre = openvdb::Vec3d(0.0,0.0,0.0))
			{
				if (radius <= 0.0)
					std::cerr << "[AddSphere]: The radius cannot be less than or equal to zero! Aborting!" << std::endl;
				else
				{
					using CurrentTreeType = typename GridT::TreeType;

					/* ----------- Compute the maximum and minimum value of i,j, and k ---------- */
					openvdb::Vec3d xyz_min_value_ws = centre - radius;
					openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
					openvdb::Coord ijk_min(static_cast<int>(xyz_min_value_is[0]),static_cast<int>(xyz_min_value_is[1]),static_cast<int>(xyz_min_value_is[2]));

					openvdb::Vec3d xyz_max_value = centre + radius;
					openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
					openvdb::Coord ijk_max(static_cast<int>(xyz_max_value_is[0]),static_cast<int>(xyz_max_value_is[1]),static_cast<int>(xyz_max_value_is[2]));
					/* -------------------------------------------------------------------------- */

					tbb::enumerable_thread_specific<CurrentTreeType> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(ijk_min[0],ijk_max[0],sizeof(ijk_min[0]));

					auto kernel = [&](const tbb::blocked_range<int>& iteration_range)
					{
						int idx_i = 0;
						CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for(idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							sphereCore(tree,idx_i,ijk_min,ijk_max,radius,centre,intensity);
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<CurrentTreeType>::iterator>;
						struct Op {
				            const bool mDelete;
				            CurrentTreeType *mTree;
				            Op(CurrentTreeType &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new CurrentTreeType(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(CurrentTreeType &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
						tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			template <class U>
			void insertPyramid(const float& amplitude_h, const float& amplitude_v, const float& length, const openvdb::Vec3d& direction, const U& intensity, const openvdb::Vec3d& origin = openvdb::Vec3d(0.0, 0.0, 0.0))
			{
				if (length <= 0.0)
					std::cerr << "[AddPyramid]: The cone length cannot be less than or equal to zero! Aborting!" << std::endl;
				else
				{
					using CurrentTreeType = typename GridT::TreeType;

					float theta_h = amplitude_h/2.0;
					float theta_v = amplitude_v/2.0;
					int i_end = std::ceil(length/voxel_size_);

					openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

					openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);
					openvdb::Vec3d rotation_axis = unit_vector_x.cross(direction);
					double rotation_angle = std::atan2(rotation_axis.length(), unit_vector_x.dot(direction));
					rotation_axis.normalize();
					openvdb::math::Quatd rotation_quat(rotation_axis, rotation_angle);

					tbb::enumerable_thread_specific<CurrentTreeType> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(0, i_end, sizeof(i_end));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						int idx_j = 0;
						int idx_k = 0;
						CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							pyramidCore(tree, idx_i,  idx_j, idx_k, theta_h, theta_v, idx_space_origin, rotation_quat, intensity);
					};


					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<CurrentTreeType>::iterator>;
						struct Op {
				            const bool mDelete;
				            CurrentTreeType *mTree;
				            Op(CurrentTreeType &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new CurrentTreeType(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(CurrentTreeType &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
				        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}

			template <class U>
			void insertBox(const float& amplitude_h, const float& amplitude_v, const float& length, const U& intensity, const openvdb::Vec3d& centre = openvdb::Vec3d(0.0, 0.0, 0.0))
			{
				if(amplitude_h < 0 || amplitude_v < 0 || length < 0)
					std::cerr << "[AddBox]: The Box length/amplitude cannot be less than zero! Aborting! Aborting!" << std::endl;
				else
				{
					using CurrentTreeType = typename GridT::TreeType;

					/* ----------- Compute the maximum and minimum value of i,j, and k ---------- */
					openvdb::Vec3d half_sizes = 0.5*openvdb::Vec3d(length, amplitude_h, amplitude_v);

					openvdb::Vec3d xyz_min_value_ws = centre - half_sizes;
					openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
					openvdb::Coord ijk_min(static_cast<int>(xyz_min_value_is[0]),static_cast<int>(xyz_min_value_is[1]),static_cast<int>(xyz_min_value_is[2]));

					openvdb::Vec3d xyz_max_value = centre + half_sizes;
					openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
					openvdb::Coord ijk_max(static_cast<int>(xyz_max_value_is[0]),static_cast<int>(xyz_max_value_is[1]),static_cast<int>(xyz_max_value_is[2]));
					/* -------------------------------------------------------------------------- */

					tbb::enumerable_thread_specific<CurrentTreeType> tbb_thread_pool(grid_->tree());
					tbb::blocked_range<int> tbb_iteration_range(ijk_min[0],ijk_max[0],sizeof(ijk_max[0]));

					auto kernel = [&](const tbb::blocked_range<int> &iteration_range)
					{
						int idx_i = 0;
						CurrentTreeType &tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

						for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i)
							boxCore(tree, idx_i, ijk_min, ijk_max, intensity);
					};

					if(threaded_)
					{
						tbb::parallel_for(tbb_iteration_range,kernel);
						using RangeT = tbb::blocked_range<typename tbb::enumerable_thread_specific<CurrentTreeType>::iterator>;
						struct Op {
				            const bool mDelete;
				            CurrentTreeType *mTree;
				            Op(CurrentTreeType &tree) : mDelete(false), mTree(&tree) {}
				            Op(const Op& other, tbb::split) : mDelete(true), mTree(new CurrentTreeType(other.mTree->background())){}
				            ~Op() { if (mDelete) delete mTree; }
				            void operator()(const RangeT &r) { for (auto i=r.begin(); i!=r.end(); ++i) this->merge(*i);}
				            void join(Op &other) { this->merge(*(other.mTree)); }
				            void merge(CurrentTreeType &tree) { mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
				        } op(grid_->tree());
				        tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
					}
					else
						kernel(tbb_iteration_range);
				}
			}
			/* -------------------------------------------------------------------------- */

			


			/* ------------------------------- PointCloud ------------------------------- */
			void updateFromPointCloud(const sensor_msgs::msg::PointCloud2& pointCloud2_msg)
			{
				grid_.reset();
				grid_ = GridT::create();
				grid_->setTransform(initial_transformation_);
				createFromPointCloud(pointCloud2_msg);
			}
			/* -------------------------------------------------------------------------- */

			/* ----------------------------------- Get ---------------------------------- */
			std::shared_ptr<GridT> getGridPtr()
			{
				return grid_;
			}
			/* -------------------------------------------------------------------------- */

			
			void setFixedFrame(const std::string fixed_frame);
			void setMapFrame(const std::string map_frame);

			void rotateMap(); // perform the grid rotation from map frame to fixed frame

			void flushFoV(); // removes everything inside the FoV; to be performed as a TopologyDifference
		/* -------------------------------------------------------------------------- */
	};
	/* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */
#endif