#pragma once

#include "fast_octree.h"
#include "fast_octree.hpp"
#include "voxel_hash_medoid.hpp"

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <vector>

namespace Ddx
{
	namespace octree
	{
		/// 体素大小对应的 FastOctree 参数（仅供仍走 FastOctree 的调用方）。
		inline OctreeParams makeDownsampleParams(float voxel_size, std::uint32_t bucket_size = 1)
		{
			OctreeParams params;
			params.bucketSize = bucket_size;
			params.minExtent = voxel_size * 0.5f;
			return params;
		}

		/// 原点对齐整数体素格真 Medoid 下采样；voxel_size 为体素边长（米）。生产默认路径。
		template <typename PointT>
		bool downsampleMedoidIndices(typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
		                             float voxel_size, std::vector<int>& indices,
		                             bool multi_thread = true)
		{
			return downsampleHashMedoidIndices<PointT>(cloud, voxel_size, indices, multi_thread);
		}

		/// 原点对齐整数体素格：每格保留首个落入点（哈希单遍，快于 Medoid；适合求解前抽稀）。
		template <typename PointT>
		bool downsampleFirstIndices(typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
		                            float voxel_size, std::vector<int>& indices)
		{
			return downsampleHashFirstIndices<PointT>(cloud, voxel_size, indices);
		}

		/// FastOctree 叶节点真 Medoid 下采样（包围盒对齐、可变叶大小；与哈希格语义不完全相同）。
		template <typename PointT>
		bool downsampleOctreeMedoidIndices(typename pcl::PointCloud<PointT>::ConstPtr const& cloud,
		                                   float voxel_size, std::vector<int>& indices,
		                                   bool multi_thread = true)
		{
			indices.clear();
			if (!cloud || cloud->empty())
				return false;

			FastOctree<PointT> fast_octree;
			fast_octree.SetMatrixData(cloud, makeDownsampleParams(voxel_size));
			return fast_octree.getMedoidCenters(indices, multi_thread);
		}

		template <typename PointT>
		typename pcl::PointCloud<PointT>::Ptr downsampleMedoid(
		    typename pcl::PointCloud<PointT>::ConstPtr const& cloud, float voxel_size,
		    bool multi_thread = true)
		{
			auto result = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			std::vector<int> indices;
			if (!downsampleMedoidIndices<PointT>(cloud, voxel_size, indices, multi_thread))
				return result;

			pcl::copyPointCloud(*cloud, indices, *result);
			return result;
		}

		template <typename PointT>
		typename pcl::PointCloud<PointT>::Ptr downsampleMedoid(pcl::PointCloud<PointT> const& cloud,
		                                                       float voxel_size,
		                                                       bool multi_thread = true)
		{
			return downsampleMedoid<PointT>(cloud.makeShared(), voxel_size, multi_thread);
		}

		template <typename PointT>
		typename pcl::PointCloud<PointT>::Ptr downsampleOctreeMedoid(
		    typename pcl::PointCloud<PointT>::ConstPtr const& cloud, float voxel_size,
		    bool multi_thread = true)
		{
			auto result = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			std::vector<int> indices;
			if (!downsampleOctreeMedoidIndices<PointT>(cloud, voxel_size, indices, multi_thread))
				return result;

			pcl::copyPointCloud(*cloud, indices, *result);
			return result;
		}

		template <typename PointT>
		typename pcl::PointCloud<PointT>::Ptr downsampleOctreeMedoid(
		    pcl::PointCloud<PointT> const& cloud, float voxel_size, bool multi_thread = true)
		{
			return downsampleOctreeMedoid<PointT>(cloud.makeShared(), voxel_size, multi_thread);
		}
	}
}
