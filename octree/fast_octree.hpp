#pragma once
#include "fast_octree.h"
#include <pcl/common/transforms.h>
#include <cmath>
#include <iostream>

namespace Ddx
{
	namespace octree
	{
		template <typename PointT>
		bool FastOctree<PointT>::SetMatrixData(const PointCloudConstPtr& data, const OctreeParams& params)
		{
			// set inpute parameter
			input_cloud_ = data;
			params_ = params;
			successors_.clear();
			leaf_id_ = 0;

			const uint32_t N = data->size();
			successors_ = std::vector<uint32_t>(N, 0);

			// initial min and max
			Eigen::Vector3f Bmin;
			Eigen::Vector3f Bmax;
			Bmin = data->points[0].getVector3fMap();
			Bmax = Bmin;
			successors_[0] = 1;

			// compute bounding box
			for (uint32_t i = 1; i < N; ++i)
			{
				successors_[i] = i + 1;
				const PointT& p = input_cloud_->points[i];

				if (p.x < Bmin[0]) Bmin[0] = p.x;
				if (p.y < Bmin[1]) Bmin[1] = p.y;
				if (p.z < Bmin[2]) Bmin[2] = p.z;
				if (p.x > Bmax[0]) Bmax[0] = p.x;
				if (p.y > Bmax[1]) Bmax[1] = p.y;
				if (p.z > Bmax[2]) Bmax[2] = p.z;
			}

			Eigen::Vector3f bounding_box_size = (Bmax - Bmin) * 0.5f;
			float max_extent = bounding_box_size.maxCoeff();
			if (true)
			{
				float r111x = max_extent / params_.minExtent;
				float r122x = std::log2f(r111x);
				float r133x = std::ceil(r122x);
				max_extent = params_.minExtent * std::pow(2.f, r133x);
			}
			Eigen::Vector3f center_point = Bmin + bounding_box_size;

			root_ = createOctant(center_point, max_extent, 0, N - 1, N);

			//std::cout << " leaf node : " << leaf_node_.size() << std::endl;
			leaf_node_.shrink_to_fit();
			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::SetMatrixData(const PointCloudConstPtr& data, const std::vector<uint32_t>& indexes, const OctreeParams& params)
		{
			// set inpute parameter
			input_cloud_ = data;
			params_ = params;
			successors_.clear();
			leaf_id_ = 0;

			const uint32_t N = data->size();
			successors_ = std::vector<uint32_t>(N);

			if (indexes.size() == 0) return false;

			// determine axis-aligned bounding box.
			uint32_t lastIdx = indexes[0];
			Eigen::Vector3f Bmin;
			Eigen::Vector3f Bmax;
			Bmin = data->points[lastIdx].getVector3fMap();
			Bmax = Bmin;

			for (uint32_t i = 1; i < indexes.size(); ++i)
			{
				uint32_t idx = indexes[i];
				// initially each element links simply to the following element.
				successors_[lastIdx] = idx;

				const PointT& p = input_cloud_->points[idx];

				if (p.x < Bmin[0]) Bmin[0] = p.x;
				if (p.y < Bmin[1]) Bmin[1] = p.y;
				if (p.z < Bmin[2]) Bmin[2] = p.z;
				if (p.x > Bmax[0]) Bmax[0] = p.x;
				if (p.y > Bmax[1]) Bmax[1] = p.y;
				if (p.z > Bmax[2]) Bmax[2] = p.z;

				lastIdx = idx;
			}

			Eigen::Vector3f bounding_box_size = (Bmax - Bmin) * 0.5f;
			float max_extent = bounding_box_size.maxCoeff();
			if (true)
			{
				float r111x = max_extent / params_.minExtent;
				float r122x = std::log2f(r111x);
				float r133x = std::ceil(r122x);
				max_extent = params_.minExtent * std::pow(2.f, r133x);
			}
			Eigen::Vector3f center_point = Bmin + bounding_box_size;

			root_ = createOctant(center_point, max_extent, 0, N - 1, N);

			leaf_node_.shrink_to_fit();
			return true;
		}

		template <typename PointT>
		OctreeNodePtr FastOctree<PointT>::getRoot()
		{
			return root_;
		}

		template <typename PointT>
		double FastOctree<PointT>::getResolution()
		{
			return params_.minExtent * 2.0;
		}

		template <typename PointT>
		OctreeParams FastOctree<PointT>::getOctreeParams()
		{
			return params_;
		}

		template <typename PointT>
		size_t FastOctree<PointT>::getLeafNumber()
		{
			return leaf_node_.size();
		}

		template <typename PointT>
		bool FastOctree<PointT>::getLeafNodeCenters(std::vector<Eigen::Vector3f>& center)
		{
			center.reserve(leaf_node_.size());
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				if (leaf_node_[i]->size_ == 0)
					continue;

				center.push_back(leaf_node_[i]->center_);
			}
			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::getLeafNodeCenters2(std::vector<Eigen::Vector3f>& center)
		{
			visit(root_, center);
			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::getMedoidCenters(std::vector<int>& indices, bool multi_thre)
		{
			if (multi_thre) return _getMedoidCentersMulti(indices);

			indices.resize(leaf_node_.size());
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				indices[i] = _getMedoidCenters(leaf_node_[i]);
			}
			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::getMedoidCenters(std::vector<int>& indices, std::map<uint32_t, int>& leaf_id_mapping, bool multi_thre)
		{
			leaf_id_mapping.clear();
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				leaf_id_mapping.emplace(leaf_node_[i]->id_, i);
			}

			if (multi_thre) return _getMedoidCentersMulti(indices);

			indices.resize(leaf_node_.size());
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				indices[i] = _getMedoidCenters(leaf_node_[i]);
			}
			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::getLeafSampling(std::vector<std::vector<uint32_t>>& leaf_sampling)
		{
			size_t leaf_node_number = leaf_node_.size();
			if (leaf_node_number <= 0) return false;

			leaf_sampling.resize(leaf_node_number);

			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				int startIdx = leaf_node_[i]->start_;
				int endIdx = leaf_node_[i]->end_;
				int size = leaf_node_[i]->size_;
				leaf_sampling[i].resize(size);
				uint32_t idx = startIdx;
				for (int k = 0; k < size; ++k)
				{
					leaf_sampling[i][k] = idx;
					idx = successors_[idx];
				}
			}

			return true;
		}

		template <typename PointT>
		bool FastOctree<PointT>::getLeafSampling(std::vector<std::vector<uint32_t>>& leaf_sampling, std::map<uint32_t, int>& leaf_id_mapping)
		{
			size_t leaf_node_number = leaf_node_.size();
			if (leaf_node_number <= 0) return false;

			leaf_sampling.resize(leaf_node_number);
			leaf_id_mapping.clear();
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				leaf_id_mapping.emplace(std::make_pair(leaf_node_[i]->id_, i));
				int startIdx = leaf_node_[i]->start_;
				int endIdx = leaf_node_[i]->end_;
				int size = leaf_node_[i]->size_;
				leaf_sampling[i].resize(size);
				uint32_t idx = startIdx;
				for (int k = 0; k < size; ++k)
				{
					leaf_sampling[i][k] = idx;
					idx = successors_[idx];
				}
			}

			return true;
		}

		template <typename PointT>
		OctreeNodePtr FastOctree<PointT>::createOctant(Eigen::Vector3f const& center, float const& extent, uint32_t const& startIdx, uint32_t const& endIdx, uint32_t const& size)
		{
			const float& x = center[0];
			const float& y = center[1];
			const float& z = center[2];

			// For a leaf we don't have to change anything; points are already correctly linked or correctly reordered.
			OctreeNodePtr node(new OctreeNode);

			// initial node member variables
			node->isLeaf_ = true;
			node->center_[0] = x;
			node->center_[1] = y;
			node->center_[2] = z;
			node->extent_ = extent;
			node->start_ = startIdx;
			node->end_ = endIdx;
			node->size_ = size;

			// subdivide subset of points and re-link points according to Morton codes
			if (size > params_.bucketSize && extent > params_.minExtent * 1.5)
			{
				// initial children node size
				node->children_.resize(8, nullptr);
				node->isLeaf_ = false;

				std::vector<uint32_t> child_starts(8, 0);
				std::vector<uint32_t> child_ends(8, 0);
				std::vector<uint32_t> child_sizes(8, 0);

				// re-link disjoint child subsets...
				uint32_t idx = startIdx;

				for (uint32_t i = 0; i < size; ++i)
				{
					const PointT& p = input_cloud_->points[idx];

					// determine Morton code for each point...
					uint32_t mortonCode = 0;
					if (p.x > x) mortonCode |= 1;
					if (p.y > y) mortonCode |= 2;
					if (p.z > z) mortonCode |= 4;

					// set child starts and update successors...
					if (child_sizes[mortonCode] == 0)
						child_starts[mortonCode] = idx;
					else
						successors_[child_ends[mortonCode]] = idx;
					child_sizes[mortonCode] += 1;

					child_ends[mortonCode] = idx;
					idx = successors_[idx];
				}


				// now, we can create the child nodes...
				float child_extent = 0.5f * extent;
				bool firsttime = true;
				uint32_t lastChildIdx = 0;
				for (uint32_t i = 0; i < 8; ++i)
				{
					if (child_sizes[i] == 0) continue;

					float childX = x + factor_[(i & 1) > 0] * extent;
					float childY = y + factor_[(i & 2) > 0] * extent;
					float childZ = z + factor_[(i & 4) > 0] * extent;

					node->children_[i] = createOctant(Eigen::Vector3f(childX, childY, childZ), child_extent, child_starts[i], child_ends[i], child_sizes[i]);

					if (firsttime)
						node->start_ = node->children_[i]->start_;
					else
						successors_[node->children_[lastChildIdx]->end_] = node->children_[i]->start_;  // we have to ensure that also the child ends link to the next child start.

					lastChildIdx = i;
					node->end_ = node->children_[i]->end_;
					firsttime = false;
				}
			}

			if (node->isLeaf_)
			{
				leaf_id_++;
				node->id_ = leaf_id_;
				leaf_node_.push_back(node);
			}

			return node;
		}

		template <typename PointT>
		bool FastOctree<PointT>::visit(OctreeNodePtr const& node, std::vector<Eigen::Vector3f>& center)
		{
			if (node == nullptr)
				return false;

			if (node->isLeaf_)
			{
				center.push_back(node->center_);
				return true;
			}

			for (int i = 0; i < node->children_.size(); ++i)
			{
				visit(node->children_[i], center);
			}

			return true;
		}

		template <typename PointT>
		std::vector<int> FastOctree<PointT>::_getLeafNodeSamples(OctreeNodePtr const& node)
		{
			std::vector<int> indices(node->size_);
			uint32_t idx = node->start_;
			for (uint32_t i = 0; i < node->size_; ++i)
			{
				indices[i] = idx;
				idx = successors_[idx];
			}

			return indices;
		}

		template <typename PointT>
		int FastOctree<PointT>::_getMedoidCenters(OctreeNodePtr const& node)
		{
			std::vector<int> indices = _getLeafNodeSamples(node);
			if (indices.size() == 1)
				return indices[0];
			Eigen::Vector4f xyz_centroid;
			// Estimate the XYZ centroid
			pcl::compute3DCentroid(*input_cloud_, indices, xyz_centroid);

			float min_distance = std::numeric_limits<float>::max();
			int min_id = -1;
			for (int i = 0; i < indices.size(); ++i)
			{
				float d = (xyz_centroid - input_cloud_->points[indices[i]].getVector4fMap()).squaredNorm();
				if (d < min_distance)
				{
					min_distance = d;
					min_id = indices[i];
				}
			}

			return min_id;
		}

		template <typename PointT>
		bool FastOctree<PointT>::_getMedoidCentersMulti(std::vector<int>& indices)
		{
			indices.resize(leaf_node_.size());
#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int i = 0; i < leaf_node_.size(); ++i)
			{
				indices[i] = _getMedoidCenters(leaf_node_[i]);
			}
			return true;
		}

	}
}