#pragma once

#include <Eigen/Core>
#include <map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

namespace Ddx
{
	namespace octree
	{
		struct OctreeParams
		{
		public:
			OctreeParams(uint32_t bucketSize = 32, float minExtent = 0.0f)
				: bucketSize(bucketSize), minExtent(minExtent)
			{
			}
			uint32_t bucketSize;			/* The minimum number of points in each leaf voxel */
			float minExtent;				/* The minimum node of an octree ,note : radius */
		};

		class Node
		{
		public:
			Node()
				: isLeaf_(true)
				, center_(Eigen::Vector3f::Zero())
				, extent_(0.f)
				, start_(0)
				, end_(0)
				, size_(0)
				, id_(0)
			{
			}

			virtual ~Node() {};

			typedef std::shared_ptr<Node> Ptr;
			typedef std::shared_ptr<const Node> ConstPtr;

			bool isLeaf_;

			// bounding box of the octant needed for overlap and contains tests...
			Eigen::Vector3f center_;
			float extent_;   // half of side-length
			uint32_t start_, end_;  // start and end in succ_
			uint32_t size_;        // number of points
			uint32_t id_;			// the ID is only valid when the current node is a leaf node

		};

		class OctreeNode : public Node
		{
		public:
			typedef std::shared_ptr<OctreeNode> Ptr;
			typedef std::shared_ptr<const OctreeNode> ConstPtr;

			OctreeNode() : Node() {}

			~OctreeNode() {}

		public:
			std::vector<Ptr> children_;

		};

		typedef OctreeNode::Ptr OctreeNodePtr;
		typedef OctreeNode::ConstPtr OctreeNodeConstPtr;

		template <typename PointT>
		class FastOctree
		{
		public:
			typedef pcl::PointCloud<PointT> PointCloud;
			typedef typename PointCloud::Ptr PointCloudPtr;
			typedef typename PointCloud::ConstPtr PointCloudConstPtr;

			typedef std::shared_ptr<FastOctree > Ptr;
			typedef std::shared_ptr<const FastOctree > ConstPtr;

		public:
			FastOctree() : root_(0), leaf_id_(0), input_cloud_(0), factor_(Eigen::Vector2f(-0.5f, 0.5f)) {};
			~FastOctree() {};


			bool SetMatrixData(const PointCloudConstPtr& data, const OctreeParams& params = OctreeParams());

			bool SetMatrixData(const PointCloudConstPtr& data, const std::vector<uint32_t>& indexes, const OctreeParams& params = OctreeParams());

			OctreeNodePtr getRoot();

			OctreeParams getOctreeParams();

			size_t getLeafNumber();

			double getResolution();

			bool getLeafNodeCenters(std::vector<Eigen::Vector3f>& center);

			bool getLeafNodeCenters2(std::vector<Eigen::Vector3f>& center);

			bool getMedoidCenters(std::vector<int>& indices, bool multi_thre = false);

			bool getMedoidCenters(std::vector<int>& indices, std::map<uint32_t, int>& leaf_id_mapping, bool multi_thre = false);

			bool getLeafSampling(std::vector<std::vector<uint32_t>>& leaf_sampling);

			bool getLeafSampling(std::vector<std::vector<uint32_t>>& leaf_sampling, std::map<uint32_t, int>& leaf_id_mapping);

		protected:
			/**
			 * \brief creation of an octant using the elements starting at startIdx.
			 *
			 * The method reorders the index such that all points are correctly linked to successors belonging
			 * to the same octant.
			 *
			 * \param  center(x,y,z)  center coordinates of octant
			 * \param extent          extent of octant
			 * \param startIdx        first index of points inside octant
			 * \param endIdx          last index of points inside octant
			 * \param size            number of points in octant
			 *
			 * \return  octant with children nodes.
			*/
			OctreeNodePtr createOctant(Eigen::Vector3f const& center, float const& extent, uint32_t const& startIdx, uint32_t const& endIdx, uint32_t const& size);

			bool visit(OctreeNodePtr const& node, std::vector<Eigen::Vector3f>& center);

			std::vector<int> _getLeafNodeSamples(OctreeNodePtr const& node);

			int _getMedoidCenters(OctreeNodePtr const& node);

			bool _getMedoidCentersMulti(std::vector<int>& indices);
		protected:
			OctreeParams params_;
			OctreeNodePtr root_;
			PointCloudConstPtr input_cloud_;
			std::vector<uint32_t> successors_;  // single connected list of next point indices...
			std::vector<OctreeNodePtr> leaf_node_;

			Eigen::Vector2f factor_;
			uint32_t leaf_id_;
		};
	}
}