#pragma once

#include <registration_type/registration_type.hpp>
#include <graph/graph.hpp>
#include <map>
#include <string>
#include <vector>

namespace Ddx
{
	inline constexpr double kPoseGraphMinSimilarity = 1e-5;

	class PoseBase
	{
	public:
		PoseBase() = default;
		virtual ~PoseBase() = default;

		void setLidarFrames(std::vector<LidarFrame> const& lidar_frames) { m_lidar_frames = lidar_frames; }
		void setAlignResults(std::vector<AlignResult> const& edges) { m_align_edges = edges; }
		bool initialization();

	protected:
		void _buildPoseGraph(std::vector<GraphNode>& graph_nodes, std::vector<GraphEdge>& graph_edges) const;

		int _findUndirectedEdgeIndex(std::string const& src_name, std::string const& tgt_name) const;
		int _findEdgeIndex(std::string const& src_name, std::string const& tgt_name) const;
		int _findLidarFrameIndex(std::string const& name) const;

		std::vector<LidarFrame> m_lidar_frames;
		std::vector<AlignResult> m_align_edges;
		std::map<std::string, int> m_frame_map;
		std::map<std::string, int> m_edge_map;
	};
}
