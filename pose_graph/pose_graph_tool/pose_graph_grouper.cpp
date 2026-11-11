#include "pose_graph_grouper.hpp"
#include <graph/graph.hpp>
#include <logging/logging.hpp>
#include <unordered_set>

namespace Ddx
{
	bool
	PoseGraphGrouper::process()
	{
		if (initialization() == false)
			return false;

		m_groups.clear();
		int const frame_size = static_cast<int>(m_lidar_frames.size());
		if (frame_size <= 0)
			return false;

		std::vector<GraphNode> graph_nodes;
		std::vector<GraphEdge> graph_edges;
		_buildPoseGraph(graph_nodes, graph_edges);

		LOG_INFO("Connected components");
		ConnectedComponentsSubgraph<GraphNode, GraphEdge> subgraph;
		const auto components = subgraph.run2(graph_nodes, graph_edges);
		m_groups.resize(components.size());

		for (int i = 0; i < static_cast<int>(components.size()); ++i)
		{
			for (int j = 0; j < static_cast<int>(components[i].size()); ++j)
			{
				int const node_id = static_cast<int>(components[i][j].id);
				if (node_id < 0 || node_id >= frame_size)
				{
					LOG_ERROR("PoseGraphGrouper: invalid node id {} (frame_size={})", node_id, frame_size);
					m_groups.clear();
					return false;
				}
				m_groups[static_cast<std::size_t>(i)].push_back(m_lidar_frames[static_cast<std::size_t>(node_id)].name_);
			}
		}

		return true;
	}

	bool
	PoseGraphGrouper::getGroupInfos(std::vector<std::string> const& group,
		std::vector<LidarFrame>& lidar_frames, std::vector<AlignResult>& align_edges)
	{
		if (initialization() == false)
			return false;

		std::unordered_set<std::string> group_names;
		lidar_frames.clear();
		lidar_frames.reserve(group.size());
		for (int i = 0; i < static_cast<int>(group.size()); ++i)
		{
			int l_id = _findLidarFrameIndex(group[i]);
			if (l_id < 0)
				continue;
			lidar_frames.push_back(m_lidar_frames[l_id]);
			group_names.insert(group[i]);
		}

		if (lidar_frames.empty())
			return false;

		align_edges.clear();
		for (int i = 0; i < static_cast<int>(m_align_edges.size()); ++i)
		{
			if (!Ddx::stateIsValid(m_align_edges[i].state_))
				continue;
			if (group_names.find(m_align_edges[i].src_name_) == group_names.end())
				continue;
			if (group_names.find(m_align_edges[i].tgt_name_) == group_names.end())
				continue;
			align_edges.push_back(m_align_edges[i]);
		}

		return true;
	}
}
