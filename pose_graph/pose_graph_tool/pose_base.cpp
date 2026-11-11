#include "pose_base.hpp"
#include <common/common.hpp>

namespace Ddx
{
	bool
	PoseBase::initialization()
	{
		if (m_lidar_frames.empty())
			return false;

		m_frame_map.clear();
		for (int i = 0; i < static_cast<int>(m_lidar_frames.size()); ++i)
			m_frame_map[m_lidar_frames[i].name_] = i;

		m_edge_map.clear();
		for (int i = 0; i < static_cast<int>(m_align_edges.size()); ++i)
		{
			std::string key = Ddx::Common::get_edge_key(
				m_align_edges[i].src_name_, m_align_edges[i].tgt_name_);
			m_edge_map[key] = i;
		}

		return true;
	}

	void
	PoseBase::_buildPoseGraph(std::vector<GraphNode>& graph_nodes, std::vector<GraphEdge>& graph_edges) const
	{
		int const frame_size = static_cast<int>(m_lidar_frames.size());
		graph_nodes.resize(frame_size);
		for (int i = 0; i < frame_size; ++i)
		{
			graph_nodes[i].id = i;
			graph_nodes[i].name_ = m_lidar_frames[i].name_;
		}

		graph_edges.clear();
		graph_edges.reserve(m_align_edges.size());
		for (int i = 0; i < static_cast<int>(m_align_edges.size()); ++i)
		{
			if (!Ddx::stateIsValid(m_align_edges[i].state_))
				continue;

			std::string const& src_name = m_align_edges[i].src_name_;
			std::string const& tgt_name = m_align_edges[i].tgt_name_;
			if (m_frame_map.find(src_name) == m_frame_map.end()
				|| m_frame_map.find(tgt_name) == m_frame_map.end())
				continue;

			GraphEdge g_edge;
			g_edge.src = m_frame_map.at(src_name);
			g_edge.dst = m_frame_map.at(tgt_name);
			double similarity = m_align_edges[i].similarity_;
			if (similarity <= 0.0)
				similarity = kPoseGraphMinSimilarity;
			g_edge.weight = m_align_edges[i].weight_ * m_align_edges[i].rms_ / similarity;
			graph_edges.push_back(g_edge);
		}
	}

	int
	PoseBase::_findUndirectedEdgeIndex(std::string const& src_name, std::string const& tgt_name) const
	{
		int e_1 = _findEdgeIndex(src_name, tgt_name);
		if (e_1 >= 0)
			return e_1;
		return _findEdgeIndex(tgt_name, src_name);
	}

	int
	PoseBase::_findEdgeIndex(std::string const& src_name, std::string const& tgt_name) const
	{
		std::string key = Common::get_edge_key(src_name, tgt_name);
		auto const it = m_edge_map.find(key);
		if (it == m_edge_map.end())
			return -1;
		return it->second;
	}

	int
	PoseBase::_findLidarFrameIndex(std::string const& name) const
	{
		auto const it = m_frame_map.find(name);
		if (it == m_frame_map.end())
			return -1;
		return it->second;
	}
}
