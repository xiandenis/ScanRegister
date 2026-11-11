#include "pose_global_system.hpp"
#include <common/natural_name.hpp>
#include <graph/graph.hpp>
#include <logging/logging.hpp>

namespace Ddx
{
	bool
	PoseGlobalSystem::process()
	{
		if (initialization() == false)
			return false;

		m_global_matrixs.clear();
		int const frame_size = static_cast<int>(m_lidar_frames.size());
		if (frame_size <= 1)
		{
			m_global_matrixs[m_lidar_frames[0].name_] = Eigen::Matrix4d::Identity();
			return true;
		}

		for (int i = 0; i < frame_size; ++i)
			m_global_matrixs[m_lidar_frames[i].name_] = Eigen::Matrix4d::Identity();

		std::vector<GraphNode> graph_nodes;
		std::vector<GraphEdge> graph_edges;
		_buildPoseGraph(graph_nodes, graph_edges);

		LOG_INFO("Minimum Spanning Tree");
		MinimumSpanningTree<GraphNode, GraphEdge> minimumSpanningTree;
		std::vector<GraphEdge> mst_edges = minimumSpanningTree.kruskalRun2(graph_nodes, graph_edges);

		// 连通图 n 个节点必有 n-1 条 MST 边；否则为多连通分量
		if (static_cast<int>(mst_edges.size()) < frame_size - 1)
		{
			LOG_WARN("PoseGlobalSystem: pose graph is not connected ({} frames, {} MST edges)",
				frame_size, mst_edges.size());
			m_global_matrixs.clear();
			return false;
		}

		std::vector<std::vector<int>> adj(static_cast<std::size_t>(frame_size));
		for (int i = 0; i < static_cast<int>(mst_edges.size()); ++i)
		{
			int const s_id = static_cast<int>(mst_edges[i].src);
			int const t_id = static_cast<int>(mst_edges[i].dst);
			if (s_id < 0 || s_id >= frame_size || t_id < 0 || t_id >= frame_size || s_id == t_id)
			{
				LOG_ERROR("PoseGlobalSystem: invalid MST edge ({} -> {})", s_id, t_id);
				m_global_matrixs.clear();
				return false;
			}
			adj[static_cast<std::size_t>(s_id)].push_back(t_id);
			adj[static_cast<std::size_t>(t_id)].push_back(s_id);
		}

		// 以站名自然序第一站为参考原点（全局单位阵）
		int root_index = 0;
		for (int i = 1; i < frame_size; ++i)
		{
			if (naturalNameLess(
					m_lidar_frames[static_cast<std::size_t>(i)].name_,
					m_lidar_frames[static_cast<std::size_t>(root_index)].name_))
				root_index = i;
		}

		std::vector<bool> visited(static_cast<std::size_t>(frame_size), false);
		visited[static_cast<std::size_t>(root_index)] = true;
		if (_DFS(root_index, visited, adj) == false)
		{
			m_global_matrixs.clear();
			return false;
		}

		return true;
	}

	bool
	PoseGlobalSystem::_DFS(int node, std::vector<bool>& visited,
		std::vector<std::vector<int>> const& adj)
	{
		for (int const i : adj[static_cast<std::size_t>(node)])
		{
			if (visited[static_cast<std::size_t>(i)])
				continue;

			std::string const& tgt_name = m_lidar_frames[static_cast<std::size_t>(node)].name_;
			std::string const& src_name = m_lidar_frames[static_cast<std::size_t>(i)].name_;

			int const e_id = _findUndirectedEdgeIndex(src_name, tgt_name);
			if (e_id < 0)
			{
				LOG_ERROR("PoseGlobalSystem: missing align edge between '{}' and '{}'", src_name, tgt_name);
				return false;
			}

			// 约定与 RealTimeManager 一致：G_src = G_tgt * matrix_（matrix_ 将 src 变到 tgt）
			Eigen::Matrix4d matrix;
			if (_findEdgeIndex(src_name, tgt_name) >= 0)
				matrix = m_global_matrixs.at(tgt_name) * m_align_edges[static_cast<std::size_t>(e_id)].matrix_;
			else
				matrix = m_global_matrixs.at(tgt_name) * m_align_edges[static_cast<std::size_t>(e_id)].matrix_.inverse();
			m_global_matrixs[src_name] = matrix;

			visited[static_cast<std::size_t>(i)] = true;
			if (_DFS(i, visited, adj) == false)
				return false;
		}
		return true;
	}
}
