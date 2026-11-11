#pragma once

#include "pose_base.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace Ddx {

	class PoseGlobalSystem : public PoseBase
	{
	public:
		bool process();
		std::map<std::string, Eigen::Matrix4d> getGlobalMatrixs() const { return m_global_matrixs; }

	private:
		bool _DFS(int node, std::vector<bool>& visited,
			std::vector<std::vector<int>> const& adj);

		std::map<std::string, Eigen::Matrix4d> m_global_matrixs;
	};
}
