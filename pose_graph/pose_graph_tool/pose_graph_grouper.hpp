#pragma once

#include "pose_base.hpp"
#include <string>
#include <vector>

namespace Ddx
{
	class PoseGraphGrouper : public PoseBase
	{
	public:
		bool process();
		std::vector<std::vector<std::string>> getGroups() const { return m_groups; }
		bool getGroupInfos(std::vector<std::string> const& group,
			std::vector<LidarFrame>& lidar_frames, std::vector<AlignResult>& align_edges);

	private:
		std::vector<std::vector<std::string>> m_groups;
	};
}
