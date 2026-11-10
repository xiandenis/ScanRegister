#pragma once
#include "registration_type.hpp"
#include <string>
#include <vector>

namespace Ddx
{
	class LidarFrameIO
	{
	public:
		static bool saveLidarFrame(std::string const& file_p, LidarFrame const& frame);
		static bool loadLidarFrame(std::string const& file_p, LidarFrame& frame);

		static bool saveLidarFrame(std::string const& file_p, std::vector<LidarFrame> const& frame_vec);
		static bool loadLidarFrame(std::string const& file_p, std::vector<LidarFrame>& frame_vec);
	};
}
