#pragma once
#include "registration_type.hpp"
#include <string>
#include <vector>

namespace Ddx
{
	class AlignResultIO
	{
	public:
		static bool saveAlignResult(std::string const& file_p, AlignResult const& info);
		static bool loadAlignResult(std::string const& file_p, AlignResult& info);

		static bool saveAlignResult(std::string const& file_p, std::vector<AlignResult> const& info_vec);
		static bool loadAlignResult(std::string const& file_p, std::vector<AlignResult>& info_vec);
	};
}
