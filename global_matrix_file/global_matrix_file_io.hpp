#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>

namespace Ddx
{
	class GlobalMatrixFileIO
	{
	public:
		static bool loading(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d>& site_global_matrix);
		static bool saving(std::string const& matrix_file_path, std::map<std::string, Eigen::Matrix4d> const& site_global_matrix);
	};
}


