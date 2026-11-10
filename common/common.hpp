#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace Ddx
{
	class Common
	{
	public:
		static std::vector<double> convert_matrix_to_vec7(Eigen::Matrix4d const& matrix);

		static Eigen::Matrix4d convert_vec7_to_matrix(std::vector<double> const& pose_vec);

		static std::string double_to_string(const double value, unsigned int precisionAfterPoint);

		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);

		static std::string matrix_to_south_string(Eigen::Matrix4d const& matrix);

		static  Eigen::Matrix4d south_string_to_matrix(std::string const& matrixText);

		static std::string matrix_k_to_south_string(Eigen::Matrix3d const& matrix);

		static  Eigen::Matrix3d south_string_to_matrix_k(std::string const& matrixText);

		static std::string vector2d_to_string(Eigen::Vector2d const& trans);

		static Eigen::Vector2d string_to_vector2d(std::string const& trans_txt);

		static std::string vector3d_to_string(Eigen::Vector3d const& trans);

		static Eigen::Vector3d string_to_vector3d(std::string const& trans_txt);

		static std::string matrix33_to_string(Eigen::Matrix3d const& matrix);

		static  Eigen::Matrix3d string_to_matrix33(std::string const& matrixText);

		static std::vector<int> generate_random_shuffle(int number, double samplingRate);

		static std::string get_edge_unique_key(std::string const& src_name, std::string const& tgt_name, bool is_src);

		static std::string get_edge_key(std::string const& src_name, std::string const& tgt_name);

		static std::string format(int index, int offset = 6);

		static std::string getVoxelName(int resolution);

		static float clamp(float x, float min_value, float max_value);

		static std::string eigen44_to_print_string(Eigen::Matrix4d const& matrix);

		static std::string current_time_to_string();

		static std::string double_to_string_fixed(double v, int decimals);

		// 编码边键。两端按自然序排成 min|max，唯一标识一条无向边。站名相等仍是字符串完全相等。
		static std::string encode_edge_key(std::string const& a, std::string const& b);

	};
}