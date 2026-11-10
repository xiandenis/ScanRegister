#include "common.hpp"
#include "natural_name.hpp"
#include <sstream>
#include <fstream>  
#include <iostream>
#include <iomanip>
#include <random>
#include <ctime>

namespace Ddx
{
	std::vector<double>
	Common::convert_matrix_to_vec7(Eigen::Matrix4d const& matrix)
	{
		std::vector<double> pose(7, 0.0);
		Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));

		pose[0] = matrix(0, 3);
		pose[1] = matrix(1, 3);
		pose[2] = matrix(2, 3);
		pose[3] = quaternion.x();
		pose[4] = quaternion.y();
		pose[5] = quaternion.z();
		pose[6] = quaternion.w();

		return pose;
	}

	Eigen::Matrix4d
	Common::convert_vec7_to_matrix(std::vector<double> const& pose_vec)
	{
		Eigen::Quaterniond quaternion(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5]);
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
		matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
		matrix(0, 3) = pose_vec[0];
		matrix(1, 3) = pose_vec[1];
		matrix(2, 3) = pose_vec[2];

		return matrix;
	}

	std::string
	Common::double_to_string(const double value, unsigned int precisionAfterPoint)
	{
		std::ostringstream out;
		// ���Ĭ�Ͼ���
		out.precision(std::numeric_limits<double>::digits10);
		out << std::setprecision(15) << std::fixed << value;

		std::string res = std::move(out.str());
		auto pos = res.find('.');
		if (pos == std::string::npos)
			return res;
		auto splitLen = pos + 1 + precisionAfterPoint;
		if (res.size() <= splitLen)
			return res;

		return res.substr(0, splitLen);
	}

	void
	Common::tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters)
	{
		// Skip delimiters at beginning.
		std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
		// Find first "non-delimiter".
		std::string::size_type pos = str.find_first_of(delimiters, lastPos);
		while (std::string::npos != pos || std::string::npos != lastPos)
		{
			// Found a token, add it to the vector.
			tokens.push_back(str.substr(lastPos, pos - lastPos));
			// Skip delimiters.  Note the "not_of"
			lastPos = str.find_first_not_of(delimiters, pos);
			// Find next "non-delimiter"
			pos = str.find_first_of(delimiters, lastPos);
		}
	}

	std::string
	Common::matrix_to_south_string(Eigen::Matrix4d const& matrix)
	{
		std::string matrixText = "";
		matrixText += double_to_string(matrix(0, 3), 6) + ",";
		matrixText += double_to_string(matrix(1, 3), 6) + ",";
		matrixText += double_to_string(matrix(2, 3), 6) + ",";
		Eigen::Quaterniond qr = Eigen::Quaterniond(matrix.block<3, 3>(0, 0));
		matrixText += double_to_string(qr.w(), 9) + ",";
		matrixText += double_to_string(qr.x(), 9) + ",";
		matrixText += double_to_string(qr.y(), 9) + ",";
		matrixText += double_to_string(qr.z(), 9) + ",";

		return matrixText;
	}

	Eigen::Matrix4d
	Common::south_string_to_matrix(std::string const& matrixText)
	{
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
		std::vector<std::string> matrixVec;
		if (!matrixText.empty()) {
			tokenize(matrixText, matrixVec, ",");
		}
		if (matrixVec.size() != 7) {
			return matrix;
		}

		Eigen::Vector3d t(atof(matrixVec[0].c_str()), atof(matrixVec[1].c_str()), atof(matrixVec[2].c_str()));
		Eigen::Quaterniond qr(atof(matrixVec[3].c_str()), \
			atof(matrixVec[4].c_str()), \
			atof(matrixVec[5].c_str()), \
			atof(matrixVec[6].c_str()));

		matrix = Eigen::Matrix4d::Identity();
		matrix.block<3, 3>(0, 0) = qr.toRotationMatrix();
		matrix.block<3, 1>(0, 3) = t;
		return matrix;
	}

	std::string 
	Common::matrix_k_to_south_string(Eigen::Matrix3d const& matrix) {
		std::string matrixText = "";
		matrixText += double_to_string(matrix(0, 0), 6) + ",";
		matrixText += double_to_string(matrix(1, 1), 6) + ",";
		matrixText += double_to_string(matrix(0, 2), 6) + ",";
		matrixText += double_to_string(matrix(1, 2), 6) + ",";
		return matrixText;
	}

	Eigen::Matrix3d 
	Common::south_string_to_matrix_k(std::string const& matrixText)
	{
		Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
		std::vector<std::string> matrixVec;
		if (!matrixText.empty()) {
			tokenize(matrixText, matrixVec, ",");
		}
		if (matrixVec.size() != 4) {
			return matrix;
		}

		matrix(0, 0) = atof(matrixVec[0].c_str());
		matrix(1, 1) = atof(matrixVec[1].c_str());
		matrix(0, 2) = atof(matrixVec[2].c_str());
		matrix(1, 2) = atof(matrixVec[3].c_str());
		return matrix;
	}

	std::string
	Common::vector3d_to_string(Eigen::Vector3d const& trans)
	{
		std::string vector_txt = "";
		vector_txt += double_to_string(trans(0), 6) + ",";
		vector_txt += double_to_string(trans(1), 6) + ",";
		vector_txt += double_to_string(trans(2), 6);
		return vector_txt;
	}

	Eigen::Vector3d
	Common::string_to_vector3d(std::string const& trans_txt)
	{
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
		std::vector<std::string> value_vec;
		if (!trans_txt.empty()) {
			tokenize(trans_txt, value_vec, ",");
		}

		if (value_vec.size() != 3) {
			return translation;
		}
		translation = Eigen::Vector3d(atof(value_vec[0].c_str()), atof(value_vec[1].c_str()), atof(value_vec[2].c_str()));

		return translation;
	}

	std::string
	Common::vector2d_to_string(Eigen::Vector2d const& trans)
	{
		std::string vector_txt = "";
		vector_txt += double_to_string(trans(0), 6) + ",";
		vector_txt += double_to_string(trans(1), 6);
		return vector_txt;
	}

	Eigen::Vector2d
	Common::string_to_vector2d(std::string const& trans_txt)
	{
		Eigen::Vector2d translation = Eigen::Vector2d::Zero();
		std::vector<std::string> value_vec;
		if (!trans_txt.empty()) {
			tokenize(trans_txt, value_vec, ",");
		}

		if (value_vec.size() != 2) {
			return translation;
		}
		translation = Eigen::Vector2d(atof(value_vec[0].c_str()), atof(value_vec[1].c_str()));

		return translation;
	}

	std::string
	Common::matrix33_to_string(Eigen::Matrix3d const& matrix)
	{
		std::string matrixText = "";
		Eigen::Quaterniond qr = Eigen::Quaterniond(matrix);
		matrixText += double_to_string(qr.w(), 9) + ",";
		matrixText += double_to_string(qr.x(), 9) + ",";
		matrixText += double_to_string(qr.y(), 9) + ",";
		matrixText += double_to_string(qr.z(), 9);

		return matrixText;
	}

	Eigen::Matrix3d
	Common::string_to_matrix33(std::string const& matrixText)
	{
		Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
		std::vector<std::string> matrixVec;
		if (!matrixText.empty()) {
			tokenize(matrixText, matrixVec, ",");
		}
		if (matrixVec.size() != 4) {
			return matrix;
		}

		Eigen::Quaterniond qr(atof(matrixVec[0].c_str()), \
			atof(matrixVec[1].c_str()), \
			atof(matrixVec[2].c_str()), \
			atof(matrixVec[3].c_str()));


		return qr.toRotationMatrix();
	}

	std::vector<int>
	Common::generate_random_shuffle(int number, double samplingRate)
	{
		std::random_device mRandomSeed;
		std::mt19937_64 mRandomDevice(mRandomSeed());
		const int int32_MAX = std::numeric_limits<int>::max();

		std::vector<int> rawVector(number, -1);
		for (int i = 0; i < number; ++i)
		{
			rawVector[i] = i;
		}

		if (samplingRate < 0.55)
		{
			int finalNumber = static_cast<int>(double(number) * samplingRate);
			int tempIndice = number;		// ��ʼ���
			for (int i = 0; i < finalNumber; ++i)
			{
				const auto r1x = static_cast<int>(mRandomDevice() & int32_MAX);
				int ix = r1x % static_cast<int>(tempIndice);		// ��ʣ��ı�ǩ����ȡһ����
				const auto r90x = rawVector[ix];
				rawVector[ix] = rawVector[tempIndice - 1];
				rawVector[tempIndice - 1] = r90x;
				tempIndice--;
			}

			return std::vector<int>(rawVector.begin() + tempIndice, rawVector.end());
		}
		else // ������ȡֵ
		{
			int finalNumber = static_cast<int> (double(number) * (1.0 - samplingRate));
			int tempIndice = number;				// ��ʼ���
			for (int i = 0; i < finalNumber; ++i)
			{
				const auto r1x = static_cast<int>(mRandomDevice() & int32_MAX);
				int ix = r1x % static_cast<int>(tempIndice);		// ��ʣ��ı�ǩ����ȡһ����
				const auto r90x = rawVector[ix];
				rawVector[ix] = rawVector[tempIndice - 1];
				rawVector[tempIndice - 1] = r90x;
				tempIndice--;
			}

			return std::vector<int>(rawVector.begin(), rawVector.begin() + tempIndice);
		}
	}

	std::string
	Common::get_edge_unique_key(std::string const& src_name, std::string const& tgt_name, bool is_src)
	{
		std::string name = src_name + "-|-" + tgt_name + "_";
		name += is_src ? src_name : tgt_name;
		return name;
	}

	std::string
	Common::get_edge_key(std::string const& src_name, std::string const& tgt_name)
	{
		return "key_" + src_name + "_" + tgt_name;
	}

	std::string
	Common::format(int index, int offset)
	{
		std::string r11x;
		if (true) {
			std::stringstream ss;
			ss << std::setw(offset) << std::setfill('0') << index;
			ss >> r11x;         //���ַ������� str
		}
		return r11x;
	}

	std::string
	Common::getVoxelName(int resolution)
	{
		return "voxel_" + std::to_string(resolution);
	}
	
	float
	Common::clamp(float x, float min_value, float max_value)
	{
		return std::max(min_value, std::min(x, max_value));
	}

	std::string 
	Common::eigen44_to_print_string(Eigen::Matrix4d const& matrix)
	{
		std::stringstream ss;
		ss << matrix;
		return ss.str();
	}

	std::string 
	Common::current_time_to_string()
	{
		std::time_t now = std::time(0);
		// ��ʱ���ת��Ϊ����ʱ��ṹ
		std::tm* localTime = std::localtime(&now);
		// ��ʽ��ʱ��Ϊ�ַ���
		char timeStr[100];
		std::strftime(timeStr, sizeof(timeStr), "%Y_%m_%d_%H_%M_%S", localTime);

		std::string name = timeStr;
		return name;
	}
	
	std::string 
	Common::double_to_string_fixed(double v, int decimals)
	{
		std::ostringstream os;
		os << std::fixed << std::setprecision(decimals) << v;
		return os.str();
	}

	std::string 
	Common::encode_edge_key(std::string const& a, std::string const& b)
	{
		if (!naturalNameLess(b, a)) {
			return a + "|" + b;
		}
		return b + "|" + a;
	}

}