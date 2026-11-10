#pragma once

#include <Eigen/Dense>

namespace Ddx
{
	class Transformations3D
	{
	public:

		static Eigen::Matrix4d interpolation(Eigen::Matrix4d const& m1,
			Eigen::Matrix4d const& m2,
			double const& t1,
			double const& t2,
			double const& time);

		static Eigen::Quaterniond slerp(Eigen::Quaterniond const& q0, Eigen::Quaterniond const& q1, double alpha);

		static Eigen::Vector3d slerp(Eigen::Vector3d const& xyz0, Eigen::Vector3d const& xyz1, double alpha);

		static Eigen::Vector3d slerpGeodetic(Eigen::Vector3d const& geodetic0, Eigen::Vector3d const& geodetic1, double alpha);

		static Eigen::Vector3d geodetic2ecef(const double& lon, const double& lat, const double& h);

		static Eigen::Vector3d ecef2geodetic(const double& x, const double& y, const double& z);

		static Eigen::Vector3d geodetic2enu(const double& lon, const double& lat, const double& h, const double& lon0, const double& lat0, const double& h0);

		static Eigen::Vector3d enu2geodetic(const double& xEast, const double& yNorth, const double& zUp, const double& lon0, const double& lat0, const double& h0);
	};
}
