#include "transformations_3d.hpp"

#ifndef WGS84_ELLIP_A
#define WGS84_ELLIP_A 6378137
#endif 
#ifndef WGS84_E2
#define WGS84_E2 0.00669437999013
#endif 

#define MATH_PI         3.14159265358979323846264338327950288   // pi

/* Angle conversions macros, DEG <-> RAD. */
#define MATH_RAD2DEG(x) ((x) * (180.0 / MATH_PI))
#define MATH_DEG2RAD(x) ((x) * (MATH_PI / 180.0))

namespace Ddx
{
	Eigen::Matrix4d
	Transformations3D::interpolation(Eigen::Matrix4d const& m1,
			Eigen::Matrix4d const& m2,
			double const& t1,
			double const& t2,
			double const& time)
	{
		double alpha = (time - t1) / (t2 - t1);

		Eigen::Vector3d v1 = m1.block<3, 1>(0, 3);
		Eigen::Quaterniond q1(m1.block<3, 3>(0, 0));

		Eigen::Vector3d v2 = m2.block<3, 1>(0, 3);
		Eigen::Quaterniond q2(m2.block<3, 3>(0, 0));

		Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

		result.block<3, 3>(0, 0) = slerp(q1, q2, alpha).toRotationMatrix();
		result.block<3, 1>(0, 3) = slerp(v1, v2, alpha);

		return result;
	}

	Eigen::Quaterniond
	Transformations3D::slerp(Eigen::Quaterniond const& q0, Eigen::Quaterniond const& q1, double alpha)
	{
		Eigen::Quaterniond qq = q0.slerp(alpha, q1);
		return qq;
	}

	Eigen::Vector3d
	Transformations3D::slerp(Eigen::Vector3d const& xyz0, Eigen::Vector3d const& xyz1, double alpha)
	{
		Eigen::Vector3d dPos = xyz1 - xyz0;

		return xyz0 + alpha * dPos;
	}

	Eigen::Vector3d
	Transformations3D::slerpGeodetic(Eigen::Vector3d const& geodetic0, Eigen::Vector3d const& geodetic1, double alpha)
	{
		Eigen::Vector3d ecef0 = Transformations3D::geodetic2ecef(geodetic0[0], geodetic0[1], geodetic0[2]);
		Eigen::Vector3d ecef1 = Transformations3D::geodetic2ecef(geodetic1[0], geodetic1[1], geodetic1[2]);
		double x = ecef0[0] + alpha * (ecef1[0] - ecef0[0]);
		double y = ecef0[1] + alpha * (ecef1[1] - ecef0[1]);
		double z = ecef0[2] + alpha * (ecef1[2] - ecef0[2]);
		Eigen::Vector3d lonlatalt = Transformations3D::ecef2geodetic(x, y, z);
		return lonlatalt;
	}

	Eigen::Vector3d
	Transformations3D::geodetic2ecef(const double& lon, const double& lat, const double& h)
	{
		double sinLat = sin(MATH_DEG2RAD(lat));
		double cosLat = cos(MATH_DEG2RAD(lat));
		double sinLon = sin(MATH_DEG2RAD(lon));
		double cosLon = cos(MATH_DEG2RAD(lon));
		double N = WGS84_ELLIP_A / (sqrt(1 - WGS84_E2 * sinLat * sinLat));
		double X = (N + h) * cosLat * cosLon;
		double Y = (N + h) * cosLat * sinLon;
		double Z = (N * (1 - WGS84_E2) + h) * sinLat;
		return Eigen::Vector3d(X, Y, Z);
	}

	Eigen::Vector3d
	Transformations3D::ecef2geodetic(const double& x, const double& y, const double& z)
	{
		//const GeocentricConverter geocentricConverter = new GeocentricConverter(spheroid);
		//const ProjCoordinate coordinate = new ProjCoordinate(x, y, z);
		//const ProjCoordinate result = new ProjCoordinate();
		//result.setValue(coordinate);
		//geocentricConverter.convertGeocentricToGeodetic(result);
		//const double[] geo = { Math.toDegrees(result.x), Math.toDegrees(result.y), result.z };
		const double& X = x;
		const double& Y = y;
		const double& Z = z;
		double B = 0.0, L = 0.0, H = 0.0; // rad
		double dZ = WGS84_E2 * Z;
		double N = 0.0;
		double sinB = 0.0;
		double dZP = 0.0;
		int flag = 0;
		do
		{
			dZP = dZ;
			L = atan2(Y, X);
			B = atan2(Z + dZ, sqrt(X * X + Y * Y));
			sinB = (Z + dZ) / sqrt(X * X + Y * Y + (Z + dZ) * (Z + dZ));
			N = WGS84_ELLIP_A / (sqrt(1 - WGS84_E2 * sin(B) * sin(B)));
			H = sqrt(X * X + Y * Y + (Z + dZ) * (Z + dZ)) - N;
			dZ = N * WGS84_E2 * sinB;
			flag += 1;
		} while (((dZ - dZP) > 1e-8) && (flag < 10));
		return Eigen::Vector3d(MATH_RAD2DEG(L), MATH_RAD2DEG(B), H);
	}

	Eigen::Vector3d
	Transformations3D::geodetic2enu(const double& lon, const double& lat, const double& h, const double& lon0, const double& lat0, const double& h0)
	{
		const Eigen::Vector3d XYZ0 = geodetic2ecef(lon0, lat0, h0);
		const Eigen::Vector3d XYZ2 = geodetic2ecef(lon, lat, h);
		double sinLon = sin(MATH_DEG2RAD(lon0));
		double cosLon = cos(MATH_DEG2RAD(lon0));
		double sinLat = sin(MATH_DEG2RAD(lat0));
		double cosLat = cos(MATH_DEG2RAD(lat0));
		const double& dx = XYZ2[0] - XYZ0[0];
		const double& dy = XYZ2[1] - XYZ0[1];
		const double& dz = XYZ2[2] - XYZ0[2];
		Eigen::Matrix3d matrix3d = Eigen::Matrix3d::Identity();

		matrix3d << -sinLon, cosLon, 0.0, -sinLat * cosLon, -sinLat * sinLon, cosLat, cosLat* cosLon, cosLat* sinLon, sinLat;

		const Eigen::Vector3d dXYZ = Eigen::Vector3d(dx, dy, dz);
		Eigen::Vector3d vector3d = matrix3d * dXYZ;
		return vector3d;
	}

	Eigen::Vector3d
	enu2geodetic0(const double& xEast, const double& yNorth, const double& zUp, const double& lon0, const double& lat0, const double& h0)
	{
		//第五步,地心地固坐标系下
		double m_fRyLatitude[3][3];		//纬度绕Y轴旋转引起的坐标转换矩阵
		double m_fRzLongitude[3][3];	//经度绕Z轴引起的坐标转换矩阵
		double m_fNavToEcef[3][3];		//导航到地心地固坐标系的坐标转换矩阵
		double m_fEcefOffset[3];

		double m_fInNavCoor[3];

		m_fInNavCoor[0] = xEast;
		m_fInNavCoor[1] = yNorth;
		m_fInNavCoor[2] = zUp;

		double fCosLat = cos(-1.0 * lat0 - 3.14159265358 / 2);
		double fSinLat = sin(-1.0 * lat0 - 3.14159265358 / 2);

		double fCosLon = cos(lon0);
		double fSinLon = sin(lon0);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m_fRyLatitude[i][j] = 0.0;
			}
		}
		m_fRyLatitude[0][0] = fCosLat;
		m_fRyLatitude[0][2] = fSinLat;
		m_fRyLatitude[1][1] = 1.0;
		m_fRyLatitude[2][0] = -1.0 * fSinLat;
		m_fRyLatitude[2][2] = fCosLat;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m_fRzLongitude[i][j] = 0.0;
			}
		}
		m_fRzLongitude[0][0] = fCosLon;
		m_fRzLongitude[0][1] = -1.0 * fSinLon;
		m_fRzLongitude[1][0] = fSinLon;
		m_fRzLongitude[1][1] = fCosLon;
		m_fRzLongitude[2][2] = 1.0;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m_fNavToEcef[i][j] = 0.0;
				for (int k = 0; k < 3; k++)
				{
					m_fNavToEcef[i][j] += m_fRzLongitude[i][k] * m_fRyLatitude[k][j];
				}
			}
		}

		//坐标转换变量初始化
		double m_fEarthA = 6378137.0;	//地球长半径
		double m_fEarthB = 6356752.3142;	//地球长半径
		double m_fEarthE = sqrt((m_fEarthA * m_fEarthA - m_fEarthB * m_fEarthB) / (m_fEarthA * m_fEarthA));	//第一偏心率平方6.69437999013e-03
		double  m_fEarthE2 = m_fEarthE * m_fEarthE;

		double fN = m_fEarthA / pow(1 - m_fEarthE2 * pow(sin(lat0), 2.0), 0.5);

		m_fEcefOffset[0] = (fN + h0) * cos(lat0) * cos(lon0);
		m_fEcefOffset[1] = (fN + h0) * cos(lat0) * sin(lon0);
		m_fEcefOffset[2] = (fN * (1 - m_fEarthE2) + h0) * sin(lat0);

		Eigen::Vector3d inEcefCoor;
		for (int i = 0; i < 3; i++)
		{
			inEcefCoor[i] = 0.0;
			for (int j = 0; j < 3; j++)
			{
				inEcefCoor[i] += (m_fNavToEcef[i][j] * m_fInNavCoor[j]);
			}
			inEcefCoor[i] += m_fEcefOffset[i];
		}
		return inEcefCoor;
	}

	Eigen::Vector3d
	Transformations3D::enu2geodetic(const double& xEast, const double& yNorth, const double& zUp, const double& lon0, const double& lat0, const double& h0)
	{
		const Eigen::Vector3d XYZ0 = geodetic2ecef(lon0, lat0, h0);
		double sinLon = sin(MATH_DEG2RAD(lon0));
		double cosLon = cos(MATH_DEG2RAD(lon0));
		double sinLat = sin(MATH_DEG2RAD(lat0));
		double cosLat = cos(MATH_DEG2RAD(lat0));

		Eigen::Matrix3d matrix3d = Eigen::Matrix3d::Identity();
		matrix3d.block<1, 3>(0, 0) = Eigen::Vector3d(-sinLon, -sinLat * cosLon, cosLat * cosLon);
		matrix3d.block<1, 3>(1, 0) = Eigen::Vector3d(cosLon, -sinLat * sinLon, cosLat * sinLon);
		matrix3d.block<1, 3>(2, 0) = Eigen::Vector3d(0.0, cosLat, sinLat);



		const Eigen::Vector3d dXYZ = matrix3d * Eigen::Vector3d(xEast, yNorth, zUp);

		const double X = dXYZ[0] + XYZ0[0];
		const double Y = dXYZ[1] + XYZ0[1];
		const double Z = dXYZ[2] + XYZ0[2];

		return ecef2geodetic(X, Y, Z);

	}
}