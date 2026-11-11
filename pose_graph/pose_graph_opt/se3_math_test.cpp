#include "se3_math.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

namespace {

int g_fail = 0;

void expect(bool cond, char const* msg)
{
	if (!cond) {
		std::cerr << "FAIL: " << msg << "\n";
		++g_fail;
	} else {
		std::cout << "OK: " << msg << "\n";
	}
}

bool almostEqualVec6(Eigen::Matrix<double, 6, 1> const& a,
	Eigen::Matrix<double, 6, 1> const& b,
	double tol)
{
	return (a - b).norm() < tol;
}

} // namespace

int main()
{
	using Ddx::infoCholL;
	using Ddx::se3Exp;
	using Ddx::se3Log;

	// 1) Identity log ≈ 0
	{
		Eigen::Matrix4d const I = Eigen::Matrix4d::Identity();
		Eigen::Matrix<double, 6, 1> const xi = se3Log(I);
		expect(almostEqualVec6(xi, Eigen::Matrix<double, 6, 1>::Zero(), 1e-12),
			"identity log ≈ 0");
		expect(se3Exp(xi).isApprox(I, 1e-12), "exp(log(I)) ≈ I");
	}

	// 2) 纯平移 (1,0,0): log head ≈ (1,0,0), tail≈0；exp 往返
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T(0, 3) = 1.0;
		Eigen::Matrix<double, 6, 1> const xi = se3Log(T);
		Eigen::Matrix<double, 6, 1> expected = Eigen::Matrix<double, 6, 1>::Zero();
		expected(0) = 1.0;
		expect(almostEqualVec6(xi, expected, 1e-12), "pure translation log head≈(1,0,0) tail≈0");
		expect(se3Exp(xi).isApprox(T, 1e-12), "pure translation exp∘log");
		expect(se3Log(se3Exp(expected)).isApprox(expected, 1e-12), "pure translation log∘exp");
	}

	// 3) 绕 Z 转 90°：往返 Matrix4d 误差 < 1e-9
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T.block<3, 3>(0, 0) =
			Eigen::AngleAxisd(0.5 * 3.14159265358979323846, Eigen::Vector3d::UnitZ())
				.toRotationMatrix();
		T(0, 3) = 0.3;
		T(1, 3) = -0.2;
		T(2, 3) = 1.5;
		Eigen::Matrix4d const T2 = se3Exp(se3Log(T));
		expect((T2 - T).norm() < 1e-9, "Z 90° + translation exp∘log Matrix4d err < 1e-9");
	}

	// 4) Ω=I → Chol 成功；Ω 全零 → 失败
	{
		Eigen::Matrix<double, 6, 6> L;
		Eigen::Matrix<double, 6, 6> const I = Eigen::Matrix<double, 6, 6>::Identity();
		expect(infoCholL(I, L), "Ω=I → Chol success");
		expect((L.transpose() * L - I).norm() < 1e-12, "Ω=I → LᵀL ≈ I");
		expect(L.isLowerTriangular(), "L is lower-triangular");

		Eigen::Matrix<double, 6, 6> const Z = Eigen::Matrix<double, 6, 6>::Zero();
		expect(!infoCholL(Z, L), "Ω=0 → Chol fails");
	}

	// matrixToQt / qtToMatrix round-trip (quaternion w,x,y,z)
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T.block<3, 3>(0, 0) =
			Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.5, 0.8).normalized()).toRotationMatrix();
		T(0, 3) = 1.1;
		T(1, 3) = -2.2;
		T(2, 3) = 3.3;
		double q[4];
		double t[3];
		Ddx::matrixToQt(T, q, t);
		Eigen::Matrix4d const T2 = Ddx::qtToMatrix(q, t);
		expect(T2.isApprox(T, 1e-12), "matrixToQt / qtToMatrix round-trip");
		expect(std::abs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0) < 1e-12,
			"quaternion unit length");
	}

	if (g_fail != 0) {
		std::cerr << g_fail << " test(s) failed\n";
		return 1;
	}
	std::cout << "all se3_math tests passed\n";
	return 0;
}
