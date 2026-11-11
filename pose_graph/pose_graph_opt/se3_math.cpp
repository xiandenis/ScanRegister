#include "se3_math.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace Ddx {
namespace {

constexpr double kSmallAngle = 1e-8;

Eigen::Matrix3d skew(Eigen::Vector3d const& v)
{
	Eigen::Matrix3d m;
	m << 0.0, -v.z(), v.y(),
		v.z(), 0.0, -v.x(),
		-v.y(), v.x(), 0.0;
	return m;
}

/// SO(3) Exp: rotation vector ϕ → R
Eigen::Matrix3d so3Exp(Eigen::Vector3d const& phi)
{
	double const theta_sq = phi.squaredNorm();
	if (theta_sq < kSmallAngle * kSmallAngle) {
		// First-order: R ≈ I + [ϕ]_×
		return Eigen::Matrix3d::Identity() + skew(phi);
	}
	double const theta = std::sqrt(theta_sq);
	Eigen::AngleAxisd const aa(theta, phi / theta);
	return aa.toRotationMatrix();
}

/// SO(3) Log: R → ϕ (axis-angle). Near 0 use first-order.
Eigen::Vector3d so3Log(Eigen::Matrix3d const& R)
{
	Eigen::AngleAxisd aa(R);
	double angle = aa.angle();
	// AngleAxis may return angle in [0, π]; keep axis consistent.
	if (angle < kSmallAngle) {
		// First-order: ϕ ≈ vee((R - Rᵀ)/2)
		Eigen::Vector3d phi;
		phi << 0.5 * (R(2, 1) - R(1, 2)),
			0.5 * (R(0, 2) - R(2, 0)),
			0.5 * (R(1, 0) - R(0, 1));
		return phi;
	}
	// Near π, AngleAxis axis is still valid; use angle * axis.
	return aa.axis() * angle;
}

/// Left Jacobian of SO(3): V such that t = V ρ for right-perturbation Exp.
Eigen::Matrix3d leftJacobianSO3(Eigen::Vector3d const& phi)
{
	double const theta_sq = phi.squaredNorm();
	Eigen::Matrix3d const Omega = skew(phi);
	if (theta_sq < kSmallAngle * kSmallAngle) {
		// V ≈ I + ½[ϕ]_× + ⅙[ϕ]_×²
		return Eigen::Matrix3d::Identity() + 0.5 * Omega + (1.0 / 6.0) * Omega * Omega;
	}
	double const theta = std::sqrt(theta_sq);
	double const A = (1.0 - std::cos(theta)) / theta_sq;
	double const B = (theta - std::sin(theta)) / (theta_sq * theta);
	return Eigen::Matrix3d::Identity() + A * Omega + B * Omega * Omega;
}

/// Inverse left Jacobian: ρ = V⁻¹ t
Eigen::Matrix3d leftJacobianSO3Inv(Eigen::Vector3d const& phi)
{
	double const theta_sq = phi.squaredNorm();
	Eigen::Matrix3d const Omega = skew(phi);
	if (theta_sq < kSmallAngle * kSmallAngle) {
		// V⁻¹ ≈ I - ½[ϕ]_× + ⅛[ϕ]_×²
		return Eigen::Matrix3d::Identity() - 0.5 * Omega + 0.125 * Omega * Omega;
	}
	double const theta = std::sqrt(theta_sq);
	double const half_theta = 0.5 * theta;
	double const cot_half =
		std::cos(half_theta) / std::sin(half_theta);
	double const C = (1.0 - theta * cot_half / 2.0) / theta_sq;
	// V⁻¹ = I - ½[ϕ]_× + (1/θ² - (1+cosθ)/(2 θ sinθ)) [ϕ]_×²
	// equivalent: C = (1 - (θ/2) cot(θ/2)) / θ²
	return Eigen::Matrix3d::Identity() - 0.5 * Omega + C * Omega * Omega;
}

} // namespace

Eigen::Matrix<double, 6, 1> se3Log(Eigen::Matrix4d const& T)
{
	Eigen::Matrix3d const R = T.block<3, 3>(0, 0);
	Eigen::Vector3d const t = T.block<3, 1>(0, 3);
	Eigen::Vector3d const phi = so3Log(R);
	Eigen::Vector3d const rho = leftJacobianSO3Inv(phi) * t;

	Eigen::Matrix<double, 6, 1> xi;
	xi.head<3>() = rho;
	xi.tail<3>() = phi;
	return xi;
}

Eigen::Matrix4d se3Exp(Eigen::Matrix<double, 6, 1> const& xi)
{
	Eigen::Vector3d const rho = xi.head<3>();
	Eigen::Vector3d const phi = xi.tail<3>();
	Eigen::Matrix3d const R = so3Exp(phi);
	Eigen::Vector3d const t = leftJacobianSO3(phi) * rho;

	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = R;
	T.block<3, 1>(0, 3) = t;
	return T;
}

void matrixToQt(Eigen::Matrix4d const& T, double q[4], double t[3])
{
	Eigen::Matrix3d const R = T.block<3, 3>(0, 0);
	Eigen::Quaterniond quat(R);
	quat.normalize();
	// QuaternionManifold layout: w, x, y, z (not EigenQuaternionManifold xyzw)
	q[0] = quat.w();
	q[1] = quat.x();
	q[2] = quat.y();
	q[3] = quat.z();
	t[0] = T(0, 3);
	t[1] = T(1, 3);
	t[2] = T(2, 3);
}

Eigen::Matrix4d qtToMatrix(double const q[4], double const t[3])
{
	Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
	quat.normalize();
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = quat.toRotationMatrix();
	T(0, 3) = t[0];
	T(1, 3) = t[1];
	T(2, 3) = t[2];
	return T;
}

bool infoCholL(Eigen::Matrix<double, 6, 6> const& Omega, Eigen::Matrix<double, 6, 6>& L_out)
{
	// Want Ω = Lᵀ L with L lower-triangular.
	// Standard Eigen LLT gives B = G Gᵀ (G lower).
	// With exchange matrix J: let B = J Ω J, then L = J Gᵀ J is lower and Ω = Lᵀ L.
	if (!Omega.allFinite()) {
		return false;
	}

	Eigen::Matrix<double, 6, 6> const B = Omega.reverse();
	Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(B);
	if (llt.info() != Eigen::Success) {
		return false;
	}

	// Reject numerically singular: tiny diagonal entries on G.
	Eigen::Matrix<double, 6, 6> const G = llt.matrixL();
	constexpr double kMinDiag = 1e-12;
	for (int i = 0; i < 6; ++i) {
		if (!(G(i, i) > kMinDiag)) {
			return false;
		}
	}

	L_out = G.transpose().reverse();
	return true;
}

} // namespace Ddx
