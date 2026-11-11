#pragma once

#include <Eigen/Core>

namespace Ddx {

/// SE(3) Log: output twist ξ = [ρ(3); ϕ(3)] (translation then rotation), right-perturbation.
Eigen::Matrix<double, 6, 1> se3Log(Eigen::Matrix4d const& T);

/// SE(3) Exp: ξ = [ρ(3); ϕ(3)] → T = [Exp(ϕ), V ρ; 0 1]
Eigen::Matrix4d se3Exp(Eigen::Matrix<double, 6, 1> const& xi);

/// Matrix4d → quaternion (w,x,y,z) + translation; layout matches Ceres QuaternionManifold.
void matrixToQt(Eigen::Matrix4d const& T, double q[4], double t[3]);

/// Quaternion (w,x,y,z) + translation → Matrix4d.
Eigen::Matrix4d qtToMatrix(double const q[4], double const t[3]);

/// Cholesky Ω ≈ Lᵀ L with L lower-triangular. Returns false if not SPD / singular.
bool infoCholL(Eigen::Matrix<double, 6, 6> const& Omega, Eigen::Matrix<double, 6, 6>& L_out);

} // namespace Ddx
