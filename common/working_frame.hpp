#pragma once

#include <Eigen/Core>

namespace Ddx
{

/// Shared local working frame for one processing job (all input files use the same origin).
/// Always apply p' = p - origin — do not special-case "large" vs "small" CRS.
/// Dense XYZ buffers prefer float in local; poses / provenance stay double / u64.
struct WorkingFrame
{
	Eigen::Vector3d origin = Eigen::Vector3d::Zero();

	static WorkingFrame fromOrigin(Eigen::Vector3d const& o)
	{
		WorkingFrame f;
		f.origin = o;
		return f;
	}

	Eigen::Vector3d toLocal(Eigen::Vector3d const& world) const { return world - origin; }

	Eigen::Vector3d toWorld(Eigen::Vector3d const& local) const { return local + origin; }

	Eigen::Vector3f toLocalF(Eigen::Vector3d const& world) const
	{
		Eigen::Vector3d const l = toLocal(world);
		return Eigen::Vector3f(static_cast<float>(l.x()), static_cast<float>(l.y()),
		                       static_cast<float>(l.z()));
	}

	Eigen::Vector3d toWorldFromF(Eigen::Vector3f const& local) const
	{
		return toWorld(Eigen::Vector3d(static_cast<double>(local.x()),
		                               static_cast<double>(local.y()),
		                               static_cast<double>(local.z())));
	}
};

} // namespace Ddx
