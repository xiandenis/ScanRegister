#include "normal_estimate.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_tbb.hpp>

namespace Ddx
{
	namespace detail
	{
		struct XyzNormalView
		{
			pcl::PointCloud<pcl::PointXYZ> const* xyz = nullptr;
			pcl::PointCloud<pcl::Normal>* normals = nullptr;
		};
	}
}

namespace small_gicp
{
	namespace traits
	{
		template <>
		struct Traits<Ddx::detail::XyzNormalView>
		{
			using Points = Ddx::detail::XyzNormalView;

			static size_t size(Points const& v)
			{
				return v.xyz ? v.xyz->size() : 0;
			}

			static void resize(Points& v, size_t n)
			{
				if (v.normals) {
					v.normals->resize(n);
				}
			}

			static bool has_points(Points const& v) { return v.xyz && !v.xyz->empty(); }
			static bool has_normals(Points const& v) { return v.normals && !v.normals->empty(); }
			static bool has_covs(Points const&) { return false; }

			static Eigen::Vector4d point(Points const& v, size_t i)
			{
				auto const& p = v.xyz->points[i];
				return Eigen::Vector4d(static_cast<double>(p.x), static_cast<double>(p.y),
				                       static_cast<double>(p.z), 1.0);
			}

			static void set_normal(Points& v, size_t i, Eigen::Vector4d const& n)
			{
				auto& o = v.normals->points[i];
				o.normal_x = static_cast<float>(n.x());
				o.normal_y = static_cast<float>(n.y());
				o.normal_z = static_cast<float>(n.z());
			}

			static void set_point(Points&, size_t, Eigen::Vector4d const&) {}
			static void set_cov(Points&, size_t, Eigen::Matrix4d const&) {}
			static Eigen::Vector4d normal(Points const& v, size_t i)
			{
				auto const& o = v.normals->points[i];
				return Eigen::Vector4d(static_cast<double>(o.normal_x), static_cast<double>(o.normal_y),
				                       static_cast<double>(o.normal_z), 0.0);
			}
			static Eigen::Matrix4d cov(Points const&, size_t) { return Eigen::Matrix4d::Zero(); }
		};
	}
}

namespace
{
	void write_zero_normal(pcl::Normal& o)
	{
		o.normal_x = 0.f;
		o.normal_y = 0.f;
		o.normal_z = 0.f;
		o.curvature = 0.f;
	}

	// small_gicp NormalSetter cannot write PCL curvature (only eigenvectors, λ discarded).
	// Replicate knn+PCA here: SelfAdjointEigenSolver::computeDirect, col(0) = normal,
	// curvature = λ0 / (λ0+λ1+λ2) (PCL NormalEstimation).
	void estimate_one_pca_curvature(small_gicp::UnsafeKdTree<Ddx::detail::XyzNormalView> const& tree,
	                                Ddx::detail::XyzNormalView& view, int num_neighbors, size_t i)
	{
		std::vector<size_t> k_indices(static_cast<size_t>(num_neighbors));
		std::vector<double> k_sq_dists(static_cast<size_t>(num_neighbors));
		size_t const n = tree.knn_search(small_gicp::traits::point(view, i), num_neighbors,
		                                 k_indices.data(), k_sq_dists.data());
		auto& o = view.normals->points[i];
		if (n < 5) {
			write_zero_normal(o);
			return;
		}

		Eigen::Vector4d sum_points = Eigen::Vector4d::Zero();
		Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();
		for (size_t j = 0; j < n; ++j) {
			Eigen::Vector4d const pt = small_gicp::traits::point(view, k_indices[j]);
			sum_points += pt;
			sum_cross += pt * pt.transpose();
		}

		Eigen::Vector4d const mean = sum_points / static_cast<double>(n);
		Eigen::Matrix4d const cov = (sum_cross - mean * sum_points.transpose()) / static_cast<double>(n);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
		eig.computeDirect(cov.block<3, 3>(0, 0));

		Eigen::Vector4d normal4;
		normal4 << eig.eigenvectors().col(0).normalized(), 0.0;
		if (small_gicp::traits::point(view, i).dot(normal4) > 0.0) {
			normal4 = -normal4;
		}

		o.normal_x = static_cast<float>(normal4.x());
		o.normal_y = static_cast<float>(normal4.y());
		o.normal_z = static_cast<float>(normal4.z());

		Eigen::Vector3d const lam = eig.eigenvalues();
		double const sum = lam(0) + lam(1) + lam(2);
		if (std::abs(sum) < 1e-12) {
			o.curvature = 0.f;
		} else {
			o.curvature = static_cast<float>(lam(0) / sum);
		}
	}
}

namespace Ddx
{
	bool estimateNormalsKnn(pcl_type::PointCloudPtr const& cloud, pcl_type::NormalPtr& normal,
	                        NormalEstimateMethod method)
	{
		if (!cloud || cloud->size() < 2) {
			return false;
		}
		if (normal == nullptr) {
			normal.reset(new pcl_type::Normal());
		}

		int const k = std::min(15, std::max(3, static_cast<int>(cloud->size()) - 1));

		if (method == NormalEstimateMethod::PclOmp) {
			pcl_type::KdTreePtr kdtree(new pcl_type::KdTree());
			kdtree->setInputCloud(cloud);

			pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud(cloud);
			ne.setSearchMethod(kdtree);
			ne.setKSearch(k);
			ne.setNumberOfThreads(0);
			ne.compute(*normal);
			return normal->size() == cloud->size();
		}

		if (method == NormalEstimateMethod::SmallGicpTbb) {
			normal->clear();
			normal->resize(cloud->size());
			detail::XyzNormalView view{cloud.get(), normal.get()};
			small_gicp::UnsafeKdTree<detail::XyzNormalView> tree(view, small_gicp::KdTreeBuilderTBB());
			tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size()),
			                  [&](tbb::blocked_range<size_t> const& range) {
				                  for (size_t i = range.begin(); i < range.end(); ++i) {
					                  estimate_one_pca_curvature(tree, view, k, i);
				                  }
			                  });
			return normal->size() == cloud->size();
		}

		return false;
	}
}
