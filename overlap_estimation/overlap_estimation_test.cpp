#include <overlap_estimation/overlap_estimation.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdlib>
#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace
{

int g_failures = 0;

void check(bool condition, char const* expr, char const* file, int line)
{
    if(!condition)
    {
        ++g_failures;
        std::cerr << "FAIL " << file << ":" << line
                  << "  " << expr << std::endl;
    }
}

#define CHECK(cond) check((cond), #cond, __FILE__, __LINE__)

#define CHECK_NEAR(a, b, eps) \
    check(std::abs((a) - (b)) <= (eps), #a " ~= " #b, __FILE__, __LINE__)

pcl::PointCloud<pcl::PointXYZ>::Ptr make_cloud(
    std::vector<std::array<float, 3>> const& points)
{
    auto cloud =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>());

    cloud->reserve(points.size());

    for(auto const& p : points)
    {
        pcl::PointXYZ pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        cloud->push_back(pt);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

void test_empty_source_returns_zero()
{
    auto target = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto empty =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>());

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(empty);

    CHECK(result.overlap_ratio == 0.f);
    CHECK(result.overlap_idx_src.empty());
    CHECK(result.overlap_idx_tgt.empty());
}

void test_without_set_target_returns_zero()
{
    auto source = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    CHECK(result.overlap_ratio == 0.f);
    CHECK(result.overlap_idx_src.empty());
}

void test_empty_target_returns_zero_overlap()
{
    auto empty =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>());

    auto source = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(empty);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    CHECK(result.overlap_ratio == 0.f);
    CHECK(result.overlap_idx_src.empty());
}

void test_identical_clouds_full_overlap()
{
    auto cloud = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(cloud);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(cloud);

    CHECK_NEAR(result.overlap_ratio, 1.f, 1e-5f);
    CHECK(result.overlap_idx_src.size() == 3);
    CHECK(result.overlap_idx_tgt.size() == 3);
}

void test_partial_overlap_after_shift()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
        {1.02f, 0.f, 0.f},
        {10.f, 0.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    CHECK_NEAR(result.overlap_ratio, 2.f / 3.f, 1e-5f);
    CHECK(result.overlap_idx_src.size() == 2);
}

void test_zero_overlap_far_apart()
{
    auto target = make_cloud({{0.f, 0.f, 0.f}});
    auto source = make_cloud({{100.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    CHECK(result.overlap_ratio == 0.f);
    CHECK(result.overlap_idx_src.empty());
}

void test_large_thresh_crosses_two_voxels()
{
    auto target = make_cloud({{0.59f, 0.f, 0.f}});
    auto source = make_cloud({{-0.01f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.6f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    CHECK(result.overlap_idx_src.size() == 1);
    CHECK(result.overlap_idx_tgt.size() == 1);
    CHECK_NEAR(result.overlap_ratio, 1.f, 1e-5f);
}

void test_large_thresh_old_radius_one_would_miss()
{
    auto target = make_cloud({{0.59f, 0.f, 0.f}});
    auto source = make_cloud({{-0.01f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.6f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(
            source,
            target);

    CHECK(result.overlap_idx_src.size() == 1);
    CHECK_NEAR(result.overlap_ratio, 1.f, 1e-5f);
}

void test_mismatched_target_size_returns_zero()
{
    auto target = make_cloud({{0.f, 0.f, 0.f}});
    auto wrong_target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
    });
    auto source = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(
            source,
            wrong_target);

    CHECK(result.overlap_ratio == 0.f);
    CHECK(result.overlap_idx_src.empty());
}

void test_extract_cloud_indices()
{
    auto cloud = make_cloud({
        {1.f, 2.f, 3.f},
        {4.f, 5.f, 6.f},
        {7.f, 8.f, 9.f},
    });

    std::vector<int> indices = {0, 2};

    auto extracted =
        Ddx::OverlapEstimator::ExtractCloud<pcl::PointXYZ>(
            cloud,
            indices);

    CHECK(extracted->size() == 2);
    CHECK_NEAR(extracted->points[0].x, 1.f, 1e-5f);
    CHECK_NEAR(extracted->points[1].z, 9.f, 1e-5f);
}

void test_overlap_ratio_matches_compute_overlap()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
        {1.02f, 0.f, 0.f},
        {10.f, 0.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto result =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    float ratio =
        estimator.ComputeOverlapRatio<pcl::PointXYZ>(source);

    CHECK_NEAR(ratio, result.overlap_ratio, 1e-5f);
    CHECK_NEAR(ratio, 2.f / 3.f, 1e-5f);
}

void test_overlap_ratio_without_set_target()
{
    auto source = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    float ratio =
        estimator.ComputeOverlapRatio<pcl::PointXYZ>(source);

    CHECK(ratio == 0.f);
}

void test_overlap_ratio_mismatched_target()
{
    auto target = make_cloud({{0.f, 0.f, 0.f}});
    auto wrong_target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
    });
    auto source = make_cloud({{0.f, 0.f, 0.f}});

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    float ratio =
        estimator.ComputeOverlapRatio<pcl::PointXYZ>(
            source,
            wrong_target);

    CHECK(ratio == 0.f);
}

void test_overlap_region_src_tgt_differ_when_many_to_one()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {0.05f, 0.f, 0.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
        {0.03f, 0.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto paired =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    auto region =
        estimator.ComputeOverlapRegion<pcl::PointXYZ>(source);

    CHECK(paired.overlap_idx_src.size() == 2);
    CHECK(paired.overlap_idx_tgt.size() == 2);
    CHECK(paired.overlap_idx_tgt[0] == paired.overlap_idx_tgt[1]);
    CHECK(region.region_idx_src.size() == 2);
    CHECK(region.region_idx_tgt.size() == 2);
    CHECK(region.region_idx_tgt[0] != region.region_idx_tgt[1]);
}

void test_overlap_region_includes_all_nearby_targets()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {0.05f, 0.f, 0.f},
        {10.f, 0.f, 0.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto paired =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    auto region =
        estimator.ComputeOverlapRegion<pcl::PointXYZ>(source);

    CHECK(paired.overlap_idx_tgt.size() == 1);
    CHECK(region.region_idx_tgt.size() == 2);
    CHECK_NEAR(region.overlap_ratio_src, 1.f, 1e-5f);
    CHECK_NEAR(region.overlap_ratio_tgt, 2.f / 3.f, 1e-5f);
}

void test_overlap_region_src_matches_paired_src_indices()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
        {1.02f, 0.f, 0.f},
        {10.f, 0.f, 0.f},
    });

    Ddx::OverlapEstimator estimator(0.5f, 0.3f);
    estimator.SetTargetCloud<pcl::PointXYZ>(target);

    auto paired =
        estimator.ComputeOverlap<pcl::PointXYZ>(source);

    auto region =
        estimator.ComputeOverlapRegion<pcl::PointXYZ>(source);

    CHECK(paired.overlap_idx_src == region.region_idx_src);
    CHECK_NEAR(
        region.overlap_ratio_src,
        paired.overlap_ratio,
        1e-5f);
}

}  // namespace

int main()
{
    test_empty_source_returns_zero();
    test_without_set_target_returns_zero();
    test_empty_target_returns_zero_overlap();
    test_identical_clouds_full_overlap();
    test_partial_overlap_after_shift();
    test_zero_overlap_far_apart();
    test_large_thresh_crosses_two_voxels();
    test_large_thresh_old_radius_one_would_miss();
    test_mismatched_target_size_returns_zero();
    test_extract_cloud_indices();
    test_overlap_ratio_matches_compute_overlap();
    test_overlap_ratio_without_set_target();
    test_overlap_ratio_mismatched_target();
    test_overlap_region_src_tgt_differ_when_many_to_one();
    test_overlap_region_includes_all_nearby_targets();
    test_overlap_region_src_matches_paired_src_indices();

    if(g_failures == 0)
    {
        std::cout
            << "All overlap_estimation unit tests passed."
            << std::endl;
        return 0;
    }

    std::cerr << g_failures << " test(s) failed." << std::endl;
    return 1;
}
