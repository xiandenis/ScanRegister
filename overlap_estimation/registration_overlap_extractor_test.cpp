#include <overlap_estimation/registration_overlap_extractor.hpp>

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

Eigen::Matrix4f identity_guess()
{
    return Eigen::Matrix4f::Identity().eval();
}

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

pcl::PointCloud<pcl::Normal>::Ptr make_normals(
    std::vector<std::array<float, 3>> const& normals)
{
    auto cloud =
        pcl::PointCloud<pcl::Normal>::Ptr(
            new pcl::PointCloud<pcl::Normal>());

    cloud->reserve(normals.size());

    for(auto const& n : normals)
    {
        pcl::Normal normal;
        normal.normal_x = n[0];
        normal.normal_y = n[1];
        normal.normal_z = n[2];
        cloud->push_back(normal);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

void test_not_ready_returns_empty()
{
    Ddx::RegistrationOverlapExtractor extractor;

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.empty());
    CHECK(result.target_indices.empty());
    CHECK(result.overlap_ratio_src == 0.f);
    CHECK(result.overlap_ratio_tgt == 0.f);
}

void test_identical_clouds_full_overlap()
{
    auto cloud = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
    });

    auto normals = make_normals({
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
    });

    Ddx::RegistrationOverlapExtractor::Params params;
    params.radius = 0.3f;
    params.max_neighbors = 10;
    params.normal_consistency_thresh = 0.2f;

    Ddx::RegistrationOverlapExtractor extractor(params);
    extractor.SetSourceCloud(cloud, normals);
    extractor.SetTargetCloud(cloud, normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.size() == 3);
    CHECK(result.target_indices.size() == 3);
    CHECK_NEAR(result.overlap_ratio_src, 1.f, 1e-5f);
    CHECK_NEAR(result.overlap_ratio_tgt, 1.f, 1e-5f);

    auto extracted =
        Ddx::RegistrationOverlapExtractor::ExtractCloud<pcl::PointXYZ>(
            cloud,
            result.source_indices);

    CHECK(extracted->size() == 3);
    CHECK_NEAR(extracted->points[0].x, 0.f, 1e-5f);
    CHECK_NEAR(extracted->points[1].x, 1.f, 1e-5f);
}

void test_opposite_normals_filtered_out()
{
    auto source = make_cloud({{0.f, 0.f, 0.f}});
    auto source_normals = make_normals({{0.f, 0.f, 1.f}});

    auto target = make_cloud({{0.01f, 0.f, 0.f}});
    auto target_normals = make_normals({{0.f, 0.f, -1.f}});

    Ddx::RegistrationOverlapExtractor::Params params;
    params.radius = 0.3f;

    Ddx::RegistrationOverlapExtractor extractor(params);
    extractor.SetSourceCloud(source, source_normals);
    extractor.SetTargetCloud(target, target_normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.empty());
    CHECK(result.target_indices.empty());
    CHECK(result.overlap_ratio_src == 0.f);
    CHECK(result.overlap_ratio_tgt == 0.f);
}

void test_partial_overlap_after_shift()
{
    auto target = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f},
    });

    auto target_normals = make_normals({
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
    });

    auto source = make_cloud({
        {0.02f, 0.f, 0.f},
        {1.02f, 0.f, 0.f},
        {10.f, 0.f, 0.f},
    });

    auto source_normals = make_normals({
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
    });

    Ddx::RegistrationOverlapExtractor::Params params;
    params.radius = 0.3f;

    Ddx::RegistrationOverlapExtractor extractor(params);
    extractor.SetSourceCloud(source, source_normals);
    extractor.SetTargetCloud(target, target_normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.size() == 2);
    CHECK_NEAR(result.overlap_ratio_src, 2.f / 3.f, 1e-5f);
    CHECK(result.target_indices.size() >= 2);
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
        Ddx::RegistrationOverlapExtractor::ExtractCloud<pcl::PointXYZ>(
            cloud,
            indices);

    CHECK(extracted->size() == 2);
    CHECK_NEAR(extracted->points[0].x, 1.f, 1e-5f);
    CHECK_NEAR(extracted->points[1].z, 9.f, 1e-5f);
}

void test_mismatched_normal_size_not_ready()
{
    auto cloud = make_cloud({{0.f, 0.f, 0.f}});
    auto normals = make_normals({
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
    });

    Ddx::RegistrationOverlapExtractor extractor;
    extractor.SetSourceCloud(cloud, normals);
    extractor.SetTargetCloud(cloud, normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.empty());
}

void test_external_trees()
{
    auto cloud = make_cloud({
        {0.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
    });

    auto normals = make_normals({
        {0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f},
    });

    auto source_tree =
        pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>());

    auto target_tree =
        pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>());

    source_tree->setInputCloud(cloud);
    target_tree->setInputCloud(cloud);

    Ddx::RegistrationOverlapExtractor::Params params;
    params.radius = 0.3f;

    Ddx::RegistrationOverlapExtractor extractor(params);
    extractor.SetSourceTree(source_tree);
    extractor.SetTargetTree(target_tree);
    extractor.SetSourceCloud(cloud, normals);
    extractor.SetTargetCloud(cloud, normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.size() == 2);
    CHECK(result.target_indices.size() == 2);
    CHECK_NEAR(result.overlap_ratio_src, 1.f, 1e-5f);
}

void test_clear_external_tree_rebuilds_owned()
{
    auto cloud = make_cloud({{0.f, 0.f, 0.f}});
    auto normals = make_normals({{0.f, 0.f, 1.f}});

    auto tree =
        pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>());

    tree->setInputCloud(cloud);

    Ddx::RegistrationOverlapExtractor::Params params;
    params.radius = 0.3f;

    Ddx::RegistrationOverlapExtractor extractor(params);
    extractor.SetSourceTree(tree);
    extractor.SetTargetTree(tree);
    extractor.SetSourceCloud(cloud, normals);
    extractor.SetTargetCloud(cloud, normals);
    extractor.ClearSourceTree();
    extractor.ClearTargetTree();

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.size() == 1);
    CHECK(result.target_indices.size() == 1);
}

void test_stale_external_tree_rejected()
{
    auto cloud_a = make_cloud({{0.f, 0.f, 0.f}});
    auto cloud_b = make_cloud({{1.f, 0.f, 0.f}});
    auto normals = make_normals({{0.f, 0.f, 1.f}});

    auto tree =
        pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>());

    tree->setInputCloud(cloud_a);

    Ddx::RegistrationOverlapExtractor extractor;
    extractor.SetSourceTree(tree);
    extractor.SetTargetTree(tree);
    extractor.SetSourceCloud(cloud_b, normals);
    extractor.SetTargetCloud(cloud_b, normals);

    auto result =
        extractor.Extract(identity_guess());

    CHECK(result.source_indices.empty());
    CHECK(result.target_indices.empty());
}

void test_extract_cloud_skips_invalid_indices()
{
    auto cloud = make_cloud({
        {1.f, 2.f, 3.f},
        {4.f, 5.f, 6.f},
    });

    std::vector<int> indices = {0, 99, -1};

    auto extracted =
        Ddx::RegistrationOverlapExtractor::ExtractCloud<pcl::PointXYZ>(
            cloud,
            indices);

    CHECK(extracted->size() == 1);
    CHECK_NEAR(extracted->points[0].x, 1.f, 1e-5f);
}

}  // namespace

int main()
{
    test_not_ready_returns_empty();
    test_identical_clouds_full_overlap();
    test_opposite_normals_filtered_out();
    test_partial_overlap_after_shift();
    test_extract_cloud_indices();
    test_mismatched_normal_size_not_ready();
    test_external_trees();
    test_clear_external_tree_rebuilds_owned();
    test_stale_external_tree_rejected();
    test_extract_cloud_skips_invalid_indices();

    if(g_failures == 0)
    {
        std::cout
            << "All registration_overlap_extractor unit tests passed."
            << std::endl;
        return 0;
    }

    std::cerr << g_failures << " test(s) failed." << std::endl;
    return 1;
}
