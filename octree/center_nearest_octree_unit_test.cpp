#include "center_nearest_octree.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace
{

using Tree = Ddx::octree::CenterNearestOctree<double>;
using Vector3 = Tree::Vector3;
using Sample = Tree::SampleType;

int g_failures = 0;

void check(bool condition, char const* expr, char const* file, int line)
{
    if (!condition)
    {
        ++g_failures;
        std::cerr << "FAIL " << file << ":" << line << "  " << expr << std::endl;
    }
}

#define CHECK(cond) check((cond), #cond, __FILE__, __LINE__)

#define CHECK_NEAR(a, b, eps) \
    check(std::abs((a) - (b)) <= (eps), #a " ~= " #b, __FILE__, __LINE__)

void test_clear_and_reinsert()
{
    Tree tree(0.01);
    for (int i = 0; i < 500; ++i)
        tree.insert_point(Vector3(i * 0.001, i * 0.0007, -i * 0.0003));

    CHECK(tree.get_num_nodes() > 1);
    CHECK(tree.get_num_points_inserted() == 500);

    tree.clear();
    CHECK(tree.get_root_node() == nullptr);
    CHECK(tree.get_num_nodes() == 0);
    CHECK(tree.get_num_points_inserted() == 0);
    CHECK(tree.get_max_level() == Tree::kMaxAllowedLevel);

    tree.insert_point(Vector3(1.0, 2.0, 3.0));
    CHECK(tree.get_num_nodes() == 1);
    CHECK(tree.get_num_points_inserted() == 1);
}

void test_move_leaves_source_empty()
{
    Tree source(0.1);
    source.insert_point(Vector3(0.0, 0.0, 0.0));
    source.insert_point(Vector3(5.0, 0.0, 0.0));

    std::size_t const nodes = source.get_num_nodes();
    std::size_t const points = source.get_num_points_inserted();

    Tree dest = std::move(source);
    CHECK(source.get_root_node() == nullptr);
    CHECK(source.get_num_nodes() == 0);
    CHECK(dest.get_num_nodes() == nodes);
    CHECK(dest.get_num_points_inserted() == points);
}

void test_root_expansion()
{
    Tree tree(0.1);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    double const initial_size = tree.get_root_size();

    tree.insert_point(Vector3(10.0, 0.0, 0.0));
    CHECK(tree.get_root_size() > initial_size);
    CHECK(tree.get_num_nodes() > 1);
}

void test_subdivision_on_second_point_in_same_leaf()
{
    Tree tree(0.01);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(10.0, 0.0, 0.0));

    std::size_t prev = tree.get_num_nodes();
    bool subdivided = false;
    for (int i = 1; i <= 32; ++i)
    {
        tree.insert_point(Vector3(1e-4 * static_cast<double>(i),
            2e-4 * static_cast<double>(i),
            3e-4 * static_cast<double>(i)));
        if (tree.get_num_nodes() > prev)
        {
            subdivided = true;
            break;
        }
        prev = tree.get_num_nodes();
    }
    CHECK(subdivided);
}

void test_max_level_limits_subdivision()
{
    Tree unrestricted(0.01);
    Tree restricted(0.01);
    restricted.set_max_level(0);

    auto insert_both = [&](Vector3 const& p) {
        unrestricted.insert_point(p);
        restricted.insert_point(p);
    };

    insert_both(Vector3(0.0, 0.0, 0.0));
    insert_both(Vector3(10.0, 0.0, 0.0));
    for (int i = 1; i <= 32; ++i)
    {
        insert_both(Vector3(1e-4 * static_cast<double>(i),
            2e-4 * static_cast<double>(i),
            3e-4 * static_cast<double>(i)));
    }

    CHECK(restricted.get_num_nodes() <= unrestricted.get_num_nodes());
    CHECK(unrestricted.get_num_nodes() > restricted.get_num_nodes());
}

void test_set_max_level_clamps()
{
    Tree tree(0.1);
    tree.set_max_level(999);
    CHECK(tree.get_max_level() == Tree::kMaxAllowedLevel);

    tree.set_max_level(-5);
    CHECK(tree.get_max_level() == 0);
}

void test_distances_match_leaf_geometry()
{
    Tree tree(0.05);
    std::vector<Vector3> inputs = {
        Vector3(0.0, 0.0, 0.0),
        Vector3(3.0, 1.0, -2.0),
        Vector3(0.01, 0.02, 0.03),
        Vector3(-0.02, 0.04, 0.01),
        Vector3(2.5, 0.5, -1.5),
    };

    for (Vector3 const& p : inputs)
        tree.insert_point(p);

    std::vector<Vector3> reps;
    std::vector<double> distances;
    tree.collect_leaf_representatives(reps, &distances);
    CHECK(reps.size() == distances.size());
    CHECK(!reps.empty());

    Tree::Iterator iter = tree.get_iterator_for_root();
    std::size_t idx = 0;
    for (iter.first_leaf(); iter.current != nullptr; iter.next_leaf())
    {
        if (!iter.current->has_point)
            continue;

        Vector3 center;
        double size = 0.0;
        tree.node_center_and_size(iter, &center, &size);

        double const expected = (reps[idx] - center).norm();
        CHECK_NEAR(distances[idx], expected, 1e-9);
        CHECK_NEAR(distances[idx], std::sqrt(iter.current->dist_to_center_sq), 1e-12);
        ++idx;
    }
    CHECK(idx == reps.size());
}

void test_iterator_center_matches_descent()
{
    Tree tree(0.08);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(6.0, 0.0, 0.0));
    tree.insert_point(Vector3(0.04, -0.03, 0.02));
    tree.insert_point(Vector3(0.05, -0.02, 0.03));

    Tree::Iterator iter = tree.get_iterator_for_root();
    for (iter.first_leaf(); iter.current != nullptr; iter.next_leaf())
    {
        Vector3 center;
        double size = 0.0;
        tree.node_center_and_size(iter, &center, &size);

        Tree::Iterator walk = tree.get_iterator_for_root();
        Vector3 walk_center = tree.get_root_center();
        double walk_size = tree.get_root_size();

        for (int depth = 0; depth < iter.level; ++depth)
        {
            int const octant =
                static_cast<int>((iter.path >> ((iter.level - depth - 1) * 3)) & 7);
            double const offset = walk_size / 4.0;
            for (int j = 0; j < 3; ++j)
                walk_center[j] += ((octant & (1 << j)) != 0) ? offset : -offset;
            walk_size /= 2.0;
        }

        CHECK_NEAR(center.x(), walk_center.x(), 1e-12);
        CHECK_NEAR(center.y(), walk_center.y(), 1e-12);
        CHECK_NEAR(center.z(), walk_center.z(), 1e-12);
        CHECK_NEAR(size, walk_size, 1e-12);
    }
}

void test_insert_point_return_value()
{
    Tree tree(0.2);
    CHECK(tree.insert_point(Vector3(0.0, 0.0, 0.0)) == true);
    CHECK(tree.insert_point(Vector3(0.09, 0.0, 0.0)) == false);

    Tree tree2(0.2);
    tree2.insert_point(Vector3(1.0, 1.0, 1.0));
    CHECK(tree2.insert_point(Vector3(1.0, 1.0, 1.0)) == false);

    Tree tree3(0.1);
    CHECK(tree3.insert_point(Vector3(0.0, 0.0, 0.0)) == true);
    CHECK(tree3.insert_point(Vector3(10.0, 0.0, 0.0)) == true);
}

void test_boundary_point_on_center()
{
    Tree tree(0.2);
    tree.insert_point(Vector3(1.0, 1.0, 1.0));
    tree.insert_point(Vector3(1.0, 1.0, 1.0));

    std::vector<Vector3> reps;
    tree.collect_leaf_representatives(reps);
    CHECK(reps.size() == 1);
    CHECK_NEAR(reps[0].x(), 1.0, 1e-12);
}

void test_active_point_count()
{
    Tree tree(0.1);
    CHECK(tree.get_num_active_points() == 0);

    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    CHECK(tree.get_num_active_points() == 1);

    tree.insert_point(Vector3(0.05, 0.0, 0.0));
    CHECK(tree.get_num_active_points() == 1);

    tree.insert_point(Vector3(5.0, 0.0, 0.0));
    CHECK(tree.get_num_active_points() == 2);
}

void test_collect_fresh_filters_stale()
{
    Tree tree(0.2);
    tree.set_max_stale_age(2.0);

    tree.insert_sample(Sample(Vector3(0.0, 0.0, 0.0), 0.0));
    tree.insert_sample(Sample(Vector3(10.0, 0.0, 0.0), 0.0));
    CHECK(tree.get_num_active_points() == 2);

    tree.insert_sample(Sample(Vector3(0.01, 0.0, 0.0), 1.0));
    tree.insert_sample(Sample(Vector3(0.02, 0.0, 0.0), 2.0));
    CHECK(tree.get_num_active_points() == 2);

    std::vector<Vector3> fresh;
    CHECK(tree.collect_leaf_representatives_fresh(4.0, fresh) == 1);
    CHECK(fresh.size() == 1);
    if (!fresh.empty())
        CHECK(fresh[0].x() < 1.0);

    std::vector<Vector3> all;
    tree.collect_leaf_representatives(all);
    CHECK(all.size() == 2);
    CHECK(tree.get_num_active_points() == 2);
}

void test_collect_near_or_fresh_keeps_visible_region()
{
    Tree tree(0.2);
    tree.set_max_stale_age(1.0);

    tree.insert_sample(Sample(Vector3(0.0, 0.0, 0.0), 0.0));
    tree.insert_sample(Sample(Vector3(20.0, 0.0, 0.0), 0.0));
    CHECK(tree.get_num_active_points() == 2);

    std::vector<Vector3> display;
    CHECK(tree.collect_leaf_representatives_near_or_fresh(
        Vector3(0.0, 0.0, 0.0), 1.0, 3.0, display) == 1);
    CHECK(display.size() == 1);
    if (!display.empty())
        CHECK_NEAR(display[0].x(), 0.0, 1e-9);

    std::vector<Vector3> all;
    tree.collect_leaf_representatives(all);
    CHECK(all.size() == 2);
}

void test_collect_in_sphere()
{
    Tree tree(0.2);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(3.0, 0.0, 0.0));
    tree.insert_point(Vector3(8.0, 0.0, 0.0));
    CHECK(tree.get_num_active_points() == 3);

    std::vector<Vector3> inside;
    CHECK(tree.collect_leaf_representatives_in_sphere(
        Vector3(0.0, 0.0, 0.0), 5.0, inside) == 2);
    CHECK(inside.size() == 2);
    CHECK(tree.get_num_active_points() == 3);

    std::vector<Vector3> all;
    tree.collect_all_leaf_representatives(all);
    CHECK(all.size() == 3);
}

void test_collect_all_matches_legacy_collect()
{
    Tree tree(0.1);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(50.0, 0.0, 0.0));

    std::vector<Sample> all_samples;
    std::vector<Sample> legacy_samples;
    CHECK(tree.collect_all_leaf_samples(all_samples) == 2);
    tree.collect_leaf_samples(legacy_samples);
    CHECK(all_samples.size() == legacy_samples.size());
    CHECK(tree.get_num_nodes() > 1);
}

void test_remove_before_time()
{
    Tree tree(0.2);
    tree.insert_sample(Sample(Vector3(0.0, 0.0, 0.0), 0.0));
    tree.insert_sample(Sample(Vector3(10.0, 0.0, 0.0), 0.0));
    // Same leaf as origin (leaf size 0.2): refreshes activity timestamp to 5.0.
    tree.insert_sample(Sample(Vector3(0.01, 0.0, 0.0), 5.0));
    CHECK(tree.get_num_active_points() == 2);

    CHECK(tree.remove_before_time(3.0) == 1);
    CHECK(tree.get_num_active_points() == 1);

    std::vector<Vector3> reps;
    tree.collect_all_leaf_representatives(reps);
    CHECK(reps.size() == 1);
    if (!reps.empty())
        CHECK(reps[0].x() < 1.0);
}

void test_remove_outside_sphere()
{
    Tree tree(0.2);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(3.0, 0.0, 0.0));
    tree.insert_point(Vector3(8.0, 0.0, 0.0));
    CHECK(tree.get_num_active_points() == 3);

    CHECK(tree.remove_outside_sphere(Vector3(0.0, 0.0, 0.0), 5.0) == 1);
    CHECK(tree.get_num_active_points() == 2);

    std::vector<Vector3> reps;
    tree.collect_all_leaf_representatives(reps);
    CHECK(reps.size() == 2);
}

void test_remove_stale_inside_sphere()
{
    Tree tree(0.2);
    tree.insert_sample(Sample(Vector3(0.0, 0.0, 0.0), 0.0));
    tree.insert_sample(Sample(Vector3(0.5, 0.0, 0.0), 5.0));
    tree.insert_sample(Sample(Vector3(10.0, 0.0, 0.0), 0.0));
    CHECK(tree.get_num_active_points() == 3);

    CHECK(tree.remove_stale_inside_sphere(Vector3(0.0, 0.0, 0.0), 2.0, 3.0) == 1);
    CHECK(tree.get_num_active_points() == 2);

    std::vector<Vector3> reps;
    tree.collect_all_leaf_representatives(reps);
    CHECK(reps.size() == 2);
    if (reps.size() == 2)
    {
        double const x0 = std::min(reps[0].x(), reps[1].x());
        double const x1 = std::max(reps[0].x(), reps[1].x());
        CHECK(x0 < 1.0);
        CHECK(x1 > 5.0);
    }
}

void test_remove_reclaims_tree()
{
    Tree tree(0.1);
    tree.insert_point(Vector3(0.0, 0.0, 0.0));
    tree.insert_point(Vector3(50.0, 0.0, 0.0));
    double const expanded_size = tree.get_root_size();
    std::size_t const nodes_before = tree.get_num_nodes();
    CHECK(expanded_size > 0.1);
    CHECK(nodes_before > 1);

    tree.remove_outside_sphere(Vector3(0.0, 0.0, 0.0), 1.0);

    CHECK(tree.get_root_size() < expanded_size);
    CHECK(tree.get_num_nodes() < nodes_before);
    CHECK(tree.get_num_active_points() == 1);
}

void test_remove_all_clears_tree()
{
    Tree tree(0.2);
    tree.insert_sample(Sample(Vector3(0.0, 0.0, 0.0), 0.0));
    CHECK(tree.get_num_nodes() == 1);

    tree.remove_before_time(1.0);

    CHECK(tree.get_root_node() == nullptr);
    CHECK(tree.get_num_nodes() == 0);
    CHECK(tree.get_num_active_points() == 0);
}

void test_representatives_are_subset_of_inputs()
{
    Tree tree(0.03);
    std::vector<Vector3> inputs = {
        Vector3(0.0, 0.0, 0.0),
        Vector3(2.0, 0.0, 0.0),
        Vector3(0.01, 0.01, 0.0),
        Vector3(0.02, -0.01, 0.01),
        Vector3(-0.01, 0.02, 0.0),
    };

    auto key = [](Vector3 const& v) {
        return std::make_tuple(v.x(), v.y(), v.z());
    };

    std::set<decltype(key(Vector3()))> input_keys;
    for (Vector3 const& p : inputs)
    {
        tree.insert_point(p);
        input_keys.insert(key(p));
    }

    std::vector<Vector3> reps;
    tree.collect_leaf_representatives(reps);
    for (Vector3 const& r : reps)
        CHECK(input_keys.count(key(r)) == 1);
}

} // namespace

int main()
{
    test_clear_and_reinsert();
    test_move_leaves_source_empty();
    test_root_expansion();
    test_subdivision_on_second_point_in_same_leaf();
    test_max_level_limits_subdivision();
    test_set_max_level_clamps();
    test_distances_match_leaf_geometry();
    test_iterator_center_matches_descent();
    test_insert_point_return_value();
    test_boundary_point_on_center();
    test_active_point_count();
    test_collect_fresh_filters_stale();
    test_collect_near_or_fresh_keeps_visible_region();
    test_collect_in_sphere();
    test_collect_all_matches_legacy_collect();
    test_remove_before_time();
    test_remove_outside_sphere();
    test_remove_stale_inside_sphere();
    test_remove_reclaims_tree();
    test_remove_all_clears_tree();
    test_representatives_are_subset_of_inputs();

    if (g_failures == 0)
    {
        std::cout << "All center_nearest_octree unit tests passed." << std::endl;
        return 0;
    }

    std::cerr << g_failures << " test(s) failed." << std::endl;
    return 1;
}
