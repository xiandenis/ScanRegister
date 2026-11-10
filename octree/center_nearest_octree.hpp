#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace Ddx
{
namespace octree
{

/** Point sample: position, normal, color, and reflectance intensity. */
template <typename Scalar = double>
struct Sample
{
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Vector3 position = Vector3::Zero();
    Vector3 normal = Vector3::Zero();
    Scalar intensity = Scalar(0);
    std::uint8_t color_r = 0;
    std::uint8_t color_g = 0;
    std::uint8_t color_b = 0;
    double timestamp = 0.0;

    Sample() = default;

    explicit Sample(Vector3 const& pos, double ts = 0.0)
        : position(pos)
        , timestamp(ts)
    {
    }

    Sample(Vector3 const& pos, Vector3 const& nrm, Scalar intens,
        std::uint8_t r, std::uint8_t g, std::uint8_t b, double ts = 0.0)
        : position(pos)
        , normal(nrm)
        , intensity(intens)
        , color_r(r)
        , color_g(g)
        , color_b(b)
        , timestamp(ts)
    {
    }

    void set_color(std::uint8_t r, std::uint8_t g, std::uint8_t b)
    {
        color_r = r;
        color_g = g;
        color_b = b;
    }
};

/**
 * Regular octree (each node has zero or eight children).
 * Each node stores at most one representative point and its Euclidean distance
 * to the node center. On insert, the point is routed to a leaf; when
 * use_center_nearest is enabled (default), the new point replaces the stored
 * one only if it is closer to that leaf center; when disabled, the first point
 * stored in a leaf keeps its representative and later inserts are ignored.
 *
 * The first point creates a min_leaf_size cube centered on that point.
 * When a later point falls outside the root, the root doubles (refer_octree
 * style) until it just contains the new point — no global rebuild, no forced
 * subdivision on the far point's branch.
 *
 * Subdivision to min_leaf_size happens only when two points land in the same
 * leaf cell and that cell is still larger than min_leaf_size. An isolated far
 * point stays in the first empty leaf on its descent path (a coarse cell).
 *
 * Geometry types: Eigen only.
 *
 * Maximum depth defaults to 20 (root = level 0),
 * iterator path limit (3 bits per level).
 *
 * Display / streaming: each Sample carries a double timestamp (last activity time).
 * Any insert that lands in an occupied leaf refreshes that leaf's timestamp even when
 * the geometric representative is kept unchanged (farther than the current one).
 * Use collect_leaf_samples* to query without deleting; use remove_* to physically
 * erase leaf representatives and reclaim_tree() to collapse empty branches.
 * set_max_stale_age controls the freshness window used by time-based collectors.
 */
template <typename Scalar = double>
class CenterNearestOctree
{
public:
    static constexpr int kMaxAllowedLevel = 20;

    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using SampleType = Sample<Scalar>;

    struct Node
    {
        Node* children = nullptr;
        Node* parent = nullptr;

        SampleType sample;
        Scalar dist_to_center_sq = std::numeric_limits<Scalar>::max();
        bool has_point = false;

        ~Node()
        {
            delete[] children;
        }
    };

    struct Iterator
    {
        Node* current = nullptr;
        Node* root = nullptr;
        std::uint64_t path = 0;
        std::uint8_t level = 0;

        Node* first_node()
        {
            current = root;
            level = 0;
            path = 0;
            return current;
        }

        Node* first_leaf()
        {
            first_node();
            while (current != nullptr && current->children != nullptr)
            {
                current = current->children;
                level = static_cast<std::uint8_t>(level + 1);
                path = path << 3;
            }
            return current;
        }

        Node* next_node()
        {
            if (current == nullptr)
                return nullptr;

            if (current->children != nullptr)
            {
                current = current->children;
                level = static_cast<std::uint8_t>(level + 1);
                path = path << 3;
                return current;
            }

            return next_branch();
        }

        Node* next_branch()
        {
            if (current == nullptr || current->parent == nullptr)
            {
                current = nullptr;
                return nullptr;
            }

            if (current - current->parent->children == 7)
            {
                current = current->parent;
                level = static_cast<std::uint8_t>(level - 1);
                path = path >> 3;
                return next_branch();
            }

            current += 1;
            path += 1;
            return current;
        }

        Node* next_leaf()
        {
            if (current == nullptr)
                return nullptr;

            if (current->children != nullptr)
            {
                while (current->children != nullptr)
                {
                    current = current->children;
                    level = static_cast<std::uint8_t>(level + 1);
                    path = path << 3;
                }
                return current;
            }

            next_branch();
            if (current == nullptr)
                return nullptr;

            while (current->children != nullptr)
            {
                current = current->children;
                level = static_cast<std::uint8_t>(level + 1);
                path = path << 3;
            }
            return current;
        }

        Iterator descend(int octant) const
        {
            Iterator iter = *this;
            iter.current = iter.current->children + octant;
            iter.level = static_cast<std::uint8_t>(iter.level + 1);
            iter.path = (iter.path << 3) | static_cast<std::uint64_t>(octant);
            return iter;
        }

        Iterator ascend() const
        {
            Iterator iter;
            iter.root = root;
            if (current == nullptr || current->parent == nullptr)
                return iter;

            iter.current = current->parent;
            iter.path = path >> 3;
            iter.level = static_cast<std::uint8_t>(level - 1);
            return iter;
        }
    };

public:
    explicit CenterNearestOctree(Scalar min_leaf_size = Scalar(0.01))
        : min_leaf_size_(min_leaf_size)
    {
        clear();
    }

    ~CenterNearestOctree()
    {
        clear();
    }

    CenterNearestOctree(const CenterNearestOctree&) = delete;
    CenterNearestOctree& operator=(const CenterNearestOctree&) = delete;

    CenterNearestOctree(CenterNearestOctree&& other) noexcept
        : root_(other.root_)
        , root_center_(other.root_center_)
        , root_size_(other.root_size_)
        , min_leaf_size_(other.min_leaf_size_)
        , use_center_nearest_(other.use_center_nearest_)
        , max_stale_age_(other.max_stale_age_)
        , max_level_(other.max_level_)
        , num_nodes_(other.num_nodes_)
        , num_points_(other.num_points_)
        , num_active_points_(other.num_active_points_)
    {
        other.root_ = nullptr;
        other.clear_counters_only();
    }

    CenterNearestOctree& operator=(CenterNearestOctree&& other) noexcept
    {
        if (this != &other)
        {
            clear();
            root_ = other.root_;
            root_center_ = other.root_center_;
            root_size_ = other.root_size_;
            min_leaf_size_ = other.min_leaf_size_;
            use_center_nearest_ = other.use_center_nearest_;
            max_stale_age_ = other.max_stale_age_;
            max_level_ = other.max_level_;
            num_nodes_ = other.num_nodes_;
            num_points_ = other.num_points_;
            num_active_points_ = other.num_active_points_;
            other.root_ = nullptr;
            other.clear_counters_only();
        }
        return *this;
    }

    void clear()
    {
        delete root_;
        root_ = nullptr;
        root_size_ = Scalar(0);
        root_center_.setZero();
        max_level_ = kMaxAllowedLevel;
        clear_counters_only();
    }

    void set_min_leaf_size(Scalar min_leaf_size)
    {
        if (min_leaf_size <= Scalar(0))
            throw std::invalid_argument("CenterNearestOctree: min_leaf_size must be positive");
        min_leaf_size_ = min_leaf_size;
    }

    Scalar get_min_leaf_size() const
    {
        return min_leaf_size_;
    }

    void set_max_level(int max_level)
    {
        max_level_ = std::max(0, std::min(kMaxAllowedLevel, max_level));
    }

    int get_max_level() const
    {
        return max_level_;
    }

    /** When true, keep the sample closest to the leaf center; when false, keep the first sample in each leaf. */
    void set_use_center_nearest(bool use_center_nearest)
    {
        use_center_nearest_ = use_center_nearest;
    }

    bool get_use_center_nearest() const
    {
        return use_center_nearest_;
    }

    Node const* get_root_node() const
    {
        return root_;
    }

    Vector3 const& get_root_center() const
    {
        return root_center_;
    }

    Scalar get_root_size() const
    {
        return root_size_;
    }

    std::size_t get_num_nodes() const
    {
        return num_nodes_;
    }

    /** Total insert_sample calls (includes rejected points). */
    std::size_t get_num_points_inserted() const
    {
        return num_points_;
    }

    /** Leaf nodes that currently hold a representative sample. */
    std::size_t get_num_active_points() const
    {
        return num_active_points_;
    }

    /** Max allowed age (same time unit as Sample::timestamp) before a leaf is stale. */
    void set_max_stale_age(double max_stale_age)
    {
        if (max_stale_age < 0.0)
            throw std::invalid_argument("CenterNearestOctree: max_stale_age must be non-negative");
        max_stale_age_ = max_stale_age;
    }

    double get_max_stale_age() const
    {
        return max_stale_age_;
    }

    Iterator get_iterator_for_root() const
    {
        if (root_ == nullptr)
            throw std::logic_error("CenterNearestOctree: iterator requested on empty tree");

        Iterator iter;
        iter.root = root_;
        iter.first_node();
        return iter;
    }

    void node_center_and_size(Iterator const& iter, Vector3* center, Scalar* size) const
    {
        if (center == nullptr || size == nullptr)
            throw std::invalid_argument("CenterNearestOctree: null output pointer");

        *center = root_center_;
        *size = root_size_;
        for (int i = 0; i < iter.level; ++i)
        {
            int const octant = static_cast<int>((iter.path >> ((iter.level - i - 1) * 3)) & 7);
            Scalar const offset = *size / Scalar(4);
            for (int j = 0; j < 3; ++j)
                (*center)[j] += ((octant & (1 << j)) != 0) ? offset : -offset;
            *size /= Scalar(2);
        }
    }

    /**
     * @return true if the sample is written to its target leaf; false if discarded
     *         (closer representative already exists when use_center_nearest is enabled,
     *         or any representative already exists when it is disabled).
     */
    bool insert_sample(SampleType const& sample)
    {
        Vector3 const& point = sample.position;
        if (root_ == nullptr)
        {
            ensure_root(point);
            ++num_points_;
            return insert_sample_descend(sample, root_, root_center_, root_size_, 0);
        }

        while (!is_inside_root(point))
            expand_root_for_point(point);

        ++num_points_;
        return insert_sample_descend(sample, root_, root_center_, root_size_, 0);
    }

    /** Convenience: insert position only; normal/color/intensity default to zero. */
    bool insert_point(Vector3 const& point)
    {
        return insert_sample(SampleType(point));
    }

    void insert_points(std::vector<SampleType> const& samples)
    {
        for (SampleType const& s : samples)
            (void)insert_sample(s);
    }

    /** Collect every leaf representative (full Sample). */
    std::size_t collect_all_leaf_samples(std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        return collect_leaf_samples_impl(
            [](Node const&) { return true; }, samples, distances);
    }

    /** Collect leaf representatives whose timestamp >= reference_time - max_stale_age. */
    std::size_t collect_leaf_samples_fresh(double reference_time,
        std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        double const cutoff = reference_time - max_stale_age_;
        return collect_leaf_samples_impl(
            [cutoff](Node const& node) { return node.sample.timestamp >= cutoff; },
            samples, distances);
    }

    /** Collect leaf representatives inside the sphere (distance <= radius). */
    std::size_t collect_leaf_samples_in_sphere(Vector3 const& center, Scalar radius,
        std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        Scalar const radius_sq = radius * radius;
        return collect_leaf_samples_impl(
            [&center, radius_sq](Node const& node) {
                return (node.sample.position - center).squaredNorm() <= radius_sq;
            },
            samples, distances);
    }

    /** Collect leaf representatives outside the sphere (distance > radius). */
    std::size_t collect_leaf_samples_outside_sphere(Vector3 const& center, Scalar radius,
        std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        Scalar const radius_sq = radius * radius;
        return collect_leaf_samples_impl(
            [&center, radius_sq](Node const& node) {
                return (node.sample.position - center).squaredNorm() > radius_sq;
            },
            samples, distances);
    }

    /**
     * Display-oriented query: fresh leaves, plus any leaf inside the sphere
     * (even if its timestamp is older than max_stale_age).
     */
    std::size_t collect_leaf_samples_near_or_fresh(Vector3 const& center, Scalar radius,
        double reference_time,
        std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        double const cutoff = reference_time - max_stale_age_;
        Scalar const radius_sq = radius * radius;
        return collect_leaf_samples_impl(
            [&center, radius_sq, cutoff](Node const& node) {
                if (node.sample.timestamp >= cutoff)
                    return true;
                return (node.sample.position - center).squaredNorm() <= radius_sq;
            },
            samples, distances);
    }

    /** Backward-compatible alias of collect_all_leaf_samples. */
    void collect_leaf_samples(std::vector<SampleType>& samples,
        std::vector<Scalar>* distances = nullptr) const
    {
        (void)collect_all_leaf_samples(samples, distances);
    }

    /** Collect every leaf position. */
    std::size_t collect_all_leaf_representatives(std::vector<Vector3>& points,
        std::vector<Scalar>* distances = nullptr) const
    {
        return collect_leaf_representatives_impl(
            [](Node const&) { return true; }, points, distances);
    }

    std::size_t collect_leaf_representatives_fresh(double reference_time,
        std::vector<Vector3>& points,
        std::vector<Scalar>* distances = nullptr) const
    {
        double const cutoff = reference_time - max_stale_age_;
        return collect_leaf_representatives_impl(
            [cutoff](Node const& node) { return node.sample.timestamp >= cutoff; },
            points, distances);
    }

    std::size_t collect_leaf_representatives_in_sphere(Vector3 const& center, Scalar radius,
        std::vector<Vector3>& points,
        std::vector<Scalar>* distances = nullptr) const
    {
        Scalar const radius_sq = radius * radius;
        return collect_leaf_representatives_impl(
            [&center, radius_sq](Node const& node) {
                return (node.sample.position - center).squaredNorm() <= radius_sq;
            },
            points, distances);
    }

    std::size_t collect_leaf_representatives_near_or_fresh(Vector3 const& center, Scalar radius,
        double reference_time,
        std::vector<Vector3>& points,
        std::vector<Scalar>* distances = nullptr) const
    {
        double const cutoff = reference_time - max_stale_age_;
        Scalar const radius_sq = radius * radius;
        return collect_leaf_representatives_impl(
            [&center, radius_sq, cutoff](Node const& node) {
                if (node.sample.timestamp >= cutoff)
                    return true;
                return (node.sample.position - center).squaredNorm() <= radius_sq;
            },
            points, distances);
    }

    /** Backward-compatible alias of collect_all_leaf_representatives. */
    void collect_leaf_representatives(std::vector<Vector3>& points,
        std::vector<Scalar>* distances = nullptr) const
    {
        (void)collect_all_leaf_representatives(points, distances);
    }

    /** Remove leaves with sample.timestamp < cutoff_time. */
    std::size_t remove_before_time(double cutoff_time)
    {
        return remove_leaf_samples_if(
            [cutoff_time](Node const& node) { return node.sample.timestamp < cutoff_time; });
    }

    /** Remove leaves outside the sphere (distance > radius). */
    std::size_t remove_outside_sphere(Vector3 const& center, Scalar radius)
    {
        Scalar const radius_sq = radius * radius;
        return remove_leaf_samples_if(
            [&center, radius_sq](Node const& node) {
                return (node.sample.position - center).squaredNorm() > radius_sq;
            });
    }

    /** Remove leaves inside the sphere with sample.timestamp < cutoff_time. */
    std::size_t remove_stale_inside_sphere(Vector3 const& center, Scalar radius,
        double cutoff_time)
    {
        Scalar const radius_sq = radius * radius;
        return remove_leaf_samples_if(
            [&center, radius_sq, cutoff_time](Node const& node) {
                if ((node.sample.position - center).squaredNorm() > radius_sq)
                    return false;
                return node.sample.timestamp < cutoff_time;
            });
    }

    /** Collapse empty branches and shrink the root around remaining content. */
    void reclaim_tree()
    {
        if (root_ == nullptr)
            return;

        (void)reclaim_node(root_);
        while (try_shrink_root_once())
        {
        }

        if (!subtree_has_point(root_))
            clear();
    }

private:
    template <typename Predicate>
    std::size_t remove_leaf_samples_if(Predicate should_remove)
    {
        if (root_ == nullptr)
            return 0;

        std::size_t removed = 0;
        Iterator iter = get_iterator_for_root();
        for (iter.first_leaf(); iter.current != nullptr; iter.next_leaf())
        {
            if (!iter.current->has_point)
                continue;
            if (!should_remove(*iter.current))
                continue;

            clear_leaf_point(iter.current);
            ++removed;
        }

        if (removed > 0)
            reclaim_tree();
        return removed;
    }

    template <typename Predicate>
    std::size_t collect_leaf_samples_impl(Predicate include_leaf,
        std::vector<SampleType>& samples,
        std::vector<Scalar>* distances) const
    {
        samples.clear();
        if (distances != nullptr)
            distances->clear();

        if (root_ == nullptr)
            return 0;

        std::size_t count = 0;
        Iterator iter = get_iterator_for_root();
        for (iter.first_leaf(); iter.current != nullptr; iter.next_leaf())
        {
            if (!iter.current->has_point)
                continue;
            if (!include_leaf(*iter.current))
                continue;

            samples.push_back(iter.current->sample);
            if (distances != nullptr)
                distances->push_back(std::sqrt(iter.current->dist_to_center_sq));
            ++count;
        }
        return count;
    }

    template <typename Predicate>
    std::size_t collect_leaf_representatives_impl(Predicate include_leaf,
        std::vector<Vector3>& points,
        std::vector<Scalar>* distances) const
    {
        points.clear();
        if (distances != nullptr)
            distances->clear();

        if (root_ == nullptr)
            return 0;

        std::size_t count = 0;
        Iterator iter = get_iterator_for_root();
        for (iter.first_leaf(); iter.current != nullptr; iter.next_leaf())
        {
            if (!iter.current->has_point)
                continue;
            if (!include_leaf(*iter.current))
                continue;

            points.push_back(iter.current->sample.position);
            if (distances != nullptr)
                distances->push_back(std::sqrt(iter.current->dist_to_center_sq));
            ++count;
        }
        return count;
    }

    void clear_counters_only()
    {
        num_nodes_ = 0;
        num_points_ = 0;
        num_active_points_ = 0;
    }

    void clear_leaf_point(Node* node)
    {
        if (node == nullptr || !node->has_point)
            return;

        node->has_point = false;
        node->dist_to_center_sq = std::numeric_limits<Scalar>::max();
        --num_active_points_;
    }

    bool subtree_has_point(Node const* node) const
    {
        if (node == nullptr)
            return false;
        if (node->children == nullptr)
            return node->has_point;

        for (int i = 0; i < 8; ++i)
        {
            if (subtree_has_point(node->children + i))
                return true;
        }
        return false;
    }

    bool reclaim_node(Node* node)
    {
        if (node->children == nullptr)
            return !node->has_point;

        bool all_empty = true;
        for (int i = 0; i < 8; ++i)
        {
            if (!reclaim_node(node->children + i))
                all_empty = false;
        }

        if (all_empty)
        {
            delete[] node->children;
            node->children = nullptr;
            num_nodes_ -= 8;
            return true;
        }
        return false;
    }

    bool try_shrink_root_once()
    {
        if (root_ == nullptr || root_->children == nullptr)
            return false;

        int only_octant = -1;
        int active_octants = 0;
        for (int i = 0; i < 8; ++i)
        {
            if (!subtree_has_point(root_->children + i))
                continue;
            only_octant = i;
            ++active_octants;
        }
        if (active_octants != 1)
            return false;

        Node* const old_children = root_->children;
        Node* const child = old_children + only_octant;
        Scalar const quarter = root_size_ / Scalar(4);
        for (int j = 0; j < 3; ++j)
            root_center_[j] += ((only_octant & (1 << j)) != 0) ? quarter : -quarter;
        root_size_ /= Scalar(2);

        // Steal fields without a temporary Node — Node::~Node deletes children[].
        Node* const promoted_children = child->children;
        SampleType const promoted_sample = child->sample;
        Scalar const promoted_dist = child->dist_to_center_sq;
        bool const promoted_has_point = child->has_point;
        child->children = nullptr;

        delete[] old_children;

        root_->children = promoted_children;
        root_->sample = promoted_sample;
        root_->dist_to_center_sq = promoted_dist;
        root_->has_point = promoted_has_point;
        num_nodes_ -= 8;

        if (root_->children != nullptr)
        {
            for (int i = 0; i < 8; ++i)
                root_->children[i].parent = root_;
        }
        return true;
    }

    void ensure_root(Vector3 const& point)
    {
        if (root_ != nullptr)
            return;

        if (min_leaf_size_ <= Scalar(0))
            throw std::logic_error("CenterNearestOctree: min_leaf_size must be positive before insert");

        root_ = new Node();
        root_center_ = point;
        root_size_ = min_leaf_size_;
        num_nodes_ = 1;
    }

    bool is_leaf_size(Scalar node_size) const
    {
        return node_size <= min_leaf_size_ * (Scalar(1) + Scalar(1e-9));
    }

    bool is_inside_root(Vector3 const& point) const
    {
        Scalar const half = root_size_ / Scalar(2);
        for (int i = 0; i < 3; ++i)
        {
            if (point[i] < root_center_[i] - half || point[i] > root_center_[i] + half)
                return false;
        }
        return true;
    }

    void expand_root_for_point(Vector3 const& point)
    {
        int octant = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (point[i] > root_center_[i])
                root_center_[i] += root_size_ / Scalar(2);
            else
            {
                octant |= (1 << i);
                root_center_[i] -= root_size_ / Scalar(2);
            }
        }
        root_size_ *= Scalar(2);

        Node* new_root = new Node();
        create_children(new_root);
        std::swap(new_root->children[octant].children, root_->children);
        std::swap(new_root->children[octant].sample, root_->sample);
        std::swap(new_root->children[octant].dist_to_center_sq, root_->dist_to_center_sq);
        std::swap(new_root->children[octant].has_point, root_->has_point);

        delete root_;
        root_ = new_root;

        if (root_->children[octant].children != nullptr)
        {
            Node* children = root_->children[octant].children;
            for (int i = 0; i < 8; ++i)
                children[i].parent = root_->children + octant;
        }
    }

    void create_children(Node* node)
    {
        if (node->children != nullptr)
            throw std::runtime_error("CenterNearestOctree: children already exist");

        node->children = new Node[8];
        num_nodes_ += 8;
        for (int i = 0; i < 8; ++i)
            node->children[i].parent = node;
    }

    static int compute_octant(Vector3 const& point, Vector3 const& center)
    {
        int octant = 0;
        for (int i = 0; i < 3; ++i)
            if (point[i] > center[i])
                octant |= (1 << i);
        return octant;
    }

    bool insert_sample_descend(SampleType const& sample, Node* node,
        Vector3 const& node_center, Scalar node_size, int level)
    {
        Vector3 const& point = sample.position;
        if (node->children != nullptr)
        {
            int const octant = compute_octant(point, node_center);
            Vector3 child_center = node_center;
            Scalar const offset = node_size / Scalar(4);
            for (int j = 0; j < 3; ++j)
                child_center[j] += ((octant & (1 << j)) != 0) ? offset : -offset;
            return insert_sample_descend(sample, node->children + octant, child_center,
                node_size / Scalar(2), level + 1);
        }

        if (!node->has_point)
            return update_node_sample(node, sample, node_center);

        if (is_leaf_size(node_size) || level >= max_level_)
            return update_node_sample(node, sample, node_center);

        SampleType const old_sample = node->sample;
        node->has_point = false;
        node->dist_to_center_sq = std::numeric_limits<Scalar>::max();
        create_children(node);
        (void)insert_sample_descend(old_sample, node, node_center, node_size, level);
        return insert_sample_descend(sample, node, node_center, node_size, level);
    }

    bool update_node_sample(Node* node, SampleType const& sample, Vector3 const& node_center)
    {
        if (node->has_point)
        {
            // Refresh last-activity time whenever this leaf is observed again.
            if (sample.timestamp > node->sample.timestamp)
                node->sample.timestamp = sample.timestamp;

            if (!use_center_nearest_)
                return false;

            Vector3 const& point = sample.position;
            Scalar const new_dist_sq = (point - node_center).squaredNorm();
            if (new_dist_sq >= node->dist_to_center_sq)
                return false;

            double const activity_ts = node->sample.timestamp;
            node->sample = sample;
            if (activity_ts > node->sample.timestamp)
                node->sample.timestamp = activity_ts;
            node->dist_to_center_sq = new_dist_sq;
            return true;
        }

        Vector3 const& point = sample.position;
        node->sample = sample;
        node->dist_to_center_sq = (point - node_center).squaredNorm();
        node->has_point = true;
        ++num_active_points_;
        return true;
    }

private:
    Node* root_ = nullptr;
    Vector3 root_center_ = Vector3::Zero();
    Scalar root_size_ = Scalar(0);
    Scalar min_leaf_size_ = Scalar(0.01);
    bool use_center_nearest_ = true;
    double max_stale_age_ = std::numeric_limits<double>::infinity();
    int max_level_ = kMaxAllowedLevel;
    std::size_t num_nodes_ = 0;
    std::size_t num_points_ = 0;
    std::size_t num_active_points_ = 0;
};

} // namespace octree
} // namespace Ddx
