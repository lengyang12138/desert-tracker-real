#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <nav_msgs/OccupancyGrid.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <vector>

static const float NAN_F = std::numeric_limits<float>::quiet_NaN();

// ============================================================
//  Helper: 2D grid
// ============================================================
struct Grid
{
    int rows, cols;
    float res, origin_x, origin_y;

    int idx(int r, int c) const { return r * cols + c; }
    bool inside(int r, int c) const { return r >= 0 && r < rows && c >= 0 && c < cols; }
    void pointToCell(float x, float y, int& r, int& c) const
    {
        c = static_cast<int>((x - origin_x) / res);
        r = static_cast<int>((y - origin_y) / res);
    }
};

// ============================================================
//  Step A: Block-based coarse ground estimation
//    分块取低百分位 → 双线性插值 → 平滑地面参考面
// ============================================================
static std::vector<float> estimateGroundSurface(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const Grid& g, double block_size, double percentile,
    int opening_radius)
{
    int bc_n = std::max(1, static_cast<int>(std::ceil((g.cols * g.res) / block_size)));
    int br_n = std::max(1, static_cast<int>(std::ceil((g.rows * g.res) / block_size)));

    std::vector<std::vector<float>> block_zvals(br_n * bc_n);

    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        int bc = static_cast<int>((p.x - g.origin_x) / block_size);
        int br = static_cast<int>((p.y - g.origin_y) / block_size);
        if (bc < 0 || bc >= bc_n || br < 0 || br >= br_n)
            continue;
        block_zvals[br * bc_n + bc].push_back(p.z);
    }

    std::vector<float> coarse(br_n * bc_n, NAN_F);
    for (int i = 0; i < br_n * bc_n; i++)
    {
        auto& zv = block_zvals[i];
        if (zv.empty()) continue;
        std::sort(zv.begin(), zv.end());
        int pidx = std::min(static_cast<int>(zv.size() * percentile),
                            static_cast<int>(zv.size()) - 1);
        coarse[i] = zv[pidx];
    }

    // Fill empty blocks
    for (int pass = 0; pass < 3; pass++)
    {
        std::vector<float> filled = coarse;
        for (int br = 0; br < br_n; br++)
            for (int bc = 0; bc < bc_n; bc++)
            {
                int bi = br * bc_n + bc;
                if (!std::isnan(coarse[bi])) continue;
                float sum = 0; int cnt = 0;
                for (int dr = -1; dr <= 1; dr++)
                    for (int dc = -1; dc <= 1; dc++)
                    {
                        int nr = br + dr, nc = bc + dc;
                        if (nr < 0 || nr >= br_n || nc < 0 || nc >= bc_n) continue;
                        float v = coarse[nr * bc_n + nc];
                        if (!std::isnan(v)) { sum += v; cnt++; }
                    }
                if (cnt > 0) filled[bi] = sum / cnt;
            }
        coarse = filled;
    }

    // Morphological opening suppresses roofs, vehicles and vegetation crowns
    // before they can become the local ground reference.
    opening_radius = std::max(0, opening_radius);
    if (opening_radius > 0)
    {
        std::vector<float> eroded(coarse.size(), NAN_F);
        for (int br = 0; br < br_n; ++br)
            for (int bc = 0; bc < bc_n; ++bc)
            {
                float minimum = std::numeric_limits<float>::infinity();
                for (int dr = -opening_radius; dr <= opening_radius; ++dr)
                    for (int dc = -opening_radius; dc <= opening_radius; ++dc)
                    {
                        int nr = br + dr, nc = bc + dc;
                        if (nr < 0 || nr >= br_n || nc < 0 || nc >= bc_n) continue;
                        float value = coarse[nr * bc_n + nc];
                        if (std::isfinite(value)) minimum = std::min(minimum, value);
                    }
                if (std::isfinite(minimum)) eroded[br * bc_n + bc] = minimum;
            }
        std::vector<float> opened(coarse.size(), NAN_F);
        for (int br = 0; br < br_n; ++br)
            for (int bc = 0; bc < bc_n; ++bc)
            {
                float maximum = -std::numeric_limits<float>::infinity();
                for (int dr = -opening_radius; dr <= opening_radius; ++dr)
                    for (int dc = -opening_radius; dc <= opening_radius; ++dc)
                    {
                        int nr = br + dr, nc = bc + dc;
                        if (nr < 0 || nr >= br_n || nc < 0 || nc >= bc_n) continue;
                        float value = eroded[nr * bc_n + nc];
                        if (std::isfinite(value)) maximum = std::max(maximum, value);
                    }
                if (std::isfinite(maximum)) opened[br * bc_n + bc] = maximum;
            }
        coarse.swap(opened);
    }

    // Bilinear interpolation to full resolution
    std::vector<float> ground(g.rows * g.cols, NAN_F);
    for (int r = 0; r < g.rows; r++)
        for (int c = 0; c < g.cols; c++)
        {
            float bx = (c + 0.5f) * g.res / block_size - 0.5f;
            float by = (r + 0.5f) * g.res / block_size - 0.5f;
            int bx0 = std::max(0, std::min(static_cast<int>(std::floor(bx)), bc_n - 1));
            int by0 = std::max(0, std::min(static_cast<int>(std::floor(by)), br_n - 1));
            int bx1 = std::min(bx0 + 1, bc_n - 1);
            int by1 = std::min(by0 + 1, br_n - 1);
            float fx = bx - std::floor(bx), fy = by - std::floor(by);

            float z00 = coarse[by0 * bc_n + bx0], z10 = coarse[by0 * bc_n + bx1];
            float z01 = coarse[by1 * bc_n + bx0], z11 = coarse[by1 * bc_n + bx1];

            if (std::isnan(z00) || std::isnan(z10) || std::isnan(z01) || std::isnan(z11))
            {
                for (float v : {z00, z10, z01, z11})
                    if (!std::isnan(v)) { ground[g.idx(r, c)] = v; break; }
            }
            else
            {
                ground[g.idx(r, c)] =
                    z00 * (1-fx) * (1-fy) + z10 * fx * (1-fy) +
                    z01 * (1-fx) * fy     + z11 * fx * fy;
            }
        }

    return ground;
}

// ============================================================
//  Step B: Build DEM from ground-only points
//    用粗地面参考面筛选地面点, 取均值 Z 构建精细 DEM
// ============================================================
static std::vector<float> buildGroundDEM(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<float>& ground_ref, const Grid& g,
    double ground_tol,
    double slope_tolerance_distance,
    double max_ground_tolerance,
    double cell_ground_percentile)
{
    int n = g.rows * g.cols;

    // The block-percentile reference can be vertically biased on a slope.
    // Estimate its local gradient and widen the admissible ground band by
    // the elevation change expected over a short horizontal distance.
    std::vector<float> adaptive_tolerance(n, static_cast<float>(ground_tol));
    double tolerance_sum = 0.0;
    int tolerance_count = 0;
    float tolerance_min = std::numeric_limits<float>::infinity();
    float tolerance_max = 0.0f;
    for (int r = 0; r < g.rows; r++)
        for (int c = 0; c < g.cols; c++)
        {
            int i = g.idx(r, c);
            float center = ground_ref[i];
            if (std::isnan(center)) continue;

            float dzdx = 0.0f;
            float dzdy = 0.0f;
            bool have_x = false;
            bool have_y = false;
            if (c > 0 && c + 1 < g.cols)
            {
                float left = ground_ref[g.idx(r, c - 1)];
                float right = ground_ref[g.idx(r, c + 1)];
                if (!std::isnan(left) && !std::isnan(right))
                {
                    dzdx = (right - left) / (2.0f * g.res);
                    have_x = true;
                }
            }
            if (!have_x && c + 1 < g.cols)
            {
                float right = ground_ref[g.idx(r, c + 1)];
                if (!std::isnan(right)) dzdx = (right - center) / g.res;
            }
            else if (!have_x && c > 0)
            {
                float left = ground_ref[g.idx(r, c - 1)];
                if (!std::isnan(left)) dzdx = (center - left) / g.res;
            }

            if (r > 0 && r + 1 < g.rows)
            {
                float down = ground_ref[g.idx(r - 1, c)];
                float up = ground_ref[g.idx(r + 1, c)];
                if (!std::isnan(down) && !std::isnan(up))
                {
                    dzdy = (up - down) / (2.0f * g.res);
                    have_y = true;
                }
            }
            if (!have_y && r + 1 < g.rows)
            {
                float up = ground_ref[g.idx(r + 1, c)];
                if (!std::isnan(up)) dzdy = (up - center) / g.res;
            }
            else if (!have_y && r > 0)
            {
                float down = ground_ref[g.idx(r - 1, c)];
                if (!std::isnan(down)) dzdy = (center - down) / g.res;
            }

            float slope_tan = std::sqrt(dzdx * dzdx + dzdy * dzdy);
            float tol = static_cast<float>(
                ground_tol + slope_tolerance_distance * slope_tan);
            tol = std::min(static_cast<float>(max_ground_tolerance), tol);
            tol = std::max(static_cast<float>(ground_tol), tol);
            adaptive_tolerance[i] = tol;
            tolerance_min = std::min(tolerance_min, tol);
            tolerance_max = std::max(tolerance_max, tol);
            tolerance_sum += tol;
            tolerance_count++;
        }

    if (tolerance_count > 0)
        ROS_INFO("  Slope-adaptive ground tolerance: min=%.2fm mean=%.2fm "
                 "max=%.2fm", tolerance_min,
                 tolerance_sum / tolerance_count, tolerance_max);

    std::vector<std::vector<float>> ground_z(n);

    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        int r, c; g.pointToCell(p.x, p.y, r, c);
        if (!g.inside(r, c)) continue;
        int i = g.idx(r, c);
        float ref = ground_ref[i];
        if (std::isnan(ref)) continue;
        float h = p.z - ref;
        float tol = adaptive_tolerance[i];
        if (h >= -tol && h <= tol) ground_z[i].push_back(p.z);
    }

    std::vector<float> dem(n, NAN_F);
    for (int i = 0; i < n; i++)
    {
        auto& values = ground_z[i];
        if (values.empty()) continue;
        float quantile = std::max(
            0.0, std::min(1.0, cell_ground_percentile));
        size_t qidx = static_cast<size_t>(
            std::floor(quantile * static_cast<double>(values.size() - 1)));
        std::nth_element(values.begin(), values.begin() + qidx, values.end());
        dem[i] = values[qidx];
    }
    return dem;
}

// ============================================================
//  Step B2: 基于欧式聚类的障碍物检测
//    1) 提取非地面点 (高于地面 DEM + height_thresh)
//    2) 欧式聚类, 过滤小簇噪点
//    3) 有效聚类投影到栅格 → 障碍物
//    坡面上的点属于地面, 不会产生非地面点簇, 因此不会被误检
// ============================================================
static std::vector<float> detectObstaclesByClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<float>& ground_dem,
    const Grid& g,
    float height_thresh,
    float cluster_tolerance,
    int min_cluster_size,
    float span_cell_size,
    float min_vertical_span)
{
    int n = g.rows * g.cols;
    span_cell_size = std::max(span_cell_size, g.res);
    min_vertical_span = std::max(0.0f, min_vertical_span);

    // Continuous slopes are locally thin surfaces even if the smoothed DEM
    // underestimates their height. Upright objects retain a large Z span in
    // the same small XY support, so use local span as a confirmation gate.
    const int span_cols = std::max(
        1, static_cast<int>(std::ceil(g.cols * g.res / span_cell_size)));
    const int span_rows = std::max(
        1, static_cast<int>(std::ceil(g.rows * g.res / span_cell_size)));
    std::vector<float> local_z_min(
        span_rows * span_cols, std::numeric_limits<float>::infinity());
    std::vector<float> local_z_max(
        span_rows * span_cols, -std::numeric_limits<float>::infinity());

    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        int sc = static_cast<int>((p.x - g.origin_x) / span_cell_size);
        int sr = static_cast<int>((p.y - g.origin_y) / span_cell_size);
        if (sr < 0 || sr >= span_rows || sc < 0 || sc >= span_cols) continue;
        int si = sr * span_cols + sc;
        local_z_min[si] = std::min(local_z_min[si], p.z);
        local_z_max[si] = std::max(local_z_max[si], p.z);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        int r, c; g.pointToCell(p.x, p.y, r, c);
        if (!g.inside(r, c)) continue;
        float ref = ground_dem[g.idx(r, c)];
        if (std::isnan(ref)) continue;
        if (p.z - ref > height_thresh)
            non_ground->push_back({p.x, p.y, p.z});
    }

    ROS_INFO("  Non-ground points: %zu", non_ground->size());

    std::vector<float> obstacle(n, 0.0f);
    if (non_ground->empty()) return obstacle;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(non_ground);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(static_cast<int>(non_ground->size()));
    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground);
    ec.extract(clusters);

    ROS_INFO("  Found %zu obstacle clusters", clusters.size());

    int accepted_points = 0;
    int rejected_surface_points = 0;
    for (const auto& cluster : clusters)
    {
        for (int idx : cluster.indices)
        {
            const auto& p = non_ground->points[idx];
            int sc = static_cast<int>((p.x - g.origin_x) / span_cell_size);
            int sr = static_cast<int>((p.y - g.origin_y) / span_cell_size);
            if (sr < 0 || sr >= span_rows || sc < 0 || sc >= span_cols)
                continue;
            int si = sr * span_cols + sc;
            float vertical_span = local_z_max[si] - local_z_min[si];
            if (!std::isfinite(vertical_span)
                || vertical_span < min_vertical_span)
            {
                rejected_surface_points++;
                continue;
            }

            int r, c; g.pointToCell(p.x, p.y, r, c);
            if (g.inside(r, c))
            {
                obstacle[g.idx(r, c)] = 1.0f;
                accepted_points++;
            }
        }
    }

    ROS_INFO("  Vertical-span confirmation: accepted=%d rejected_surface=%d "
             "(cell=%.2fm, min_span=%.2fm)",
             accepted_points, rejected_surface_points,
             span_cell_size, min_vertical_span);

    return obstacle;
}

// ============================================================
//  Step C: IDW interpolation to fill holes
// ============================================================
static void fillHoles(std::vector<float>& dem, const Grid& g,
                      int iters, int radius, int min_neighbors,
                      float max_height_range)
{
    min_neighbors = std::max(1, min_neighbors);
    max_height_range = std::max(0.0f, max_height_range);
    for (int it = 0; it < iters; it++)
    {
        std::vector<float> filled = dem;
        int cnt = 0;
        for (int r = 0; r < g.rows; r++)
            for (int c = 0; c < g.cols; c++)
            {
                int i = g.idx(r, c);
                if (!std::isnan(dem[i])) continue;
                float sum = 0, wsum = 0;
                float local_min = std::numeric_limits<float>::infinity();
                float local_max = -std::numeric_limits<float>::infinity();
                int valid_neighbors = 0;
                for (int dr = -radius; dr <= radius; dr++)
                    for (int dc = -radius; dc <= radius; dc++)
                    {
                        if (dr == 0 && dc == 0) continue;
                        int nr = r + dr, nc = c + dc;
                        if (!g.inside(nr, nc)) continue;
                        float v = dem[g.idx(nr, nc)];
                        if (std::isnan(v)) continue;
                        float w = 1.0f / (std::sqrt(float(dr*dr + dc*dc)) + 0.01f);
                        sum += v * w; wsum += w;
                        local_min = std::min(local_min, v);
                        local_max = std::max(local_max, v);
                        valid_neighbors++;
                    }
                if (valid_neighbors >= min_neighbors && wsum > 0.0f &&
                    local_max - local_min <= max_height_range)
                {
                    filled[i] = sum / wsum;
                    cnt++;
                }
            }
        dem = filled;
        if (cnt == 0) break;
        ROS_INFO("  Fill pass %d: %d cells", it + 1, cnt);
    }
}

// ============================================================
//  Step D: Gaussian smooth
// ============================================================
static std::vector<float> gaussianSmooth(const std::vector<float>& dem, const Grid& g, int kr)
{
    int ks = 2 * kr + 1;
    float sigma = kr / 2.0f;
    std::vector<float> kernel(ks * ks);
    float ksum = 0;
    for (int dr = -kr; dr <= kr; dr++)
        for (int dc = -kr; dc <= kr; dc++)
        {
            float v = std::exp(-(dr*dr + dc*dc) / (2.0f * sigma * sigma));
            kernel[(dr+kr) * ks + (dc+kr)] = v;
            ksum += v;
        }
    for (auto& v : kernel) v /= ksum;

    std::vector<float> out(g.rows * g.cols, NAN_F);
    for (int r = 0; r < g.rows; r++)
        for (int c = 0; c < g.cols; c++)
        {
            if (std::isnan(dem[g.idx(r, c)])) continue;
            float sum = 0, wsum = 0;
            for (int dr = -kr; dr <= kr; dr++)
                for (int dc = -kr; dc <= kr; dc++)
                {
                    int nr = r + dr, nc = c + dc;
                    if (!g.inside(nr, nc)) continue;
                    float v = dem[g.idx(nr, nc)];
                    if (std::isnan(v)) continue;
                    float w = kernel[(dr+kr) * ks + (dc+kr)];
                    sum += v * w; wsum += w;
                }
            if (wsum > 0) out[g.idx(r, c)] = sum / wsum;
        }
    return out;
}

// ============================================================
//  Step E: Gradient components (Sobel, NaN-tolerant)
//    返回 dz/dx 和 dz/dy 分量，供航向相关代价计算使用
// ============================================================
static void computeGradient(
    const std::vector<float>& dem, const Grid& g,
    std::vector<float>& grad_x, std::vector<float>& grad_y)
{
    int n = g.rows * g.cols;
    grad_x.assign(n, NAN_F);
    grad_y.assign(n, NAN_F);
    float inv8r = 1.0f / (8.0f * g.res);

    for (int r = 1; r < g.rows - 1; r++)
        for (int c = 1; c < g.cols - 1; c++)
        {
            if (std::isnan(dem[g.idx(r, c)])) continue;

            float z[8] = {
                dem[g.idx(r+1,c-1)], dem[g.idx(r+1,c)], dem[g.idx(r+1,c+1)],
                dem[g.idx(r,  c-1)],                     dem[g.idx(r,  c+1)],
                dem[g.idx(r-1,c-1)], dem[g.idx(r-1,c)], dem[g.idx(r-1,c+1)]
            };

            int nan_cnt = 0;
            float valid_sum = 0;
            int valid_n = 0;
            for (int k = 0; k < 8; k++)
            {
                if (std::isnan(z[k])) nan_cnt++;
                else { valid_sum += z[k]; valid_n++; }
            }
            if (nan_cnt > 3 || valid_n == 0) continue;

            float fill = valid_sum / valid_n;
            for (int k = 0; k < 8; k++)
                if (std::isnan(z[k])) z[k] = fill;

            grad_x[g.idx(r, c)] = (-z[0] + z[2] - 2*z[3] + 2*z[4] - z[5] + z[7]) * inv8r;
            grad_y[g.idx(r, c)] = (-z[5] - 2*z[6] - z[7] + z[0] + 2*z[1] + z[2]) * inv8r;
        }
}

// ============================================================
//  Step F: 代价层 — 三因素通行代价量化 (创新点 1 核心)
//
//  基于库仑摩擦锥理论和车辆倾覆力学，将地形梯度量化为
//  三项归一化代价 [0,1]:
//    c_climb = tan(α) / μ          — 爬坡阻力 (摩擦锥约束)
//    c_roll  = tan(α) / tan(α_crit) — 侧翻趋势 (倾覆力矩比)
//    c_slip  = tan(α) / μ          — 滑移概率 (各方向最坏情况)
//
//  离线地图不知航向，存各航向最坏情况 + 梯度分量;
//  创新点 2 规划器在线按航向 θ 分解纵/横坡度计算方向相关代价
// ============================================================
struct TerrainParams
{
    float friction_coeff;
    float track_width;
    float cg_height;
    float weight_climb;
    float weight_rollover;
    float weight_slip;
};

static void computeTerrainCosts(
    const std::vector<float>& grad_x,
    const std::vector<float>& grad_y,
    const std::vector<float>& obstacle,
    const std::vector<float>& dem,
    const Grid& g,
    const TerrainParams& tp,
    std::vector<float>& cost_climb,
    std::vector<float>& cost_rollover,
    std::vector<float>& cost_slip,
    std::vector<float>& cost_total)
{
    int n = g.rows * g.cols;
    cost_climb.assign(n, NAN_F);
    cost_rollover.assign(n, NAN_F);
    cost_slip.assign(n, NAN_F);
    cost_total.assign(n, NAN_F);

    float tan_alpha_crit = tp.track_width / (2.0f * tp.cg_height);

    for (int i = 0; i < n; i++)
    {
        if (std::isnan(dem[i])) continue;

        if (obstacle[i] > 0.5f)
        {
            cost_climb[i] = cost_rollover[i] = cost_slip[i] = cost_total[i] = 1.0f;
            continue;
        }

        float dzdx = std::isnan(grad_x[i]) ? 0.0f : grad_x[i];
        float dzdy = std::isnan(grad_y[i]) ? 0.0f : grad_y[i];
        float tan_slope = std::sqrt(dzdx * dzdx + dzdy * dzdy);

        float cc = std::min(1.0f, tan_slope / tp.friction_coeff);
        float cr = std::min(1.0f, tan_slope / tan_alpha_crit);
        float cs = std::min(1.0f, tan_slope / tp.friction_coeff);

        cost_climb[i] = cc;
        cost_rollover[i] = cr;
        cost_slip[i] = cs;
        cost_total[i] = std::min(1.0f,
            tp.weight_climb * cc + tp.weight_rollover * cr + tp.weight_slip * cs);
    }
}

// ============================================================
//  Step G: 降采样 DEM → 粗分辨率
// ============================================================
static void downsampleDEM(
    const std::vector<float>& fine, const Grid& fg,
    std::vector<float>& coarse, Grid& cg, int scale)
{
    cg.res = fg.res * scale;
    cg.origin_x = fg.origin_x;
    cg.origin_y = fg.origin_y;
    cg.cols = (fg.cols + scale - 1) / scale;
    cg.rows = (fg.rows + scale - 1) / scale;

    int n = cg.rows * cg.cols;
    coarse.assign(n, NAN_F);

    for (int cr = 0; cr < cg.rows; cr++)
        for (int cc = 0; cc < cg.cols; cc++)
        {
            float sum = 0; int cnt = 0;
            for (int dr = 0; dr < scale; dr++)
                for (int dc = 0; dc < scale; dc++)
                {
                    int fr = cr * scale + dr;
                    int fc = cc * scale + dc;
                    if (!fg.inside(fr, fc)) continue;
                    float v = fine[fg.idx(fr, fc)];
                    if (!std::isnan(v)) { sum += v; cnt++; }
                }
            if (cnt > 0) coarse[cg.idx(cr, cc)] = sum / cnt;
        }
}

// ============================================================
//  Main
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_to_costmap");
    ros::NodeHandle nh("~");

    // --- Parameters ---
    std::string pcd_file, output_dir, map_name, frame_id;
    double resolution, voxel_size;
    double step_threshold, cluster_tolerance;
    int min_cluster_size;
    double obstacle_span_cell_size, obstacle_min_vertical_span;
    double ground_block_size, ground_percentile, ground_tolerance;
    double ground_slope_tolerance_distance, ground_max_tolerance;
    double ground_cell_percentile;
    int ground_opening_radius;
    int smooth_kernel, fill_iterations, fill_radius, fill_min_neighbors;
    double fill_max_height_range;
    int sor_k; double sor_std;
    double friction_coeff, vehicle_track_width, vehicle_cg_height;
    double weight_climb, weight_rollover, weight_slip;
    double coarse_resolution;

    nh.param<std::string>("pcd_file",   pcd_file, "");
    nh.param<std::string>("output_dir", output_dir, "");
    nh.param<std::string>("map_name",   map_name, "terrain_map");
    nh.param<std::string>("frame_id",   frame_id, "map");
    nh.param("resolution",          resolution, 0.2);
    nh.param("voxel_size",          voxel_size, 0.1);
    nh.param("sor_k",               sor_k, 20);
    nh.param("sor_std",             sor_std, 1.5);
    nh.param("ground_block_size",   ground_block_size, 2.0);
    nh.param("ground_percentile",   ground_percentile, 0.1);
    nh.param("ground_tolerance",    ground_tolerance, 0.3);
    nh.param("ground_slope_tolerance_distance",
             ground_slope_tolerance_distance, 1.0);
    nh.param("ground_max_tolerance", ground_max_tolerance, 0.9);
    nh.param("ground_cell_percentile", ground_cell_percentile, 0.25);
    nh.param("ground_opening_radius", ground_opening_radius, 1);
    nh.param("step_threshold",      step_threshold, 0.3);
    nh.param("cluster_tolerance",   cluster_tolerance, 0.5);
    nh.param("min_cluster_size",    min_cluster_size, 10);
    nh.param("obstacle_span_cell_size", obstacle_span_cell_size, 0.5);
    nh.param("obstacle_min_vertical_span", obstacle_min_vertical_span, 0.55);
    nh.param("fill_iterations",     fill_iterations, 8);
    nh.param("fill_radius",         fill_radius, 8);
    nh.param("fill_min_neighbors",  fill_min_neighbors, 6);
    nh.param("fill_max_height_range", fill_max_height_range, 0.8);
    nh.param("smooth_kernel",       smooth_kernel, 2);

    nh.param("friction_coeff",      friction_coeff, 0.4);
    nh.param("vehicle_track_width", vehicle_track_width, 1.277);
    nh.param("vehicle_cg_height",   vehicle_cg_height, 0.5);
    nh.param("weight_climb",        weight_climb, 0.4);
    nh.param("weight_rollover",     weight_rollover, 0.3);
    nh.param("weight_slip",         weight_slip, 0.3);
    nh.param("coarse_resolution",   coarse_resolution, 1.0);

    if (pcd_file.empty() || output_dir.empty())
    {
        ROS_FATAL("Both private parameters ~pcd_file and ~output_dir are required.");
        return 1;
    }
    if (resolution <= 0.0 || coarse_resolution <= 0.0 || voxel_size < 0.0)
    {
        ROS_FATAL("Map resolutions must be positive and voxel_size non-negative.");
        return 1;
    }

    TerrainParams tp;
    tp.friction_coeff = static_cast<float>(friction_coeff);
    tp.track_width    = static_cast<float>(vehicle_track_width);
    tp.cg_height      = static_cast<float>(vehicle_cg_height);
    tp.weight_climb   = static_cast<float>(weight_climb);
    tp.weight_rollover = static_cast<float>(weight_rollover);
    tp.weight_slip    = static_cast<float>(weight_slip);

    float tan_alpha_crit = tp.track_width / (2.0f * tp.cg_height);
    ROS_INFO("Terrain params: mu=%.2f, B=%.3fm, h_cg=%.2fm, alpha_crit=%.1f deg",
             tp.friction_coeff, tp.track_width, tp.cg_height,
             std::atan(tan_alpha_crit) * 180.0f / M_PI);

    // ===== 1. Load PCD =====
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1)
    {
        ROS_ERROR("Failed to load PCD: %s", pcd_file.c_str());
        return 1;
    }
    if (cloud->empty())
    {
        ROS_ERROR("Input PCD contains no points: %s", pcd_file.c_str());
        return 1;
    }
    ROS_INFO("[1/9] Loaded %zu points", cloud->size());

    // ===== 2. Statistical outlier removal =====
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_k);
        sor.setStddevMulThresh(sor_std);
        sor.filter(*cleaned);
        ROS_INFO("[2/9] Outlier removal: %zu -> %zu", cloud->size(), cleaned->size());
        cloud = cleaned;
    }

    if (cloud->empty())
    {
        ROS_ERROR("No points remain after outlier removal.");
        return 1;
    }

    // ===== 3. Voxel downsample =====
    if (voxel_size > 0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*ds);
        ROS_INFO("[3/9] Downsample: %zu -> %zu", cloud->size(), ds->size());
        cloud = ds;
    }

    // ===== 4. Setup grid =====
    float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    float pad = static_cast<float>(resolution);
    min_x -= pad; min_y -= pad; max_x += pad; max_y += pad;

    Grid g;
    g.cols = static_cast<int>(std::ceil((max_x - min_x) / resolution));
    g.rows = static_cast<int>(std::ceil((max_y - min_y) / resolution));
    g.res = static_cast<float>(resolution);
    g.origin_x = min_x;
    g.origin_y = min_y;
    ROS_INFO("[4/9] Grid: %d x %d  (%.1f x %.1f m)", g.cols, g.rows,
             max_x - min_x, max_y - min_y);

    // ===== 5. Ground estimation =====
    ROS_INFO("[5/9] Ground estimation (block=%.1fm)...", ground_block_size);
    auto ground_ref = estimateGroundSurface(
        cloud, g, ground_block_size, ground_percentile,
        ground_opening_radius);

    // ===== 6. Build DEM from ground points + fill + smooth =====
    ROS_INFO("[6/9] Building DEM...");
    auto dem = buildGroundDEM(
        cloud, ground_ref, g, ground_tolerance,
        ground_slope_tolerance_distance,
        ground_max_tolerance,
        ground_cell_percentile);
    fillHoles(dem, g, fill_iterations, fill_radius, fill_min_neighbors,
              static_cast<float>(fill_max_height_range));
    auto smooth_dem = gaussianSmooth(dem, g, smooth_kernel);

    // ===== 7. Obstacle detection (step discontinuity) =====
    ROS_INFO("[7/9] Detecting obstacles (clustering)...");
    auto obstacle = detectObstaclesByClustering(cloud, dem, g,
        static_cast<float>(step_threshold),
        static_cast<float>(cluster_tolerance),
        min_cluster_size,
        static_cast<float>(obstacle_span_cell_size),
        static_cast<float>(obstacle_min_vertical_span));

    // Persist auditable classifications so a map is never accepted only from
    // a plausible-looking occupancy image.
    pcl::PointCloud<pcl::PointXYZI> debug_ground;
    pcl::PointCloud<pcl::PointXYZI> debug_non_ground;
    pcl::PointCloud<pcl::PointXYZI> debug_obstacle;
    for (const auto& point : cloud->points)
    {
        int r, c;
        g.pointToCell(point.x, point.y, r, c);
        if (!g.inside(r, c)) continue;
        const int index = g.idx(r, c);
        const float reference = dem[index];
        if (!std::isfinite(reference)) continue;
        if (std::fabs(point.z - reference) <= ground_max_tolerance)
            debug_ground.push_back(point);
        else
            debug_non_ground.push_back(point);
        if (obstacle[index] > 0.5f)
            debug_obstacle.push_back(point);
    }
    pcl::io::savePCDFileBinary(
        output_dir + "/debug_ground_points.pcd", debug_ground);
    pcl::io::savePCDFileBinary(
        output_dir + "/debug_non_ground_points.pcd", debug_non_ground);
    pcl::io::savePCDFileBinary(
        output_dir + "/debug_obstacle_points.pcd", debug_obstacle);
    ROS_INFO("Saved debug PCDs: ground=%zu non_ground=%zu obstacle=%zu",
             debug_ground.size(), debug_non_ground.size(),
             debug_obstacle.size());

    // ===== 8. Gradient (fine) =====
    ROS_INFO("[8/11] Computing gradient (fine, %.2fm)...", g.res);
    std::vector<float> grad_x, grad_y;
    computeGradient(smooth_dem, g, grad_x, grad_y);

    // ===== 9. Terrain costs (fine) =====
    ROS_INFO("[9/11] Computing terrain costs (fine)...");
    std::vector<float> cost_climb, cost_rollover, cost_slip, cost_total;
    computeTerrainCosts(grad_x, grad_y, obstacle, smooth_dem, g, tp,
                        cost_climb, cost_rollover, cost_slip, cost_total);

    // ===== 10. Coarse map =====
    int coarse_scale = std::max(1, static_cast<int>(std::round(coarse_resolution / resolution)));
    Grid cg;
    std::vector<float> coarse_dem;
    downsampleDEM(smooth_dem, g, coarse_dem, cg, coarse_scale);
    ROS_INFO("[10/11] Coarse map: %d x %d  (%.2fm/cell)", cg.cols, cg.rows, cg.res);

    std::vector<float> cg_grad_x, cg_grad_y;
    computeGradient(coarse_dem, cg, cg_grad_x, cg_grad_y);

    std::vector<float> coarse_obstacle(cg.rows * cg.cols, 0.0f);
    for (int cr = 0; cr < cg.rows; cr++)
        for (int cc = 0; cc < cg.cols; cc++)
        {
            for (int dr = 0; dr < coarse_scale; dr++)
                for (int dc = 0; dc < coarse_scale; dc++)
                {
                    int fr = cr * coarse_scale + dr;
                    int fc = cc * coarse_scale + dc;
                    if (g.inside(fr, fc) && obstacle[g.idx(fr, fc)] > 0.5f)
                        coarse_obstacle[cg.idx(cr, cc)] = 1.0f;
                }
        }

    std::vector<float> cg_climb, cg_rollover, cg_slip, cg_total;
    computeTerrainCosts(cg_grad_x, cg_grad_y, coarse_obstacle, coarse_dem, cg, tp,
                        cg_climb, cg_rollover, cg_slip, cg_total);

    // ===== 11. Build grid_map (fine) =====
    ROS_INFO("[11/11] Building grid_map...");
    float length_x = g.cols * g.res;
    float length_y = g.rows * g.res;
    float center_x = min_x + length_x / 2.0f;
    float center_y = min_y + length_y / 2.0f;

    grid_map::GridMap map({
        "elevation", "grad_x", "grad_y", "obstacle",
        "cost_climb", "cost_rollover", "cost_slip", "cost_total"});
    map.setFrameId(frame_id);
    map.setGeometry(
        grid_map::Length(length_x, length_y),
        resolution,
        grid_map::Position(center_x, center_y));

    ROS_INFO("GridMap size: %d x %d", map.getSize()(0), map.getSize()(1));

    for (const auto& layer : map.getLayers())
        map[layer].setConstant(NAN_F);

    for (int r = 0; r < g.rows; r++)
    {
        for (int c = 0; c < g.cols; c++)
        {
            float wx = min_x + (c + 0.5f) * g.res;
            float wy = min_y + (r + 0.5f) * g.res;

            grid_map::Position pos(wx, wy);
            grid_map::Index index;
            if (!map.getIndex(pos, index))
                continue;

            int i = g.idx(r, c);
            if (std::isnan(smooth_dem[i])) continue;

            map.at("elevation",      index) = smooth_dem[i];
            map.at("grad_x",        index) = std::isnan(grad_x[i]) ? 0.0f : grad_x[i];
            map.at("grad_y",        index) = std::isnan(grad_y[i]) ? 0.0f : grad_y[i];
            map.at("obstacle",       index) = obstacle[i];
            map.at("cost_climb",     index) = std::isnan(cost_climb[i]) ? 0.0f : cost_climb[i];
            map.at("cost_rollover",  index) = std::isnan(cost_rollover[i]) ? 0.0f : cost_rollover[i];
            map.at("cost_slip",      index) = std::isnan(cost_slip[i]) ? 0.0f : cost_slip[i];
            map.at("cost_total",     index) = std::isnan(cost_total[i]) ? 0.0f : cost_total[i];
        }
    }

    // Save fine grid_map to bag
    std::string bag_path = output_dir + "/" + map_name + ".bag";
    grid_map::GridMapRosConverter::saveToBag(map, bag_path, "/grid_map");
    ROS_INFO("Saved fine GridMap: %s", bag_path.c_str());

    // Save the binary collision map used by move_base.
    //
    // Terrain risk must not be encoded as occupancy here.  map_server turns
    // sufficiently dark risk pixels into LETHAL_OBSTACLE cells; on a large,
    // continuous slope those cells can form an artificial wall and make an
    // otherwise traversable goal unreachable.  Fine/coarse terrain risk is
    // already preserved in terrain_cost*.bin and is consumed as a soft cost
    // by Hybrid A* and MPC.  This PGM therefore carries only hard obstacles.
    {
        std::string pgm_path = output_dir + "/traversability_map.pgm";
        std::string yaml_path = output_dir + "/traversability_map.yaml";

        std::vector<uint8_t> pgm_data(g.rows * g.cols, 205);
        for (int r = 0; r < g.rows; r++)
            for (int c = 0; c < g.cols; c++)
            {
                int i = g.idx(r, c);
                if (std::isnan(cost_total[i])) continue;
                pgm_data[i] = obstacle[i] > 0.5f ? 0 : 254;
            }

        std::vector<uint8_t> flipped(g.rows * g.cols);
        for (int r = 0; r < g.rows; r++)
            std::memcpy(&flipped[r * g.cols],
                        &pgm_data[(g.rows - 1 - r) * g.cols],
                        g.cols);

        std::ofstream pgm(pgm_path, std::ios::binary);
        pgm << "P5\n" << g.cols << " " << g.rows << "\n255\n";
        pgm.write(reinterpret_cast<const char*>(flipped.data()), flipped.size());
        pgm.close();

        std::ofstream yml(yaml_path);
        yml << "image: traversability_map.pgm\n"
            << "resolution: " << resolution << "\n"
            << "origin: [" << min_x << ", " << min_y << ", 0.0]\n"
            << "negate: 0\n"
            << "occupied_thresh: 0.65\n"
            << "free_thresh: 0.196\n";
        yml.close();

        ROS_INFO("Saved 2D costmap (fine): %s", yaml_path.c_str());
    }

    // Save the hard-obstacle classification independently from terrain risk.
    // This prevents steep but continuous terrain from being painted as an
    // obstacle in diagnostic figures merely because its total cost is high.
    {
        std::string path = output_dir + "/obstacle_map.pgm";
        std::vector<uint8_t> obstacle_data(g.rows * g.cols, 205);
        for (int r = 0; r < g.rows; r++)
            for (int c = 0; c < g.cols; c++)
            {
                int i = g.idx(r, c);
                if (std::isnan(smooth_dem[i])) continue;
                obstacle_data[i] = obstacle[i] > 0.5f ? 0 : 254;
            }

        std::vector<uint8_t> flipped(g.rows * g.cols);
        for (int r = 0; r < g.rows; r++)
            std::memcpy(&flipped[r * g.cols],
                        &obstacle_data[(g.rows - 1 - r) * g.cols],
                        g.cols);

        std::ofstream pgm(path, std::ios::binary);
        pgm << "P5\n" << g.cols << " " << g.rows << "\n255\n";
        pgm.write(reinterpret_cast<const char*>(flipped.data()), flipped.size());
        pgm.close();
        ROS_INFO("Saved hard-obstacle mask: %s", path.c_str());
    }

    // Save terrain cost binary (fine) — header + grad_x + grad_y + cost layers
    {
        std::string path = output_dir + "/terrain_cost.bin";
        std::ofstream out(path, std::ios::binary);
        const char magic[] = "TCM1";
        out.write(magic, 4);
        out.write(reinterpret_cast<const char*>(&g.rows), 4);
        out.write(reinterpret_cast<const char*>(&g.cols), 4);
        out.write(reinterpret_cast<const char*>(&g.res), 4);
        out.write(reinterpret_cast<const char*>(&g.origin_x), 4);
        out.write(reinterpret_cast<const char*>(&g.origin_y), 4);
        out.write(reinterpret_cast<const char*>(&tp.friction_coeff), 4);
        out.write(reinterpret_cast<const char*>(&tp.track_width), 4);
        out.write(reinterpret_cast<const char*>(&tp.cg_height), 4);
        int n = g.rows * g.cols;
        out.write(reinterpret_cast<const char*>(smooth_dem.data()), n * sizeof(float));
        out.write(reinterpret_cast<const char*>(grad_x.data()),     n * sizeof(float));
        out.write(reinterpret_cast<const char*>(grad_y.data()),     n * sizeof(float));
        out.write(reinterpret_cast<const char*>(cost_total.data()), n * sizeof(float));
        out.close();
        ROS_INFO("Saved terrain cost (fine): %s (%dx%d, %.2fm/cell)",
                 path.c_str(), g.cols, g.rows, g.res);
    }

    // Save the elevation layer separately for nav_evaluator. Keeping it in
    // output_dir prevents aligned maps from being evaluated with a stale DEM.
    {
        std::string path = output_dir + "/elevation_map.dem";
        std::ofstream out(path, std::ios::binary);
        const char magic[] = "DEM1";
        out.write(magic, 4);
        out.write(reinterpret_cast<const char*>(&g.rows), 4);
        out.write(reinterpret_cast<const char*>(&g.cols), 4);
        out.write(reinterpret_cast<const char*>(&g.res), 4);
        out.write(reinterpret_cast<const char*>(&g.origin_x), 4);
        out.write(reinterpret_cast<const char*>(&g.origin_y), 4);
        int n = g.rows * g.cols;
        out.write(reinterpret_cast<const char*>(smooth_dem.data()), n * sizeof(float));
        out.close();
        ROS_INFO("Saved elevation DEM: %s", path.c_str());
    }

    // Save terrain cost binary (coarse)
    {
        std::string path = output_dir + "/terrain_cost_coarse.bin";
        std::ofstream out(path, std::ios::binary);
        const char magic[] = "TCM1";
        out.write(magic, 4);
        out.write(reinterpret_cast<const char*>(&cg.rows), 4);
        out.write(reinterpret_cast<const char*>(&cg.cols), 4);
        out.write(reinterpret_cast<const char*>(&cg.res), 4);
        out.write(reinterpret_cast<const char*>(&cg.origin_x), 4);
        out.write(reinterpret_cast<const char*>(&cg.origin_y), 4);
        out.write(reinterpret_cast<const char*>(&tp.friction_coeff), 4);
        out.write(reinterpret_cast<const char*>(&tp.track_width), 4);
        out.write(reinterpret_cast<const char*>(&tp.cg_height), 4);
        int cn = cg.rows * cg.cols;
        out.write(reinterpret_cast<const char*>(coarse_dem.data()), cn * sizeof(float));
        out.write(reinterpret_cast<const char*>(cg_grad_x.data()), cn * sizeof(float));
        out.write(reinterpret_cast<const char*>(cg_grad_y.data()), cn * sizeof(float));
        out.write(reinterpret_cast<const char*>(cg_total.data()),  cn * sizeof(float));
        out.close();
        ROS_INFO("Saved terrain cost (coarse): %s (%dx%d, %.2fm/cell)",
                 path.c_str(), cg.cols, cg.rows, cg.res);
    }

    // ===== Publish =====
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);

    ROS_INFO("Publishing /grid_map (8 layers). Ctrl+C to exit.");
    ros::Rate rate(2.0);
    while (ros::ok())
    {
        msg.info.header.stamp = ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
