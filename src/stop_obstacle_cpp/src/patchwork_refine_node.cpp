#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class PatchworkRefineNode
{
public:
  PatchworkRefineNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      last_ground_process_wall_(0.0),
      last_nonground_process_wall_(0.0),
      skipped_ground_frames_(0),
      skipped_nonground_frames_(0)
  {
    loadParams();
    validateParams();
    buildSearchPlan();

    ground_sub_ = nh_.subscribe(
      ground_topic_,
      1,
      &PatchworkRefineNode::groundCallback,
      this,
      ros::TransportHints().tcpNoDelay());

    nonground_sub_ = nh_.subscribe(
      nonground_topic_,
      1,
      &PatchworkRefineNode::nongroundCallback,
      this,
      ros::TransportHints().tcpNoDelay());

    refined_nonground_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(refined_nonground_topic_, 1);

    if (publish_returned_ground_)
    {
      returned_ground_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(returned_ground_topic_, 1);
    }

    if (publish_corrected_ground_)
    {
      corrected_ground_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(corrected_ground_topic_, 1);
    }

    ROS_INFO("patchwork_refine_node started [CPU optimized, same refine logic]");
    ROS_INFO("Subscribe ground          : %s", ground_topic_.c_str());
    ROS_INFO("Subscribe nonground       : %s", nonground_topic_.c_str());
    ROS_INFO("Publish refined no_ground : %s", refined_nonground_topic_.c_str());
    ROS_INFO("Publish returned ground   : %s [%s]",
             returned_ground_topic_.c_str(),
             publish_returned_ground_ ? "ON" : "OFF");
    ROS_INFO("Publish corrected ground  : %s [%s]",
             corrected_ground_topic_.c_str(),
             publish_corrected_ground_ ? "ON" : "OFF");
    ROS_INFO("ground_grid_size=%.3f, component_grid_size=%.3f",
             ground_grid_size_, component_grid_size_);
    ROS_INFO("search_radius=[%.2f, %.2f], step=%.2f, levels=%zu",
             search_radius_min_, search_radius_max_, search_radius_step_,
             search_radii_.size());
    ROS_INFO("ground_update_hz=%.2f, refine_hz=%.2f",
             ground_update_hz_, refine_hz_);
    ROS_INFO("ROI max_abs_x=%.2f, max_abs_y=%.2f (<=0 means disabled)",
             max_abs_x_, max_abs_y_);
  }

private:
  // ============================================================
  // ROS
  // ============================================================

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber ground_sub_;
  ros::Subscriber nonground_sub_;

  ros::Publisher refined_nonground_pub_;
  ros::Publisher returned_ground_pub_;
  ros::Publisher corrected_ground_pub_;

  // ============================================================
  // Parameters
  // ============================================================

  std::string ground_topic_;
  std::string nonground_topic_;
  std::string refined_nonground_topic_;
  std::string returned_ground_topic_;
  std::string corrected_ground_topic_;

  double ground_grid_size_;
  double component_grid_size_;

  double search_radius_min_;
  double search_radius_step_;
  double search_radius_max_;

  int min_ground_cells_;

  double max_local_slope_;
  double return_to_ground_dist_;
  double keep_nonground_dist_;

  double large_component_area_;
  double large_component_return_radius_;
  double max_stamp_diff_warn_;

  bool publish_returned_ground_;
  bool publish_corrected_ground_;
  bool use_median_z_;

  double max_abs_x_;
  double max_abs_y_;

  // Processing-frequency limits. <= 0 disables the limit.
  double ground_update_hz_;
  double refine_hz_;

  // ============================================================
  // Data types
  // ============================================================

  struct PointXYZ
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct GroundAccum
  {
    int ix = 0;
    int iy = 0;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    int count = 0;
    std::vector<double> zs;
  };

  struct GroundCell
  {
    int ix = 0;
    int iy = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int count = 0;
  };

  struct GroundData
  {
    std::unordered_map<uint64_t, GroundCell> grid;

    // Only populated when corrected-ground output is enabled.
    std::vector<PointXYZ> points;

    std_msgs::Header header;
    std::size_t accepted_point_count = 0;
  };

  struct EstimateResult
  {
    bool valid = false;

    // z = a*x + b*y + c
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    double slope = 0.0;
    double radius_used = 0.0;
    int cells_used = 0;
  };

  struct PlaneSums
  {
    int count = 0;
    double sxx = 0.0;
    double sxy = 0.0;
    double sx = 0.0;
    double syy = 0.0;
    double sy = 0.0;
    double sxz = 0.0;
    double syz = 0.0;
    double sz = 0.0;

    void add(const GroundCell& cell)
    {
      const double x = cell.x;
      const double y = cell.y;
      const double z = cell.z;

      ++count;
      sxx += x * x;
      sxy += x * y;
      sx += x;
      syy += y * y;
      sy += y;
      sxz += x * z;
      syz += y * z;
      sz += z;
    }
  };

  struct NonGroundCell
  {
    int ix = 0;
    int iy = 0;
    std::vector<int> point_ids;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    bool visited = false;

    PointXYZ meanPoint() const
    {
      PointXYZ result;
      if (point_ids.empty())
      {
        return result;
      }

      const double inv_n = 1.0 / static_cast<double>(point_ids.size());
      result.x = sx * inv_n;
      result.y = sy * inv_n;
      result.z = sz * inv_n;
      return result;
    }
  };

  using GroundDataPtr = std::shared_ptr<const GroundData>;
  using NonGroundCellMap = std::unordered_map<uint64_t, NonGroundCell>;

  // ============================================================
  // Shared state
  // ============================================================

  std::mutex ground_mutex_;
  GroundDataPtr latest_ground_data_;

  std::mutex ground_rate_mutex_;
  std::mutex nonground_rate_mutex_;
  double last_ground_process_wall_;
  double last_nonground_process_wall_;
  std::uint64_t skipped_ground_frames_;
  std::uint64_t skipped_nonground_frames_;

  // Search plan is built once instead of rebuilding nested radius loops
  // for every occupied non-ground cell.
  std::vector<double> search_radii_;
  std::vector<double> search_radius_sq_;
  std::vector<std::pair<int, int>> max_radius_offsets_;

  // ============================================================
  // Parameter setup
  // ============================================================

  void loadParams()
  {
    pnh_.param<std::string>(
      "ground_topic", ground_topic_,
      "/ground_segmentation_front/ground");

    pnh_.param<std::string>(
      "nonground_topic", nonground_topic_,
      "/ground_segmentation_front/points_no_ground_front");

    pnh_.param<std::string>(
      "refined_nonground_topic", refined_nonground_topic_,
      "/patchwork_refine/no_ground");

    pnh_.param<std::string>(
      "returned_ground_topic", returned_ground_topic_,
      "/patchwork_refine/returned_ground");

    pnh_.param<std::string>(
      "corrected_ground_topic", corrected_ground_topic_,
      "/patchwork_refine/corrected_ground");

    // CPU-oriented defaults requested by the user.
    // The algorithm structure and all topics remain unchanged.
    pnh_.param("ground_grid_size", ground_grid_size_, 0.20);
    pnh_.param("component_grid_size", component_grid_size_, 0.20);

    pnh_.param("search_radius_min", search_radius_min_, 0.20);
    pnh_.param("search_radius_step", search_radius_step_, 0.20);
    pnh_.param("search_radius_max", search_radius_max_, 0.60);

    pnh_.param("min_ground_cells", min_ground_cells_, 4);
    pnh_.param("max_local_slope", max_local_slope_, 0.70);
    pnh_.param("return_to_ground_dist", return_to_ground_dist_, 0.04);
    pnh_.param("keep_nonground_dist", keep_nonground_dist_, 0.08);
    pnh_.param("large_component_area", large_component_area_, 0.25);
    pnh_.param("large_component_return_radius", large_component_return_radius_, 0.25);
    pnh_.param("max_stamp_diff_warn", max_stamp_diff_warn_, 0.20);

    pnh_.param("publish_returned_ground", publish_returned_ground_, false);
    pnh_.param("publish_corrected_ground", publish_corrected_ground_, false);
    pnh_.param("use_median_z", use_median_z_, false);

    // ROI remains disabled by default because the correct physical limits
    // depend on the vehicle installation and aligned-lidar coordinate frame.
    pnh_.param("max_abs_x", max_abs_x_, 0.0);
    pnh_.param("max_abs_y", max_abs_y_, 0.0);

    // Requested processing limits. Set either value <= 0 to process every frame.
    pnh_.param("ground_update_hz", ground_update_hz_, 0.0);
    pnh_.param("refine_hz", refine_hz_, 10.0);
  }

  void validateParams()
  {
    if (ground_grid_size_ <= 0.0)
    {
      ROS_WARN("ground_grid_size <= 0; reset to 0.20");
      ground_grid_size_ = 0.20;
    }

    if (component_grid_size_ <= 0.0)
    {
      ROS_WARN("component_grid_size <= 0; reset to 0.20");
      component_grid_size_ = 0.20;
    }

    if (search_radius_min_ <= 0.0)
    {
      search_radius_min_ = 0.20;
    }

    if (search_radius_step_ <= 0.0)
    {
      search_radius_step_ = 0.20;
    }

    if (search_radius_max_ < search_radius_min_)
    {
      search_radius_max_ = search_radius_min_;
    }

    min_ground_cells_ = std::max(3, min_ground_cells_);

    if (return_to_ground_dist_ < 0.0)
    {
      return_to_ground_dist_ = 0.0;
    }

    if (keep_nonground_dist_ < return_to_ground_dist_)
    {
      ROS_WARN("keep_nonground_dist < return_to_ground_dist; clamp to return distance");
      keep_nonground_dist_ = return_to_ground_dist_;
    }
  }

  void buildSearchPlan()
  {
    search_radii_.clear();
    search_radius_sq_.clear();
    max_radius_offsets_.clear();

    for (double radius = search_radius_min_;
         radius <= search_radius_max_ + 1e-9;
         radius += search_radius_step_)
    {
      search_radii_.push_back(radius);
      search_radius_sq_.push_back(radius * radius);
    }

    if (search_radii_.empty())
    {
      search_radii_.push_back(search_radius_min_);
      search_radius_sq_.push_back(search_radius_min_ * search_radius_min_);
    }

    const int max_cells = static_cast<int>(
      std::ceil(search_radii_.back() / ground_grid_size_));

    max_radius_offsets_.reserve(
      static_cast<std::size_t>((2 * max_cells + 1) * (2 * max_cells + 1)));

    for (int dx = -max_cells; dx <= max_cells; ++dx)
    {
      for (int dy = -max_cells; dy <= max_cells; ++dy)
      {
        max_radius_offsets_.push_back(std::make_pair(dx, dy));
      }
    }
  }

  // ============================================================
  // General helpers
  // ============================================================

  static bool isFinite3(double x, double y, double z)
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  static uint64_t packKey(int ix, int iy)
  {
    const uint64_t ux = static_cast<uint32_t>(ix);
    const uint64_t uy = static_cast<uint32_t>(iy);
    return (ux << 32) | uy;
  }

  static void unpackKey(uint64_t key, int& ix, int& iy)
  {
    ix = static_cast<int32_t>((key >> 32) & 0xffffffffULL);
    iy = static_cast<int32_t>(key & 0xffffffffULL);
  }

  static int toGridIndex(double value, double grid_size)
  {
    return static_cast<int>(std::floor(value / grid_size));
  }

  bool passRoi(double x, double y) const
  {
    if (max_abs_x_ > 0.0 && std::fabs(x) > max_abs_x_)
    {
      return false;
    }

    if (max_abs_y_ > 0.0 && std::fabs(y) > max_abs_y_)
    {
      return false;
    }

    return true;
  }

  bool shouldProcess(double max_hz,
                     double& last_wall,
                     std::mutex& rate_mutex,
                     std::uint64_t& skipped_frames)
  {
    if (max_hz <= 0.0)
    {
      return true;
    }

    const double now = ros::WallTime::now().toSec();
    const double min_period = 1.0 / max_hz;

    std::lock_guard<std::mutex> lock(rate_mutex);

    if (last_wall > 0.0 && now - last_wall < min_period)
    {
      ++skipped_frames;
      return false;
    }

    last_wall = now;
    return true;
  }

  static double medianZ(std::vector<double>& values)
  {
    if (values.empty())
    {
      return 0.0;
    }

    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double median = values[middle];

    if (values.size() % 2 == 0 && middle > 0)
    {
      std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
      median = 0.5 * (median + values[middle - 1]);
    }

    return median;
  }

  // ============================================================
  // Ground-grid construction
  // ============================================================

  std::shared_ptr<GroundData> buildGroundData(
    const sensor_msgs::PointCloud2& msg)
  {
    std::shared_ptr<GroundData> data(new GroundData);
    data->header = msg.header;

    const std::size_t input_count =
      static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

    std::unordered_map<uint64_t, GroundAccum> accumulators;
    accumulators.reserve(input_count / 4 + 1);

    if (publish_corrected_ground_)
    {
      data->points.reserve(input_count);
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);

      if (!isFinite3(x, y, z) || !passRoi(x, y))
      {
        continue;
      }

      ++data->accepted_point_count;

      if (publish_corrected_ground_)
      {
        PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        data->points.push_back(point);
      }

      const int ix = toGridIndex(x, ground_grid_size_);
      const int iy = toGridIndex(y, ground_grid_size_);
      const uint64_t key = packKey(ix, iy);

      std::unordered_map<uint64_t, GroundAccum>::iterator it =
        accumulators.find(key);

      if (it == accumulators.end())
      {
        GroundAccum initial;
        initial.ix = ix;
        initial.iy = iy;
        it = accumulators.emplace(key, std::move(initial)).first;
      }

      GroundAccum& accum = it->second;
      accum.sx += x;
      accum.sy += y;
      accum.sz += z;
      ++accum.count;

      if (use_median_z_)
      {
        accum.zs.push_back(z);
      }
    }

    data->grid.reserve(accumulators.size());

    for (std::unordered_map<uint64_t, GroundAccum>::iterator it =
           accumulators.begin();
         it != accumulators.end();
         ++it)
    {
      GroundAccum& accum = it->second;
      const int count = std::max(1, accum.count);
      const double inv_count = 1.0 / static_cast<double>(count);

      GroundCell cell;
      cell.ix = accum.ix;
      cell.iy = accum.iy;
      cell.x = accum.sx * inv_count;
      cell.y = accum.sy * inv_count;
      cell.z = use_median_z_ ? medianZ(accum.zs) : accum.sz * inv_count;
      cell.count = count;

      data->grid.emplace(it->first, cell);
    }

    return data;
  }

  // ============================================================
  // Local-ground estimation
  // ============================================================

  bool fitLocalGroundPlane(const PlaneSums& sums,
                           double& a,
                           double& b,
                           double& c,
                           double& slope) const
  {
    if (sums.count < min_ground_cells_)
    {
      return false;
    }

    Eigen::Matrix3d ata;
    ata(0, 0) = sums.sxx;
    ata(0, 1) = sums.sxy;
    ata(0, 2) = sums.sx;
    ata(1, 0) = sums.sxy;
    ata(1, 1) = sums.syy;
    ata(1, 2) = sums.sy;
    ata(2, 0) = sums.sx;
    ata(2, 1) = sums.sy;
    ata(2, 2) = static_cast<double>(sums.count);

    Eigen::Vector3d atz(sums.sxz, sums.syz, sums.sz);
    Eigen::LDLT<Eigen::Matrix3d> solver(ata);

    if (solver.info() != Eigen::Success)
    {
      return false;
    }

    const Eigen::Vector3d coefficients = solver.solve(atz);

    if (solver.info() != Eigen::Success)
    {
      return false;
    }

    a = coefficients(0);
    b = coefficients(1);
    c = coefficients(2);

    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
    {
      return false;
    }

    slope = std::sqrt(a * a + b * b);

    return std::isfinite(slope) && slope <= max_local_slope_;
  }

  EstimateResult estimateGroundAt(
    const PointXYZ& query,
    const std::unordered_map<uint64_t, GroundCell>& ground_grid) const
  {
    EstimateResult result;

    const int query_ix = toGridIndex(query.x, ground_grid_size_);
    const int query_iy = toGridIndex(query.y, ground_grid_size_);

    // Each level receives exactly the cells whose actual XY distance is
    // inside that level's radius. Hash lookup is performed only once per
    // offset up to the maximum radius.
    std::vector<PlaneSums> sums_by_radius(search_radii_.size());

    for (std::size_t offset_index = 0;
         offset_index < max_radius_offsets_.size();
         ++offset_index)
    {
      const int ix = query_ix + max_radius_offsets_[offset_index].first;
      const int iy = query_iy + max_radius_offsets_[offset_index].second;
      const uint64_t key = packKey(ix, iy);

      std::unordered_map<uint64_t, GroundCell>::const_iterator it =
        ground_grid.find(key);

      if (it == ground_grid.end())
      {
        continue;
      }

      const GroundCell& cell = it->second;
      const double dx = cell.x - query.x;
      const double dy = cell.y - query.y;
      const double distance_squared = dx * dx + dy * dy;

      for (std::size_t radius_index = 0;
           radius_index < search_radius_sq_.size();
           ++radius_index)
      {
        if (distance_squared <= search_radius_sq_[radius_index])
        {
          sums_by_radius[radius_index].add(cell);
        }
      }
    }

    for (std::size_t radius_index = 0;
         radius_index < search_radii_.size();
         ++radius_index)
    {
      const PlaneSums& sums = sums_by_radius[radius_index];

      if (sums.count < min_ground_cells_)
      {
        continue;
      }

      double a = 0.0;
      double b = 0.0;
      double c = 0.0;
      double slope = 0.0;

      if (!fitLocalGroundPlane(sums, a, b, c, slope))
      {
        continue;
      }

      result.valid = true;
      result.a = a;
      result.b = b;
      result.c = c;
      result.slope = slope;
      result.radius_used = search_radii_[radius_index];
      result.cells_used = sums.count;
      return result;
    }

    return result;
  }

  // ============================================================
  // Non-ground preparation and connected components
  // ============================================================

  void readNongroundCloud(
    const sensor_msgs::PointCloud2& msg,
    std::vector<PointXYZ>& points,
    NonGroundCellMap& cells) const
  {
    const std::size_t input_count =
      static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

    points.clear();
    points.reserve(input_count);

    cells.clear();
    cells.reserve(input_count / 4 + 1);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);

      if (!isFinite3(x, y, z) || !passRoi(x, y))
      {
        continue;
      }

      PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = z;

      const int point_index = static_cast<int>(points.size());
      points.push_back(point);

      const int ix = toGridIndex(x, component_grid_size_);
      const int iy = toGridIndex(y, component_grid_size_);
      const uint64_t key = packKey(ix, iy);

      NonGroundCellMap::iterator it = cells.find(key);

      if (it == cells.end())
      {
        NonGroundCell initial;
        initial.ix = ix;
        initial.iy = iy;
        it = cells.emplace(key, std::move(initial)).first;
      }

      NonGroundCell& cell = it->second;
      cell.point_ids.push_back(point_index);
      cell.sx += x;
      cell.sy += y;
      cell.sz += z;
    }
  }

  void collectComponent(uint64_t start_key,
                        NonGroundCellMap& cells,
                        std::vector<uint64_t>& component_keys) const
  {
    component_keys.clear();

    NonGroundCellMap::iterator start = cells.find(start_key);
    if (start == cells.end() || start->second.visited)
    {
      return;
    }

    std::vector<uint64_t> queue;
    queue.reserve(32);
    queue.push_back(start_key);
    start->second.visited = true;

    std::size_t head = 0;

    while (head < queue.size())
    {
      const uint64_t current_key = queue[head++];
      component_keys.push_back(current_key);

      int ix = 0;
      int iy = 0;
      unpackKey(current_key, ix, iy);

      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          if (dx == 0 && dy == 0)
          {
            continue;
          }

          const uint64_t neighbor_key = packKey(ix + dx, iy + dy);
          NonGroundCellMap::iterator neighbor = cells.find(neighbor_key);

          if (neighbor == cells.end() || neighbor->second.visited)
          {
            continue;
          }

          neighbor->second.visited = true;
          queue.push_back(neighbor_key);
        }
      }
    }
  }

  // ============================================================
  // Output
  // ============================================================

  static void publishXYZCloud(const std::vector<PointXYZ>& points,
                              const std_msgs::Header& header,
                              ros::Publisher& publisher)
  {
    if (!publisher)
    {
      return;
    }

    sensor_msgs::PointCloud2 output;
    output.header = header;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (std::size_t index = 0;
         index < points.size();
         ++index, ++out_x, ++out_y, ++out_z)
    {
      *out_x = static_cast<float>(points[index].x);
      *out_y = static_cast<float>(points[index].y);
      *out_z = static_cast<float>(points[index].z);
    }

    publisher.publish(output);
  }

  // ============================================================
  // Callbacks
  // ============================================================

  void groundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (!shouldProcess(ground_update_hz_,
                       last_ground_process_wall_,
                       ground_rate_mutex_,
                       skipped_ground_frames_))
    {
      return;
    }

    const ros::WallTime start_time = ros::WallTime::now();
    std::shared_ptr<GroundData> data = buildGroundData(*msg);

    {
      std::lock_guard<std::mutex> lock(ground_mutex_);
      latest_ground_data_ = data;
    }

    const double cost_ms =
      (ros::WallTime::now() - start_time).toSec() * 1000.0;

    ROS_INFO_THROTTLE(
      1.0,
      "Ground updated: accepted_points=%zu, ground_cells=%zu, "
      "skipped_frames=%llu, cost=%.2f ms",
      data->accepted_point_count,
      data->grid.size(),
      static_cast<unsigned long long>(skipped_ground_frames_),
      cost_ms);
  }

  void nongroundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (!shouldProcess(refine_hz_,
                       last_nonground_process_wall_,
                       nonground_rate_mutex_,
                       skipped_nonground_frames_))
    {
      return;
    }

    const ros::WallTime start_time = ros::WallTime::now();

    GroundDataPtr ground_data;
    {
      std::lock_guard<std::mutex> lock(ground_mutex_);
      ground_data = latest_ground_data_;
    }

    std::vector<PointXYZ> nonground_points;
    NonGroundCellMap cells;
    readNongroundCloud(*msg, nonground_points, cells);

    if (!ground_data || ground_data->grid.empty())
    {
      ROS_WARN_THROTTLE(1.0,
                        "No ground cloud received yet. Publish nonground directly.");
      publishXYZCloud(nonground_points, msg->header, refined_nonground_pub_);
      return;
    }

    if (msg->header.stamp.toSec() > 0.0 &&
        ground_data->header.stamp.toSec() > 0.0)
    {
      const double stamp_difference =
        std::fabs((msg->header.stamp - ground_data->header.stamp).toSec());

      if (stamp_difference > max_stamp_diff_warn_)
      {
        ROS_WARN_THROTTLE(
          1.0,
          "Ground and nonground stamp diff is %.3f s. "
          "Still processing with latest ground.",
          stamp_difference);
      }
    }

    std::vector<PointXYZ> refined_nonground;
    refined_nonground.reserve(nonground_points.size());

    const bool need_returned_points =
      publish_returned_ground_ || publish_corrected_ground_;

    std::vector<PointXYZ> returned_ground;
    if (need_returned_points)
    {
      returned_ground.reserve(nonground_points.size() / 5 + 1);
    }

    int total_components = 0;
    int large_components = 0;
    int no_ground_reference_count = 0;
    int returned_count = 0;
    int kept_count = 0;
    int estimated_cells = 0;

    std::vector<uint64_t> component_keys;
    component_keys.reserve(64);

    for (NonGroundCellMap::iterator cell_it = cells.begin();
         cell_it != cells.end();
         ++cell_it)
    {
      if (cell_it->second.visited)
      {
        continue;
      }

      collectComponent(cell_it->first, cells, component_keys);

      if (component_keys.empty())
      {
        continue;
      }

      ++total_components;

      const double component_area =
        static_cast<double>(component_keys.size()) *
        component_grid_size_ * component_grid_size_;

      const bool is_large_component =
        component_area >= large_component_area_;

      if (is_large_component)
      {
        ++large_components;
      }

      for (std::size_t key_index = 0;
           key_index < component_keys.size();
           ++key_index)
      {
        NonGroundCellMap::iterator current =
          cells.find(component_keys[key_index]);

        if (current == cells.end())
        {
          continue;
        }

        const NonGroundCell& cell = current->second;
        const std::vector<int>& point_ids = cell.point_ids;
        const PointXYZ query = cell.meanPoint();
        const EstimateResult estimate =
          estimateGroundAt(query, ground_data->grid);

        ++estimated_cells;

        if (!estimate.valid)
        {
          for (std::size_t id_index = 0;
               id_index < point_ids.size();
               ++id_index)
          {
            refined_nonground.push_back(nonground_points[point_ids[id_index]]);
            ++no_ground_reference_count;
            ++kept_count;
          }
          continue;
        }

        for (std::size_t id_index = 0;
             id_index < point_ids.size();
             ++id_index)
        {
          const PointXYZ& point = nonground_points[point_ids[id_index]];
          const double ground_z =
            estimate.a * point.x + estimate.b * point.y + estimate.c;

          if (!std::isfinite(ground_z))
          {
            refined_nonground.push_back(point);
            ++no_ground_reference_count;
            ++kept_count;
            continue;
          }

          const double absolute_height_difference =
            std::fabs(point.z - ground_z);

          // Same original decision logic.
          if (absolute_height_difference >= keep_nonground_dist_)
          {
            refined_nonground.push_back(point);
            ++kept_count;
            continue;
          }

          const bool return_candidate =
            absolute_height_difference <= return_to_ground_dist_;

          if (!return_candidate)
          {
            refined_nonground.push_back(point);
            ++kept_count;
            continue;
          }

          if (is_large_component &&
              estimate.radius_used > large_component_return_radius_)
          {
            refined_nonground.push_back(point);
            ++kept_count;
            continue;
          }

          if (need_returned_points)
          {
            returned_ground.push_back(point);
          }

          ++returned_count;
        }
      }
    }

    publishXYZCloud(refined_nonground, msg->header, refined_nonground_pub_);

    if (publish_returned_ground_)
    {
      publishXYZCloud(returned_ground, msg->header, returned_ground_pub_);
    }

    if (publish_corrected_ground_)
    {
      std::vector<PointXYZ> corrected_ground;
      corrected_ground.reserve(
        ground_data->points.size() + returned_ground.size());
      corrected_ground.insert(
        corrected_ground.end(),
        ground_data->points.begin(),
        ground_data->points.end());
      corrected_ground.insert(
        corrected_ground.end(),
        returned_ground.begin(),
        returned_ground.end());

      publishXYZCloud(corrected_ground, msg->header, corrected_ground_pub_);
    }

    const double cost_ms =
      (ros::WallTime::now() - start_time).toSec() * 1000.0;

    ROS_INFO_THROTTLE(
      0.5,
      "Patchwork refine: input_no_ground=%zu, refined_no_ground=%zu, "
      "returned=%d, cells=%zu, estimated_cells=%d, components=%d, "
      "large_components=%d, no_ground_ref=%d, kept=%d, "
      "skipped_frames=%llu, cost=%.2f ms",
      nonground_points.size(),
      refined_nonground.size(),
      returned_count,
      cells.size(),
      estimated_cells,
      total_components,
      large_components,
      no_ground_reference_count,
      kept_count,
      static_cast<unsigned long long>(skipped_nonground_frames_),
      cost_ms);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "patchwork_refine_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PatchworkRefineNode node(nh, pnh);

  // Keep the original two-thread architecture so ground-model refresh and
  // non-ground refinement can run independently.
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
