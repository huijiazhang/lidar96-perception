#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rrcmc_msgs/SrvCmd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <dirent.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#ifdef HAVE_YAML_CPP
  #include <yaml-cpp/yaml.h>
#endif

class PointsObstacleM96Detector
{
public:
  PointsObstacleM96Detector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      config_mtime_(0),
      last_status_(0),
      last_level1_count_(0),
      last_level2_count_(0),
      last_valid_voxel_count_(0),
      last_raw_points_count_(0),
      last_ignored_voxel_count_(0),
      delay_active_(false),
      last_reload_check_time_(0.0)
  {
    set_default_config();

    std::string default_config_path = "points_obstacle_96.yaml";
    pnh_.param<std::string>("config_path", config_path_, default_config_path);
    config_path_ = find_config_file(config_path_);

    load_config_from_yaml(true);
    check_param_sanity();
    write_runtime_config_to_rosparam();

    pub_ = nh_.advertise<std_msgs::Int32>(output_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(status_topic_, 1);
    roi_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(roi_marker_topic_, 1);
    voxel_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(voxel_cloud_topic_, 1);
    ignore_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ignore_marker_topic_, 1);

    cloud_sub_ = nh_.subscribe(
        input_topic_, 1,
        &PointsObstacleM96Detector::pointcloud_callback,
        this);

    // /rrcmc_ii may not publish on some vehicles. In that case the
    // detector uses the normal-driving Level-1 range.
    rrcmc_ii_sub_ = nh_.subscribe(
        "/rrcmc_ii", 10,
        &PointsObstacleM96Detector::rrcmc_ii_callback,
        this);

    // /rrcmc_status field0[73]:
    //   '1' = feeding lane / inside cowshed
    //   '0' = outside cowshed
    // Only the right-side Y minimum is overridden inside the feeding lane.
    rrcmc_status_sub_ = nh_.subscribe(
        "/rrcmc_status", 10,
        &PointsObstacleM96Detector::rrcmc_status_callback,
        this);

    srv_cmd_server_ = nh_.advertiseService(
        srv_name_,
        &PointsObstacleM96Detector::srv_cmd_callback,
        this);

    status_timer_ = nh_.createTimer(
        ros::Duration(1.0),
        &PointsObstacleM96Detector::status_timer_callback,
        this);

    reload_timer_ = nh_.createTimer(
        ros::Duration(0.5),
        &PointsObstacleM96Detector::reload_timer_callback,
        this);

    rosparam_timer_ = nh_.createTimer(
        ros::Duration(std::max(0.05, rosparam_sync_interval_)),
        &PointsObstacleM96Detector::rosparam_timer_callback,
        this);

    last_level1_time_ = ros::Time(0);

    initialize_logging();
    log_startup_info();
  }

  ~PointsObstacleM96Detector()
  {
    if (!log_enable_)
      return;

    const std::chrono::system_clock::time_point now =
        std::chrono::system_clock::now();

    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_file_.is_open())
    {
      log_file_ << "[SHUTDOWN] time=" << format_wall_time(now) << '\n';
      log_file_.flush();
      log_file_.close();
    }
  }

private:
  typedef std::tuple<int, int, int> VoxelKey;

  enum RrcmcIiRangeMode
  {
    RRCMC_II_USE_YAML = 0,
    RRCMC_II_USE_MIN_RANGE = 1,
    RRCMC_II_USE_DEFAULT_RANGE = 2
  };

  struct RuntimeConfig
  {
    bool enable;

    double level1_x_min;
    double level1_x_max;
    double level1_normal_x_max;
    double level2_x_min;
    double level2_x_max;
    double y_min;
    double feeding_lane_y_min;
    double y_max;
    double z_min;
    double z_max;

    double voxel_size;
    int level1_threshold;
    int level2_threshold;
    double level1_delay;
    double level1_normal_delay;

    bool ignore_push_plate;
    double push_plate_x_min;
    double push_plate_x_max;
    double push_plate_y_min;
    double push_plate_y_max;
    double push_plate_z_min;
    double push_plate_z_max;

    double marker_lifetime;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber rrcmc_ii_sub_;
  ros::Subscriber rrcmc_status_sub_;
  ros::Publisher pub_;
  ros::Publisher status_pub_;
  ros::Publisher roi_marker_pub_;
  ros::Publisher voxel_cloud_pub_;
  ros::Publisher ignore_marker_pub_;
  ros::ServiceServer srv_cmd_server_;
  ros::Timer status_timer_;
  ros::Timer reload_timer_;
  ros::Timer rosparam_timer_;

  std::mutex config_mutex_;
  std::mutex log_mutex_;

  std::string config_path_;
  time_t config_mtime_;

  bool enable_;
  bool reload_yaml_;
  double reload_interval_;
  bool sync_rosparam_;
  double rosparam_sync_interval_;

  std::string srv_name_;
  int srv_cmd_code_;

  std::string input_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string roi_marker_topic_;
  std::string voxel_cloud_topic_;
  std::string ignore_marker_topic_;

  double level1_x_min_;
  // Turning range. It can be updated by the webpage service.
  double level1_x_max_;
  // Normal-driving range, also used before the first valid /rrcmc_ii message.
  double level1_normal_x_max_;
  double level2_x_min_;
  double level2_x_max_;
  double y_min_;
  double feeding_lane_y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  double voxel_size_;
  int level1_threshold_;
  int level2_threshold_;
  // Turning-state Level-1 hold time. The webpage service updates this value.
  double level1_delay_;
  // Normal-driving hold time, also used before the first valid /rrcmc_ii message.
  double level1_normal_delay_;

  bool ignore_push_plate_;
  double push_plate_x_min_;
  double push_plate_x_max_;
  double push_plate_y_min_;
  double push_plate_y_max_;
  double push_plate_z_min_;
  double push_plate_z_max_;

  double marker_lifetime_;

  // /rrcmc_ii only changes the active Level-1 maximum range in memory.
  // It never writes this temporary value back to YAML.
  RrcmcIiRangeMode rrcmc_ii_range_mode_;

  // Feeding-lane state from /rrcmc_status field0[73].
  // When true, the active right-side Y minimum uses feeding_lane_y_min_.
  // When false (or before a valid status arrives), YAML y_min_ is used.
  bool feeding_lane_;
  bool rrcmc_status_received_;

  int last_status_;
  int last_level1_count_;
  int last_level2_count_;
  int last_valid_voxel_count_;
  int last_raw_points_count_;
  int last_ignored_voxel_count_;
  bool delay_active_;
  ros::Time last_level1_time_;
  double last_reload_check_time_;

  // Rolling decision log. The detector keeps writing continuously while
  // retaining only the most recent 24 hours of segmented log files.
  bool log_enable_;
  std::string log_dir_;
  double log_result_period_sec_;
  int log_segment_minutes_;
  double log_retention_hours_;
  double log_cleanup_period_sec_;
  double log_flush_period_sec_;
  double input_timeout_sec_;

  std::ofstream log_file_;
  std::string current_log_segment_;
  bool log_directory_ready_;
  bool has_received_cloud_;
  bool input_timeout_active_;
  std::string last_rrcmc_ii_command_;

  std::chrono::system_clock::time_point node_start_wall_time_;
  std::chrono::system_clock::time_point last_cloud_wall_time_;
  std::chrono::system_clock::time_point last_result_log_wall_time_;
  std::chrono::system_clock::time_point last_log_flush_wall_time_;
  std::chrono::system_clock::time_point last_log_cleanup_wall_time_;

  void set_default_config()
  {
    enable_ = true;
    reload_yaml_ = true;
    reload_interval_ = 1.0;
    sync_rosparam_ = true;
    rosparam_sync_interval_ = 0.2;

    srv_name_ = "/srv_cmd01";
    srv_cmd_code_ = 83001;

    input_topic_ = "/patchwork_refine/no_ground";
    output_topic_ = "/srs_obstacle_m_96";
    status_topic_ = "/srs_obstacle_m_96_status";
    roi_marker_topic_ = "/srs_obstacle_m_96_roi_markers";
    voxel_cloud_topic_ = "/srs_obstacle_m_96_voxel_cloud";
    ignore_marker_topic_ = "/srs_obstacle_m_96_ignore_markers";

    level1_x_min_ = 0.0;
    level1_x_max_ = 1.7;
    level1_normal_x_max_ = 1.9;
    level2_x_min_ = 0.0;
    level2_x_max_ = 0.0;
    y_min_ = -0.8;
    feeding_lane_y_min_ = -0.30;
    y_max_ = 0.8;
    z_min_ = -1.6;
    z_max_ = 0.0;

    voxel_size_ = 0.05;
    level1_threshold_ = 20;
    level2_threshold_ = 20;
    level1_delay_ = 5.0;
    level1_normal_delay_ = 5.0;

    ignore_push_plate_ = true;
    push_plate_x_min_ = 0.0;
    push_plate_x_max_ = 1.0;
    push_plate_y_min_ = -1.17;
    push_plate_y_max_ = 0.8;
    push_plate_z_min_ = -1.65;
    push_plate_z_max_ = 0.0;

    marker_lifetime_ = 0.3;

    // Before any valid /rrcmc_ii message is received, use the normal range.
    rrcmc_ii_range_mode_ = RRCMC_II_USE_YAML;

    feeding_lane_ = false;
    rrcmc_status_received_ = false;

    log_enable_ = true;
    log_dir_ = "/home/casp/obstacle96_logs";
    log_result_period_sec_ = 1.0;
    log_segment_minutes_ = 10;
    log_retention_hours_ = 24.0;
    log_cleanup_period_sec_ = 300.0;
    log_flush_period_sec_ = 5.0;
    input_timeout_sec_ = 1.0;

    log_directory_ready_ = false;
    has_received_cloud_ = false;
    input_timeout_active_ = false;
    last_rrcmc_ii_command_ = "NONE";

    node_start_wall_time_ = std::chrono::system_clock::now();
    last_cloud_wall_time_ = node_start_wall_time_;
    last_result_log_wall_time_ = std::chrono::system_clock::time_point();
    last_log_flush_wall_time_ = node_start_wall_time_;
    last_log_cleanup_wall_time_ = std::chrono::system_clock::time_point();
  }

  static std::string find_config_file(const std::string& config_path)
  {
    if (config_path.empty())
      return config_path;

    if (config_path[0] == '/' ||
        config_path.find('/') != std::string::npos ||
        config_path.find('\\') != std::string::npos)
    {
      return config_path;
    }

    const std::vector<std::string> search_paths = {
      config_path,
      "./src/" + config_path,
      "./src/stop_obstacle_cpp/src/" + config_path,
      "../src/" + config_path,
      "../../src/" + config_path,
      "../../../src/" + config_path,
    };

    for (std::size_t i = 0; i < search_paths.size(); ++i)
    {
      if (std::ifstream(search_paths[i].c_str()).good())
        return search_paths[i];
    }

    return config_path;
  }

  RuntimeConfig get_runtime_config() const
  {
    RuntimeConfig cfg;
    cfg.enable = enable_;
    cfg.level1_x_min = level1_x_min_;
    cfg.level1_x_max = level1_x_max_;
    cfg.level1_normal_x_max = level1_normal_x_max_;
    cfg.level2_x_min = level2_x_min_;
    cfg.level2_x_max = level2_x_max_;
    cfg.y_min = y_min_;
    cfg.feeding_lane_y_min = feeding_lane_y_min_;
    cfg.y_max = y_max_;
    cfg.z_min = z_min_;
    cfg.z_max = z_max_;
    cfg.voxel_size = voxel_size_;
    cfg.level1_threshold = level1_threshold_;
    cfg.level2_threshold = level2_threshold_;
    cfg.level1_delay = level1_delay_;
    cfg.level1_normal_delay = level1_normal_delay_;
    cfg.ignore_push_plate = ignore_push_plate_;
    cfg.push_plate_x_min = push_plate_x_min_;
    cfg.push_plate_x_max = push_plate_x_max_;
    cfg.push_plate_y_min = push_plate_y_min_;
    cfg.push_plate_y_max = push_plate_y_max_;
    cfg.push_plate_z_min = push_plate_z_min_;
    cfg.push_plate_z_max = push_plate_z_max_;
    cfg.marker_lifetime = marker_lifetime_;
    return cfg;
  }

  void set_runtime_config(const RuntimeConfig& cfg)
  {
    const bool was_enabled = enable_;

    enable_ = cfg.enable;
    level1_x_min_ = cfg.level1_x_min;
    level1_x_max_ = cfg.level1_x_max;
    level1_normal_x_max_ = cfg.level1_normal_x_max;
    level2_x_min_ = cfg.level2_x_min;
    level2_x_max_ = cfg.level2_x_max;
    y_min_ = cfg.y_min;
    feeding_lane_y_min_ = cfg.feeding_lane_y_min;
    y_max_ = cfg.y_max;
    z_min_ = cfg.z_min;
    z_max_ = cfg.z_max;
    voxel_size_ = cfg.voxel_size;
    level1_threshold_ = cfg.level1_threshold;
    level2_threshold_ = cfg.level2_threshold;
    level1_delay_ = cfg.level1_delay;
    level1_normal_delay_ = cfg.level1_normal_delay;
    ignore_push_plate_ = cfg.ignore_push_plate;
    push_plate_x_min_ = cfg.push_plate_x_min;
    push_plate_x_max_ = cfg.push_plate_x_max;
    push_plate_y_min_ = cfg.push_plate_y_min;
    push_plate_y_max_ = cfg.push_plate_y_max;
    push_plate_z_min_ = cfg.push_plate_z_min;
    push_plate_z_max_ = cfg.push_plate_z_max;
    marker_lifetime_ = cfg.marker_lifetime;

    if (was_enabled && !enable_)
    {
      last_level1_time_ = ros::Time(0);
      delay_active_ = false;
      last_status_ = 0;
    }
  }

  bool validate_runtime_config(const RuntimeConfig& cfg, std::string& error) const
  {
    if (!std::isfinite(cfg.level1_x_min) ||
        !std::isfinite(cfg.level1_x_max) ||
        !std::isfinite(cfg.level1_normal_x_max) ||
        !std::isfinite(cfg.level2_x_min) ||
        !std::isfinite(cfg.level2_x_max) ||
        !std::isfinite(cfg.y_min) ||
        !std::isfinite(cfg.feeding_lane_y_min) ||
        !std::isfinite(cfg.y_max) ||
        !std::isfinite(cfg.z_min) ||
        !std::isfinite(cfg.z_max) ||
        !std::isfinite(cfg.voxel_size) ||
        !std::isfinite(cfg.level1_delay) ||
        !std::isfinite(cfg.level1_normal_delay) ||
        !std::isfinite(cfg.push_plate_x_min) ||
        !std::isfinite(cfg.push_plate_x_max) ||
        !std::isfinite(cfg.push_plate_y_min) ||
        !std::isfinite(cfg.push_plate_y_max) ||
        !std::isfinite(cfg.push_plate_z_min) ||
        !std::isfinite(cfg.push_plate_z_max) ||
        !std::isfinite(cfg.marker_lifetime))
    {
      error = "parameter contains NaN or infinity";
      return false;
    }

    if (cfg.level1_x_max <= cfg.level1_x_min)
    {
      error = "level1_x_max must be greater than level1_x_min";
      return false;
    }

    if (cfg.level1_normal_x_max <= cfg.level1_x_min)
    {
      error = "level1_normal_x_max must be greater than level1_x_min";
      return false;
    }

    if (cfg.level2_x_max < cfg.level2_x_min)
    {
      error = "level2_x_max must be greater than or equal to level2_x_min";
      return false;
    }

    if (cfg.y_max <= cfg.y_min)
    {
      error = "y_max must be greater than y_min";
      return false;
    }

    if (cfg.y_max <= cfg.feeding_lane_y_min)
    {
      error = "y_max must be greater than feeding_lane_y_min";
      return false;
    }

    if (cfg.z_max <= cfg.z_min)
    {
      error = "z_max must be greater than z_min";
      return false;
    }

    if (cfg.voxel_size <= 0.0)
    {
      error = "voxel_size must be greater than 0";
      return false;
    }

    if (cfg.level1_threshold < 0 || cfg.level2_threshold < 0)
    {
      error = "voxel threshold must be greater than or equal to 0";
      return false;
    }

    if (cfg.level1_delay < 0.0)
    {
      error = "level1_delay must be greater than or equal to 0";
      return false;
    }

    if (cfg.level1_normal_delay < 0.0)
    {
      error = "level1_normal_delay must be greater than or equal to 0";
      return false;
    }

    if (cfg.marker_lifetime < 0.0)
    {
      error = "marker_lifetime must be greater than or equal to 0";
      return false;
    }

    if (cfg.ignore_push_plate &&
        (cfg.push_plate_x_max <= cfg.push_plate_x_min ||
         cfg.push_plate_y_max <= cfg.push_plate_y_min ||
         cfg.push_plate_z_max <= cfg.push_plate_z_min))
    {
      error = "push plate ignore box max values must be greater than min values";
      return false;
    }

    return true;
  }

  void check_param_sanity() const
  {
    RuntimeConfig cfg = get_runtime_config();
    std::string error;
    if (!validate_runtime_config(cfg, error))
      ROS_WARN("Current obstacle config warning: %s", error.c_str());

    if (level2_x_min_ != level2_x_max_ &&
        std::fabs(level2_x_min_ - level1_x_max_) > 1e-6)
    {
      ROS_WARN("level2_x_min != level1_x_max");
    }
  }

  void load_config_from_yaml(bool initial)
  {
#ifdef HAVE_YAML_CPP
    if (!std::ifstream(config_path_.c_str()).good())
    {
      ROS_WARN("YAML config not found: %s. Use current/default values.", config_path_.c_str());
      config_mtime_ = 0;
      return;
    }

    try
    {
      YAML::Node config = YAML::LoadFile(config_path_);

      if (config["enable"])
        enable_ = config["enable"].as<bool>();
      if (config["reload_yaml"])
        reload_yaml_ = config["reload_yaml"].as<bool>();
      if (config["reload_interval"])
        reload_interval_ = std::max(0.1, config["reload_interval"].as<double>());
      if (config["sync_rosparam"])
        sync_rosparam_ = config["sync_rosparam"].as<bool>();
      if (config["rosparam_sync_interval"])
        rosparam_sync_interval_ = std::max(0.05, config["rosparam_sync_interval"].as<double>());

      if (config["srv_cmd_code"])
        srv_cmd_code_ = config["srv_cmd_code"].as<int>();

      // 话题名和服务名只在启动时创建，运行中修改后需要重启。
      if (initial)
      {
        if (config["srv_name"])
          srv_name_ = config["srv_name"].as<std::string>();
        if (config["input_topic"])
          input_topic_ = config["input_topic"].as<std::string>();
        if (config["output_topic"])
          output_topic_ = config["output_topic"].as<std::string>();
        if (config["status_topic"])
          status_topic_ = config["status_topic"].as<std::string>();
        if (config["roi_marker_topic"])
          roi_marker_topic_ = config["roi_marker_topic"].as<std::string>();
        if (config["voxel_cloud_topic"])
          voxel_cloud_topic_ = config["voxel_cloud_topic"].as<std::string>();
        if (config["ignore_marker_topic"])
          ignore_marker_topic_ = config["ignore_marker_topic"].as<std::string>();
      }

      RuntimeConfig cfg = get_runtime_config();

      if (config["level1_x_min"])
        cfg.level1_x_min = config["level1_x_min"].as<double>();
      if (config["level1_x_max"])
        cfg.level1_x_max = config["level1_x_max"].as<double>();
      if (config["level1_normal_x_max"])
        cfg.level1_normal_x_max = config["level1_normal_x_max"].as<double>();
      if (config["level2_x_min"])
        cfg.level2_x_min = config["level2_x_min"].as<double>();
      if (config["level2_x_max"])
        cfg.level2_x_max = config["level2_x_max"].as<double>();
      if (config["y_min"])
        cfg.y_min = config["y_min"].as<double>();
      if (config["feeding_lane_y_min"])
        cfg.feeding_lane_y_min = config["feeding_lane_y_min"].as<double>();
      if (config["y_max"])
        cfg.y_max = config["y_max"].as<double>();
      if (config["z_min"])
        cfg.z_min = config["z_min"].as<double>();
      if (config["z_max"])
        cfg.z_max = config["z_max"].as<double>();
      if (config["voxel_size"])
        cfg.voxel_size = config["voxel_size"].as<double>();
      if (config["level1_voxel_threshold"])
        cfg.level1_threshold = config["level1_voxel_threshold"].as<int>();
      if (config["level2_voxel_threshold"])
        cfg.level2_threshold = config["level2_voxel_threshold"].as<int>();
      if (config["level1_delay"])
        cfg.level1_delay = config["level1_delay"].as<double>();
      if (config["level1_normal_delay"])
        cfg.level1_normal_delay = config["level1_normal_delay"].as<double>();
      if (config["ignore_push_plate"])
        cfg.ignore_push_plate = config["ignore_push_plate"].as<bool>();
      if (config["push_plate_x_min"])
        cfg.push_plate_x_min = config["push_plate_x_min"].as<double>();
      if (config["push_plate_x_max"])
        cfg.push_plate_x_max = config["push_plate_x_max"].as<double>();
      if (config["push_plate_y_min"])
        cfg.push_plate_y_min = config["push_plate_y_min"].as<double>();
      if (config["push_plate_y_max"])
        cfg.push_plate_y_max = config["push_plate_y_max"].as<double>();
      if (config["push_plate_z_min"])
        cfg.push_plate_z_min = config["push_plate_z_min"].as<double>();
      if (config["push_plate_z_max"])
        cfg.push_plate_z_max = config["push_plate_z_max"].as<double>();
      if (config["marker_lifetime"])
        cfg.marker_lifetime = config["marker_lifetime"].as<double>();

      std::string error;
      if (!validate_runtime_config(cfg, error))
      {
        ROS_ERROR("YAML config rejected: %s", error.c_str());
        return;
      }

      set_runtime_config(cfg);

      struct stat sb;
      if (stat(config_path_.c_str(), &sb) == 0)
        config_mtime_ = sb.st_mtime;

      ROS_INFO("Loaded YAML config: %s", config_path_.c_str());
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Failed to load YAML config: %s", e.what());
    }
#else
    (void)initial;
    ROS_WARN("yaml-cpp not available. Using default config only.");
#endif
  }

  static std::string format_yaml_double(double value)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    std::string text = stream.str();

    while (text.size() > 2 && text.back() == '0')
      text.pop_back();
    if (!text.empty() && text.back() == '.')
      text.push_back('0');

    return text;
  }

  bool write_runtime_config_to_yaml(
      const RuntimeConfig& cfg,
      const std::set<std::string>& changed_keys,
      std::string& error)
  {
#ifdef HAVE_YAML_CPP
    if (changed_keys.empty())
      return true;

    try
    {
      std::ifstream input(config_path_.c_str());
      if (!input.is_open())
      {
        error = "cannot open YAML file: " + config_path_;
        return false;
      }

      std::vector<std::string> lines;
      std::string line;
      while (std::getline(input, line))
        lines.push_back(line);
      input.close();

      std::map<std::string, std::string> values;
      values["enable"] = cfg.enable ? "true" : "false";
      values["level1_x_min"] = format_yaml_double(cfg.level1_x_min);
      values["level1_x_max"] = format_yaml_double(cfg.level1_x_max);
      values["level1_normal_x_max"] = format_yaml_double(cfg.level1_normal_x_max);
      values["level2_x_min"] = format_yaml_double(cfg.level2_x_min);
      values["level2_x_max"] = format_yaml_double(cfg.level2_x_max);
      values["y_min"] = format_yaml_double(cfg.y_min);
      values["feeding_lane_y_min"] = format_yaml_double(cfg.feeding_lane_y_min);
      values["y_max"] = format_yaml_double(cfg.y_max);
      values["z_min"] = format_yaml_double(cfg.z_min);
      values["z_max"] = format_yaml_double(cfg.z_max);
      values["voxel_size"] = format_yaml_double(cfg.voxel_size);
      values["level1_voxel_threshold"] = std::to_string(cfg.level1_threshold);
      values["level2_voxel_threshold"] = std::to_string(cfg.level2_threshold);
      values["level1_delay"] = format_yaml_double(cfg.level1_delay);
      values["level1_normal_delay"] = format_yaml_double(cfg.level1_normal_delay);
      values["ignore_push_plate"] = cfg.ignore_push_plate ? "true" : "false";
      values["push_plate_x_min"] = format_yaml_double(cfg.push_plate_x_min);
      values["push_plate_x_max"] = format_yaml_double(cfg.push_plate_x_max);
      values["push_plate_y_min"] = format_yaml_double(cfg.push_plate_y_min);
      values["push_plate_y_max"] = format_yaml_double(cfg.push_plate_y_max);
      values["push_plate_z_min"] = format_yaml_double(cfg.push_plate_z_min);
      values["push_plate_z_max"] = format_yaml_double(cfg.push_plate_z_max);
      values["marker_lifetime"] = format_yaml_double(cfg.marker_lifetime);

      std::set<std::string> written_keys;

      for (std::string& current_line : lines)
      {
        const std::size_t first = current_line.find_first_not_of(" \t");
        if (first == std::string::npos || current_line[first] == '#')
          continue;

        const std::size_t colon = current_line.find(':', first);
        if (colon == std::string::npos)
          continue;

        std::string key = current_line.substr(first, colon - first);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t'))
          key.pop_back();

        if (changed_keys.count(key) == 0 || values.count(key) == 0)
          continue;

        const std::size_t comment_pos = current_line.find('#', colon + 1);
        std::string inline_comment;
        if (comment_pos != std::string::npos)
        {
          inline_comment = current_line.substr(comment_pos);
          while (!inline_comment.empty() &&
                 (inline_comment.front() == ' ' || inline_comment.front() == '\t'))
          {
            inline_comment.erase(inline_comment.begin());
          }
        }

        current_line = current_line.substr(0, colon + 1) + " " + values[key];
        if (!inline_comment.empty())
          current_line += "  " + inline_comment;

        written_keys.insert(key);
      }

      bool appended_header = false;
      for (const std::string& key : changed_keys)
      {
        if (written_keys.count(key) != 0 || values.count(key) == 0)
          continue;

        if (!appended_header)
        {
          lines.push_back("");
          lines.push_back("# ================= Service写入的缺失参数 =================");
          appended_header = true;
        }
        lines.push_back(key + ": " + values[key]);
      }

      const std::string temp_path = config_path_ + ".tmp";
      {
        std::ofstream output(temp_path.c_str(), std::ios::out | std::ios::trunc);
        if (!output.is_open())
        {
          error = "cannot open temporary YAML file: " + temp_path;
          return false;
        }

        for (std::size_t i = 0; i < lines.size(); ++i)
        {
          output << lines[i];
          if (i + 1 < lines.size())
            output << '\n';
        }
        output << '\n';
        output.flush();

        if (!output.good())
        {
          error = "failed while writing temporary YAML file";
          return false;
        }
      }

      if (std::rename(temp_path.c_str(), config_path_.c_str()) != 0)
      {
        std::ostringstream stream;
        stream << "rename YAML failed, errno=" << errno;
        error = stream.str();
        std::remove(temp_path.c_str());
        return false;
      }

      struct stat sb;
      if (stat(config_path_.c_str(), &sb) == 0)
        config_mtime_ = sb.st_mtime;

      return true;
    }
    catch (const std::exception& e)
    {
      error = e.what();
      return false;
    }
#else
    (void)cfg;
    (void)changed_keys;
    error = "yaml-cpp support is disabled";
    return false;
#endif
  }

  void write_runtime_config_to_rosparam()
  {
    pnh_.setParam("enable", enable_);
    pnh_.setParam("level1_x_min", level1_x_min_);
    pnh_.setParam("level1_x_max", level1_x_max_);
    pnh_.setParam("level1_normal_x_max", level1_normal_x_max_);
    pnh_.setParam("level2_x_min", level2_x_min_);
    pnh_.setParam("level2_x_max", level2_x_max_);
    pnh_.setParam("y_min", y_min_);
    pnh_.setParam("feeding_lane_y_min", feeding_lane_y_min_);
    pnh_.setParam("y_max", y_max_);
    pnh_.setParam("z_min", z_min_);
    pnh_.setParam("z_max", z_max_);
    pnh_.setParam("voxel_size", voxel_size_);
    pnh_.setParam("level1_voxel_threshold", level1_threshold_);
    pnh_.setParam("level2_voxel_threshold", level2_threshold_);
    pnh_.setParam("level1_delay", level1_delay_);
    pnh_.setParam("level1_normal_delay", level1_normal_delay_);
    pnh_.setParam("ignore_push_plate", ignore_push_plate_);
    pnh_.setParam("push_plate_x_min", push_plate_x_min_);
    pnh_.setParam("push_plate_x_max", push_plate_x_max_);
    pnh_.setParam("push_plate_y_min", push_plate_y_min_);
    pnh_.setParam("push_plate_y_max", push_plate_y_max_);
    pnh_.setParam("push_plate_z_min", push_plate_z_min_);
    pnh_.setParam("push_plate_z_max", push_plate_z_max_);
    pnh_.setParam("marker_lifetime", marker_lifetime_);
    pnh_.setParam("srv_cmd_code", srv_cmd_code_);
  }

  void read_runtime_config_from_rosparam()
  {
    if (!sync_rosparam_)
      return;

    const RuntimeConfig old_cfg = get_runtime_config();
    const double old_active_x_max = get_active_level1_x_max();
    const std::string old_rrcmc_command = last_rrcmc_ii_command_;

    RuntimeConfig cfg = old_cfg;

    pnh_.getParam("enable", cfg.enable);
    pnh_.getParam("level1_x_min", cfg.level1_x_min);
    pnh_.getParam("level1_x_max", cfg.level1_x_max);
    pnh_.getParam("level1_normal_x_max", cfg.level1_normal_x_max);
    pnh_.getParam("level2_x_min", cfg.level2_x_min);
    pnh_.getParam("level2_x_max", cfg.level2_x_max);
    pnh_.getParam("y_min", cfg.y_min);
    pnh_.getParam("feeding_lane_y_min", cfg.feeding_lane_y_min);
    pnh_.getParam("y_max", cfg.y_max);
    pnh_.getParam("z_min", cfg.z_min);
    pnh_.getParam("z_max", cfg.z_max);
    pnh_.getParam("voxel_size", cfg.voxel_size);
    pnh_.getParam("level1_voxel_threshold", cfg.level1_threshold);
    pnh_.getParam("level2_voxel_threshold", cfg.level2_threshold);
    pnh_.getParam("level1_delay", cfg.level1_delay);
    pnh_.getParam("level1_normal_delay", cfg.level1_normal_delay);
    pnh_.getParam("ignore_push_plate", cfg.ignore_push_plate);
    pnh_.getParam("push_plate_x_min", cfg.push_plate_x_min);
    pnh_.getParam("push_plate_x_max", cfg.push_plate_x_max);
    pnh_.getParam("push_plate_y_min", cfg.push_plate_y_min);
    pnh_.getParam("push_plate_y_max", cfg.push_plate_y_max);
    pnh_.getParam("push_plate_z_min", cfg.push_plate_z_min);
    pnh_.getParam("push_plate_z_max", cfg.push_plate_z_max);
    pnh_.getParam("marker_lifetime", cfg.marker_lifetime);

    int new_cmd_code = srv_cmd_code_;
    pnh_.getParam("srv_cmd_code", new_cmd_code);

    std::string error;
    if (!validate_runtime_config(cfg, error))
    {
      ROS_WARN_THROTTLE(1.0, "Ignore invalid rosparam config: %s", error.c_str());
      return;
    }

    set_runtime_config(cfg);
    srv_cmd_code_ = new_cmd_code;

    log_runtime_config_changes(
        old_cfg,
        get_runtime_config(),
        "ROSPARAM",
        old_active_x_max,
        get_active_level1_x_max(),
        old_rrcmc_command,
        last_rrcmc_ii_command_);
  }

  void reload_config_if_needed()
  {
    if (!reload_yaml_)
      return;

    const double now = ros::Time::now().toSec();
    if (now - last_reload_check_time_ < reload_interval_)
      return;

    last_reload_check_time_ = now;

    struct stat sb;
    if (stat(config_path_.c_str(), &sb) != 0)
      return;

    if (sb.st_mtime == config_mtime_)
      return;

    const RuntimeConfig old_cfg = get_runtime_config();
    const double old_active_x_max = get_active_level1_x_max();
    const std::string old_rrcmc_command = last_rrcmc_ii_command_;

    ROS_INFO("YAML config changed. Reloading...");
    load_config_from_yaml(false);
    check_param_sanity();
    write_runtime_config_to_rosparam();

    log_runtime_config_changes(
        old_cfg,
        get_runtime_config(),
        "YAML",
        old_active_x_max,
        get_active_level1_x_max(),
        old_rrcmc_command,
        last_rrcmc_ii_command_);
  }

  double get_active_level1_x_max() const
  {
    // 81003;999 means the vehicle is turning. The turning range is the
    // webpage/YAML-configurable level1_x_max value.
    if (rrcmc_ii_range_mode_ == RRCMC_II_USE_MIN_RANGE)
      return level1_x_max_;

    // 81003;0 and the initial state before a valid message both use the
    // normal-driving range. Unsupported messages retain the previous mode.
    return level1_normal_x_max_;
  }

  double get_active_level1_delay() const
  {
    // Keep the delay state selection aligned with the active Level-1 range.
    if (rrcmc_ii_range_mode_ == RRCMC_II_USE_MIN_RANGE)
      return level1_delay_;

    return level1_normal_delay_;
  }


  static bool nearly_equal(double lhs, double rhs)
  {
    return std::fabs(lhs - rhs) <= 1e-9;
  }

  static std::string format_wall_time(
      const std::chrono::system_clock::time_point& wall_time)
  {
    const std::time_t value =
        std::chrono::system_clock::to_time_t(wall_time);
    std::tm local_tm;
    localtime_r(&value, &local_tm);

    const long long milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            wall_time.time_since_epoch()).count() % 1000;

    char buffer[64];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "%04d-%02d-%02d %02d:%02d:%02d.%03lld",
        local_tm.tm_year + 1900,
        local_tm.tm_mon + 1,
        local_tm.tm_mday,
        local_tm.tm_hour,
        local_tm.tm_min,
        local_tm.tm_sec,
        milliseconds);
    return std::string(buffer);
  }

  static double elapsed_wall_seconds(
      const std::chrono::system_clock::time_point& newer,
      const std::chrono::system_clock::time_point& older)
  {
    return std::chrono::duration_cast<std::chrono::duration<double> >(
        newer - older).count();
  }

  std::string make_log_segment_name(
      const std::chrono::system_clock::time_point& wall_time) const
  {
    const std::time_t value =
        std::chrono::system_clock::to_time_t(wall_time);
    std::tm local_tm;
    localtime_r(&value, &local_tm);

    const int segment_minutes = std::max(1, log_segment_minutes_);
    const int segment_start_minute =
        (local_tm.tm_min / segment_minutes) * segment_minutes;

    char buffer[128];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "obstacle96_%04d%02d%02d_%02d%02d.log",
        local_tm.tm_year + 1900,
        local_tm.tm_mon + 1,
        local_tm.tm_mday,
        local_tm.tm_hour,
        segment_start_minute);
    return std::string(buffer);
  }

  static bool is_obstacle_log_file(const std::string& name)
  {
    const std::string prefix = "obstacle96_";
    const std::string suffix = ".log";

    return name.size() > prefix.size() + suffix.size() &&
           name.compare(0, prefix.size(), prefix) == 0 &&
           name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  bool ensure_log_directory()
  {
    if (log_directory_ready_)
      return true;

    struct stat info;
    if (stat(log_dir_.c_str(), &info) == 0)
    {
      if (S_ISDIR(info.st_mode))
      {
        log_directory_ready_ = true;
        return true;
      }

      ROS_ERROR("Obstacle96 log path exists but is not a directory: %s",
                log_dir_.c_str());
      return false;
    }

    if (mkdir(log_dir_.c_str(), 0755) != 0 && errno != EEXIST)
    {
      ROS_ERROR("Failed to create obstacle96 log directory %s, errno=%d",
                log_dir_.c_str(), errno);
      return false;
    }

    log_directory_ready_ = true;
    return true;
  }

  std::string current_reason(
      int status,
      int level1_count,
      int level2_count) const
  {
    (void)level1_count;
    (void)level2_count;

    if (!enable_)
      return "DISABLED";
    if (status == 1 && delay_active_)
      return "LEVEL1_DELAY";
    if (status == 1)
      return "LEVEL1_TRIGGER";
    if (status == 2)
      return "LEVEL2_TRIGGER";
    return "CLEAR";
  }

  std::string build_config_log_line(
      const std::chrono::system_clock::time_point& wall_time) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "[CONFIG] time=" << format_wall_time(wall_time)
           << " enable=" << (enable_ ? 1 : 0)
           << " L1_threshold=" << level1_threshold_
           << " L2_threshold=" << level2_threshold_
           << " L1_X=[" << level1_x_min_ << ","
           << get_active_level1_x_max() << "]"
           << " L1_X_config=[" << level1_x_min_ << ","
           << level1_x_max_ << "]"
           << " L1_X_normal_config=[" << level1_x_min_ << ","
           << level1_normal_x_max_ << "]"
           << " L2_X=[" << level2_x_min_ << ","
           << level2_x_max_ << "]"
           << " Y=[" << y_min_ << "," << y_max_ << "]"
           << " Y_feeding=[" << feeding_lane_y_min_ << "," << y_max_ << "]"
           << " Z=[" << z_min_ << "," << z_max_ << "]"
           << " voxel=" << std::setprecision(3) << voxel_size_
           << std::setprecision(2)
           << " delay_sec=" << get_active_level1_delay()
           << " delay_turning_config=" << level1_delay_
           << " delay_normal_config=" << level1_normal_delay_
           << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\"";
    return stream.str();
  }

  bool ensure_log_file_locked(
      const std::chrono::system_clock::time_point& wall_time)
  {
    if (!ensure_log_directory())
      return false;

    const std::string segment_name = make_log_segment_name(wall_time);
    if (log_file_.is_open() && current_log_segment_ == segment_name)
      return true;

    if (log_file_.is_open())
    {
      log_file_.flush();
      log_file_.close();
    }

    const std::string full_path = log_dir_ + "/" + segment_name;
    log_file_.open(full_path.c_str(), std::ios::out | std::ios::app);
    if (!log_file_.is_open())
    {
      ROS_ERROR("Failed to open obstacle96 log file: %s", full_path.c_str());
      current_log_segment_.clear();
      return false;
    }

    current_log_segment_ = segment_name;
    log_file_ << build_config_log_line(wall_time) << '\n';
    log_file_.flush();
    last_log_flush_wall_time_ = wall_time;

    ROS_INFO("Obstacle96 decision log opened: %s", full_path.c_str());
    return true;
  }

  void cleanup_old_logs_locked(
      const std::chrono::system_clock::time_point& wall_time)
  {
    if (!log_directory_ready_ || log_retention_hours_ <= 0.0)
      return;

    DIR* directory = opendir(log_dir_.c_str());
    if (directory == NULL)
    {
      ROS_WARN("Cannot scan obstacle96 log directory: %s", log_dir_.c_str());
      return;
    }

    const std::time_t now_time =
        std::chrono::system_clock::to_time_t(wall_time);
    const double retention_seconds = log_retention_hours_ * 3600.0;

    struct dirent* entry = NULL;
    while ((entry = readdir(directory)) != NULL)
    {
      const std::string name(entry->d_name);
      if (!is_obstacle_log_file(name) || name == current_log_segment_)
        continue;

      const std::string full_path = log_dir_ + "/" + name;
      struct stat info;
      if (stat(full_path.c_str(), &info) != 0)
        continue;

      if (std::difftime(now_time, info.st_mtime) <= retention_seconds)
        continue;

      if (std::remove(full_path.c_str()) == 0)
      {
        ROS_INFO("Deleted expired obstacle96 log: %s", full_path.c_str());
      }
      else
      {
        ROS_WARN("Failed to delete expired obstacle96 log: %s, errno=%d",
                 full_path.c_str(), errno);
      }
    }

    closedir(directory);
    last_log_cleanup_wall_time_ = wall_time;
  }

  void write_log_line_at(
      const std::string& line,
      const std::chrono::system_clock::time_point& wall_time,
      bool flush_now)
  {
    if (!log_enable_)
      return;

    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!ensure_log_file_locked(wall_time))
      return;

    log_file_ << line << '\n';

    const bool flush_due =
        elapsed_wall_seconds(wall_time, last_log_flush_wall_time_) >=
        log_flush_period_sec_;

    if (flush_now || flush_due)
    {
      log_file_.flush();
      last_log_flush_wall_time_ = wall_time;
    }
  }

  std::string build_result_fields(
      const std::chrono::system_clock::time_point& wall_time,
      int status,
      int level1_count,
      int level2_count) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "time=" << format_wall_time(wall_time)
           << " status=" << status
           << " reason=" << current_reason(
                  status, level1_count, level2_count)
           << " L1=" << level1_count
           << " L2=" << level2_count
           << " delay=" << (delay_active_ ? 1 : 0)
           << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\""
           << " X=[" << level1_x_min_ << ","
           << get_active_level1_x_max() << "]";
    return stream.str();
  }

  void log_periodic_result(
      const std::chrono::system_clock::time_point& wall_time,
      int status,
      int level1_count,
      int level2_count)
  {
    if (!log_enable_)
      return;

    if (last_result_log_wall_time_.time_since_epoch().count() != 0 &&
        elapsed_wall_seconds(
            wall_time, last_result_log_wall_time_) <
            log_result_period_sec_)
    {
      return;
    }

    write_log_line_at(
        "[RESULT] " +
            build_result_fields(
                wall_time, status, level1_count, level2_count),
        wall_time,
        false);
    last_result_log_wall_time_ = wall_time;
  }

  void log_state_change(
      const std::chrono::system_clock::time_point& wall_time,
      int old_status,
      int new_status,
      int level1_count,
      int level2_count)
  {
    std::ostringstream stream;
    stream << "[STATE_CHANGE] "
           << build_result_fields(
                  wall_time, new_status, level1_count, level2_count)
           << " previous_status=" << old_status;
    write_log_line_at(stream.str(), wall_time, true);
    last_result_log_wall_time_ = wall_time;
  }

  void log_delay_change(
      const std::chrono::system_clock::time_point& wall_time,
      bool old_delay,
      bool new_delay,
      int status,
      int level1_count,
      int level2_count)
  {
    std::ostringstream stream;
    stream << "[DELAY_CHANGE] "
           << build_result_fields(
                  wall_time, status, level1_count, level2_count)
           << " previous_delay=" << (old_delay ? 1 : 0)
           << " new_delay=" << (new_delay ? 1 : 0);
    write_log_line_at(stream.str(), wall_time, true);
    last_result_log_wall_time_ = wall_time;
  }

  static void append_config_change(
      std::vector<std::string>& changes,
      const std::string& key,
      double old_value,
      double new_value)
  {
    if (nearly_equal(old_value, new_value))
      return;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << key << ":" << old_value << "->" << new_value;
    changes.push_back(stream.str());
  }

  static void append_config_change(
      std::vector<std::string>& changes,
      const std::string& key,
      int old_value,
      int new_value)
  {
    if (old_value == new_value)
      return;

    std::ostringstream stream;
    stream << key << ":" << old_value << "->" << new_value;
    changes.push_back(stream.str());
  }

  static void append_config_change(
      std::vector<std::string>& changes,
      const std::string& key,
      bool old_value,
      bool new_value)
  {
    if (old_value == new_value)
      return;

    std::ostringstream stream;
    stream << key << ":" << (old_value ? 1 : 0)
           << "->" << (new_value ? 1 : 0);
    changes.push_back(stream.str());
  }

  void log_runtime_config_changes(
      const RuntimeConfig& old_cfg,
      const RuntimeConfig& new_cfg,
      const std::string& source,
      double old_active_x_max,
      double new_active_x_max,
      const std::string& old_rrcmc_command,
      const std::string& new_rrcmc_command)
  {
    if (!log_enable_)
      return;

    const std::chrono::system_clock::time_point wall_time =
        std::chrono::system_clock::now();

    if (old_cfg.enable != new_cfg.enable)
    {
      std::ostringstream stream;
      stream << "[ENABLE_CHANGE] time=" << format_wall_time(wall_time)
             << " source=" << source
             << " enable=" << (old_cfg.enable ? 1 : 0)
             << "->" << (new_cfg.enable ? 1 : 0);
      write_log_line_at(stream.str(), wall_time, true);
    }

    if (!nearly_equal(old_cfg.level1_x_min, new_cfg.level1_x_min) ||
        !nearly_equal(old_active_x_max, new_active_x_max) ||
        old_rrcmc_command != new_rrcmc_command)
    {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(2);
      stream << "[RANGE_CHANGE] time=" << format_wall_time(wall_time)
             << " source=" << source
             << " rrcmc_ii=\"" << new_rrcmc_command << "\""
             << " X=[" << old_cfg.level1_x_min << ","
             << old_active_x_max << "]->["
             << new_cfg.level1_x_min << ","
             << new_active_x_max << "]";
      write_log_line_at(stream.str(), wall_time, true);
    }

    std::vector<std::string> changes;
    append_config_change(
        changes, "level1_x_max_config",
        old_cfg.level1_x_max, new_cfg.level1_x_max);
    append_config_change(
        changes, "level1_normal_x_max_config",
        old_cfg.level1_normal_x_max, new_cfg.level1_normal_x_max);
    append_config_change(
        changes, "level2_x_min",
        old_cfg.level2_x_min, new_cfg.level2_x_min);
    append_config_change(
        changes, "level2_x_max",
        old_cfg.level2_x_max, new_cfg.level2_x_max);
    append_config_change(changes, "y_min", old_cfg.y_min, new_cfg.y_min);
    append_config_change(
        changes, "feeding_lane_y_min",
        old_cfg.feeding_lane_y_min, new_cfg.feeding_lane_y_min);
    append_config_change(changes, "y_max", old_cfg.y_max, new_cfg.y_max);
    append_config_change(changes, "z_min", old_cfg.z_min, new_cfg.z_min);
    append_config_change(changes, "z_max", old_cfg.z_max, new_cfg.z_max);
    append_config_change(
        changes, "voxel_size",
        old_cfg.voxel_size, new_cfg.voxel_size);
    append_config_change(
        changes, "level1_threshold",
        old_cfg.level1_threshold, new_cfg.level1_threshold);
    append_config_change(
        changes, "level2_threshold",
        old_cfg.level2_threshold, new_cfg.level2_threshold);
    append_config_change(
        changes, "level1_delay",
        old_cfg.level1_delay, new_cfg.level1_delay);
    append_config_change(
        changes, "level1_normal_delay",
        old_cfg.level1_normal_delay, new_cfg.level1_normal_delay);
    append_config_change(
        changes, "ignore_push_plate",
        old_cfg.ignore_push_plate, new_cfg.ignore_push_plate);
    append_config_change(
        changes, "push_plate_x_min",
        old_cfg.push_plate_x_min, new_cfg.push_plate_x_min);
    append_config_change(
        changes, "push_plate_x_max",
        old_cfg.push_plate_x_max, new_cfg.push_plate_x_max);
    append_config_change(
        changes, "push_plate_y_min",
        old_cfg.push_plate_y_min, new_cfg.push_plate_y_min);
    append_config_change(
        changes, "push_plate_y_max",
        old_cfg.push_plate_y_max, new_cfg.push_plate_y_max);
    append_config_change(
        changes, "push_plate_z_min",
        old_cfg.push_plate_z_min, new_cfg.push_plate_z_min);
    append_config_change(
        changes, "push_plate_z_max",
        old_cfg.push_plate_z_max, new_cfg.push_plate_z_max);

    if (!changes.empty())
    {
      std::ostringstream stream;
      stream << "[CONFIG_CHANGE] time=" << format_wall_time(wall_time)
             << " source=" << source
             << " changes=\"";
      for (std::size_t i = 0; i < changes.size(); ++i)
      {
        if (i != 0)
          stream << ",";
        stream << changes[i];
      }
      stream << "\"";
      write_log_line_at(stream.str(), wall_time, true);
    }
  }

  void initialize_logging()
  {
    pnh_.param<bool>("log_enable", log_enable_, log_enable_);
    pnh_.param<std::string>("log_dir", log_dir_, log_dir_);
    pnh_.param<double>(
        "log_result_period_sec",
        log_result_period_sec_,
        log_result_period_sec_);
    pnh_.param<int>(
        "log_segment_minutes",
        log_segment_minutes_,
        log_segment_minutes_);
    pnh_.param<double>(
        "log_retention_hours",
        log_retention_hours_,
        log_retention_hours_);
    pnh_.param<double>(
        "log_cleanup_period_sec",
        log_cleanup_period_sec_,
        log_cleanup_period_sec_);
    pnh_.param<double>(
        "log_flush_period_sec",
        log_flush_period_sec_,
        log_flush_period_sec_);
    pnh_.param<double>(
        "input_timeout_sec",
        input_timeout_sec_,
        input_timeout_sec_);

    log_result_period_sec_ = std::max(0.1, log_result_period_sec_);
    log_segment_minutes_ = std::max(1, log_segment_minutes_);
    log_retention_hours_ = std::max(0.1, log_retention_hours_);
    log_cleanup_period_sec_ = std::max(10.0, log_cleanup_period_sec_);
    log_flush_period_sec_ = std::max(0.1, log_flush_period_sec_);
    input_timeout_sec_ = std::max(0.1, input_timeout_sec_);

    if (!log_enable_)
      return;

    const std::chrono::system_clock::time_point wall_time =
        std::chrono::system_clock::now();

    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!ensure_log_directory())
    {
      ROS_ERROR("Obstacle96 file logging disabled because log directory is unavailable");
      log_enable_ = false;
      return;
    }

    cleanup_old_logs_locked(wall_time);
    if (ensure_log_file_locked(wall_time))
    {
      log_file_ << "[STARTUP] time=" << format_wall_time(wall_time)
                << " node=points_obstacle_m_96_detector"
                << " input=" << input_topic_
                << " output=" << output_topic_
                << '\n';
      log_file_.flush();
      last_log_flush_wall_time_ = wall_time;
    }
  }

  void log_maintenance()
  {
    if (!log_enable_)
      return;

    const std::chrono::system_clock::time_point wall_time =
        std::chrono::system_clock::now();

    bool timeout_started = false;
    double no_cloud_ms = 0.0;

    const std::chrono::system_clock::time_point reference_time =
        has_received_cloud_ ? last_cloud_wall_time_ : node_start_wall_time_;
    const double no_cloud_seconds =
        elapsed_wall_seconds(wall_time, reference_time);

    if (!input_timeout_active_ && no_cloud_seconds >= input_timeout_sec_)
    {
      input_timeout_active_ = true;
      timeout_started = true;
      no_cloud_ms = no_cloud_seconds * 1000.0;
    }

    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      if (ensure_log_file_locked(wall_time))
      {
        if (last_log_cleanup_wall_time_.time_since_epoch().count() == 0 ||
            elapsed_wall_seconds(
                wall_time, last_log_cleanup_wall_time_) >=
                log_cleanup_period_sec_)
        {
          cleanup_old_logs_locked(wall_time);
        }

        if (elapsed_wall_seconds(
                wall_time, last_log_flush_wall_time_) >=
            log_flush_period_sec_)
        {
          log_file_.flush();
          last_log_flush_wall_time_ = wall_time;
        }
      }
    }

    if (timeout_started)
    {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(0);
      stream << "[INPUT_TIMEOUT] time=" << format_wall_time(wall_time)
             << " no_cloud_ms=" << no_cloud_ms
             << " last_status=" << last_status_
             << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\""
             << std::setprecision(2)
             << " X=[" << level1_x_min_ << ","
             << get_active_level1_x_max() << "]";
      write_log_line_at(stream.str(), wall_time, true);
    }
  }

  static std::string trim_copy(const std::string& input)
  {
    const std::size_t first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
      return std::string();

    const std::size_t last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
  }

  double get_active_y_min() const
  {
    // Inside feeding lane, use its independently configurable right boundary.
    // Coordinate convention: +Y left, -Y right.
    // Outside feeding lane, use y_min_ from YAML / webpage service.
    return feeding_lane_ ? feeding_lane_y_min_ : y_min_;
  }

  void rrcmc_status_callback(const std_msgs::String::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);

    const std::string& data = msg->data;
    const std::size_t separator = data.find(';');
    const std::string field0 =
        (separator == std::string::npos) ? data : data.substr(0, separator);

    if (field0.size() <= 73)
    {
      ROS_WARN_THROTTLE(
          2.0,
          "/rrcmc_status field0 too short: len=%zu, need index 73; "
          "keep feeding_lane=%d",
          field0.size(),
          feeding_lane_ ? 1 : 0);
      return;
    }

    const char bit = field0[73];
    if (bit != '0' && bit != '1')
    {
      ROS_WARN_THROTTLE(
          2.0,
          "/rrcmc_status field0[73] is '%c', expected 0/1; "
          "keep feeding_lane=%d",
          bit,
          feeding_lane_ ? 1 : 0);
      return;
    }

    const bool new_feeding_lane = (bit == '1');
    const bool changed =
        !rrcmc_status_received_ || new_feeding_lane != feeding_lane_;

    feeding_lane_ = new_feeding_lane;
    rrcmc_status_received_ = true;

    if (changed)
    {
      ROS_INFO(
          "/rrcmc_status field0[73]=%c -> feeding_lane=%d, "
          "active_y_min=%.2f m (yaml_y_min=%.2f m)",
          bit,
          feeding_lane_ ? 1 : 0,
          get_active_y_min(),
          y_min_);
    }
  }

  void rrcmc_ii_callback(const std_msgs::Header::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);

    // The real vehicle publishes the command in std_msgs/Header.frame_id.
    const std::string command = trim_copy(msg->frame_id);
    const double old_active_x_max = get_active_level1_x_max();
    const std::string old_command = last_rrcmc_ii_command_;

    if (command == "81003;999")
    {
      rrcmc_ii_range_mode_ = RRCMC_II_USE_MIN_RANGE;
      last_rrcmc_ii_command_ = command;

      const double new_active_x_max = get_active_level1_x_max();
      ROS_INFO(
          "Received /rrcmc_ii frame_id='%s': active level1_x_max changed to %.2f m "
          "(memory only, YAML unchanged)",
          command.c_str(),
          new_active_x_max);

      if (!nearly_equal(old_active_x_max, new_active_x_max) ||
          old_command != last_rrcmc_ii_command_)
      {
        const std::chrono::system_clock::time_point wall_time =
            std::chrono::system_clock::now();
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2);
        stream << "[RANGE_CHANGE] time=" << format_wall_time(wall_time)
               << " source=RRCMC"
               << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\""
               << " X=[" << level1_x_min_ << "," << old_active_x_max
               << "]->[" << level1_x_min_ << "," << new_active_x_max << "]";
        write_log_line_at(stream.str(), wall_time, true);
      }
      return;
    }

    if (command == "81003;0")
    {
      rrcmc_ii_range_mode_ = RRCMC_II_USE_DEFAULT_RANGE;
      last_rrcmc_ii_command_ = command;

      const double new_active_x_max = get_active_level1_x_max();
      ROS_INFO(
          "Received /rrcmc_ii frame_id='%s': active level1_x_max changed to %.2f m "
          "(memory only, YAML unchanged)",
          command.c_str(),
          new_active_x_max);

      if (!nearly_equal(old_active_x_max, new_active_x_max) ||
          old_command != last_rrcmc_ii_command_)
      {
        const std::chrono::system_clock::time_point wall_time =
            std::chrono::system_clock::now();
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2);
        stream << "[RANGE_CHANGE] time=" << format_wall_time(wall_time)
               << " source=RRCMC"
               << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\""
               << " X=[" << level1_x_min_ << "," << old_active_x_max
               << "]->[" << level1_x_min_ << "," << new_active_x_max << "]";
        write_log_line_at(stream.str(), wall_time, true);
      }
      return;
    }

    // Invalid or unrelated messages do not change the current active value.
    ROS_WARN_THROTTLE(
        2.0,
        "Ignore unsupported /rrcmc_ii frame_id: '%s'; keep active level1_x_max=%.2f m",
        command.c_str(),
        get_active_level1_x_max());
  }


  static bool is_valid_number(double value)
  {
    return std::isfinite(value);
  }

  static float rgb_to_float(uint8_t r, uint8_t g, uint8_t b)
  {
    const uint32_t rgb_int =
        (static_cast<uint32_t>(r) << 16) |
        (static_cast<uint32_t>(g) << 8) |
        static_cast<uint32_t>(b);

    float result;
    std::memcpy(&result, &rgb_int, sizeof(float));
    return result;
  }

  VoxelKey make_voxel_key(double x, double y, double z) const
  {
    const int ix = static_cast<int>(std::floor(x / voxel_size_));
    const int iy = static_cast<int>(std::floor(y / voxel_size_));
    const int iz = static_cast<int>(std::floor(z / voxel_size_));
    return std::make_tuple(ix, iy, iz);
  }

  void voxel_key_to_center(const VoxelKey& key, double& x, double& y, double& z) const
  {
    const int ix = std::get<0>(key);
    const int iy = std::get<1>(key);
    const int iz = std::get<2>(key);
    x = (ix + 0.5) * voxel_size_;
    y = (iy + 0.5) * voxel_size_;
    z = (iz + 0.5) * voxel_size_;
  }

  std_msgs::Header make_header(const std_msgs::Header& input_header) const
  {
    std_msgs::Header header = input_header;
    if (header.stamp.toSec() <= 0.0)
      header.stamp = ros::Time::now();
    if (header.frame_id.empty())
      header.frame_id = "rslidar";
    return header;
  }

  std::string build_runtime_config_json(const RuntimeConfig& cfg) const
  {
    std::ostringstream stream;
    stream << std::setprecision(15);
    stream << "{";
    stream << "\"enable\":" << (cfg.enable ? "true" : "false") << ",";
    stream << "\"level1_x_min\":" << cfg.level1_x_min << ",";
    stream << "\"level1_x_max\":" << cfg.level1_x_max << ",";
    stream << "\"active_level1_x_max\":" << get_active_level1_x_max() << ",";
    stream << "\"level2_x_min\":" << cfg.level2_x_min << ",";
    stream << "\"level2_x_max\":" << cfg.level2_x_max << ",";
    stream << "\"y_min\":" << cfg.y_min << ",";
    stream << "\"feeding_lane_y_min\":" << cfg.feeding_lane_y_min << ",";
    stream << "\"y_max\":" << cfg.y_max << ",";
    stream << "\"z_min\":" << cfg.z_min << ",";
    stream << "\"z_max\":" << cfg.z_max << ",";
    stream << "\"voxel_size\":" << cfg.voxel_size << ",";
    stream << "\"level1_voxel_threshold\":" << cfg.level1_threshold << ",";
    stream << "\"level2_voxel_threshold\":" << cfg.level2_threshold << ",";
    stream << "\"level1_delay\":" << cfg.level1_delay << ",";
    stream << "\"ignore_push_plate\":" << (cfg.ignore_push_plate ? "true" : "false") << ",";
    stream << "\"push_plate_x_min\":" << cfg.push_plate_x_min << ",";
    stream << "\"push_plate_x_max\":" << cfg.push_plate_x_max << ",";
    stream << "\"push_plate_y_min\":" << cfg.push_plate_y_min << ",";
    stream << "\"push_plate_y_max\":" << cfg.push_plate_y_max << ",";
    stream << "\"push_plate_z_min\":" << cfg.push_plate_z_min << ",";
    stream << "\"push_plate_z_max\":" << cfg.push_plate_z_max << ",";
    stream << "\"marker_lifetime\":" << cfg.marker_lifetime << ",";
    stream << "\"srv_cmd_code\":" << srv_cmd_code_;
    stream << "}";
    return stream.str();
  }

#ifdef HAVE_YAML_CPP
  static void apply_json_to_config(
      const YAML::Node& data,
      RuntimeConfig& cfg,
      std::set<std::string>& changed_keys)
  {
    if (data["enable"])
    {
      cfg.enable = data["enable"].as<bool>();
      changed_keys.insert("enable");
    }
    if (data["level1_x_min"])
    {
      cfg.level1_x_min = data["level1_x_min"].as<double>();
      changed_keys.insert("level1_x_min");
    }
    if (data["level1_x_max"])
    {
      cfg.level1_x_max = data["level1_x_max"].as<double>();
      changed_keys.insert("level1_x_max");
    }
    if (data["level2_x_min"])
    {
      cfg.level2_x_min = data["level2_x_min"].as<double>();
      changed_keys.insert("level2_x_min");
    }
    if (data["level2_x_max"])
    {
      cfg.level2_x_max = data["level2_x_max"].as<double>();
      changed_keys.insert("level2_x_max");
    }
    if (data["y_min"])
    {
      cfg.y_min = data["y_min"].as<double>();
      changed_keys.insert("y_min");
    }
    if (data["feeding_lane_y_min"])
    {
      cfg.feeding_lane_y_min = data["feeding_lane_y_min"].as<double>();
      changed_keys.insert("feeding_lane_y_min");
    }
    if (data["y_max"])
    {
      cfg.y_max = data["y_max"].as<double>();
      changed_keys.insert("y_max");
    }
    if (data["z_min"])
    {
      cfg.z_min = data["z_min"].as<double>();
      changed_keys.insert("z_min");
    }
    if (data["z_max"])
    {
      cfg.z_max = data["z_max"].as<double>();
      changed_keys.insert("z_max");
    }
    if (data["voxel_size"])
    {
      cfg.voxel_size = data["voxel_size"].as<double>();
      changed_keys.insert("voxel_size");
    }
    if (data["level1_voxel_threshold"])
    {
      cfg.level1_threshold = data["level1_voxel_threshold"].as<int>();
      changed_keys.insert("level1_voxel_threshold");
    }
    if (data["level2_voxel_threshold"])
    {
      cfg.level2_threshold = data["level2_voxel_threshold"].as<int>();
      changed_keys.insert("level2_voxel_threshold");
    }
    if (data["level1_delay"])
    {
      cfg.level1_delay = data["level1_delay"].as<double>();
      changed_keys.insert("level1_delay");
    }
    if (data["ignore_push_plate"])
    {
      cfg.ignore_push_plate = data["ignore_push_plate"].as<bool>();
      changed_keys.insert("ignore_push_plate");
    }
    if (data["push_plate_x_min"])
    {
      cfg.push_plate_x_min = data["push_plate_x_min"].as<double>();
      changed_keys.insert("push_plate_x_min");
    }
    if (data["push_plate_x_max"])
    {
      cfg.push_plate_x_max = data["push_plate_x_max"].as<double>();
      changed_keys.insert("push_plate_x_max");
    }
    if (data["push_plate_y_min"])
    {
      cfg.push_plate_y_min = data["push_plate_y_min"].as<double>();
      changed_keys.insert("push_plate_y_min");
    }
    if (data["push_plate_y_max"])
    {
      cfg.push_plate_y_max = data["push_plate_y_max"].as<double>();
      changed_keys.insert("push_plate_y_max");
    }
    if (data["push_plate_z_min"])
    {
      cfg.push_plate_z_min = data["push_plate_z_min"].as<double>();
      changed_keys.insert("push_plate_z_min");
    }
    if (data["push_plate_z_max"])
    {
      cfg.push_plate_z_max = data["push_plate_z_max"].as<double>();
      changed_keys.insert("push_plate_z_max");
    }
    if (data["marker_lifetime"])
    {
      cfg.marker_lifetime = data["marker_lifetime"].as<double>();
      changed_keys.insert("marker_lifetime");
    }
  }

#endif

  bool srv_cmd_callback(
      rrcmc_msgs::SrvCmd::Request& request,
      rrcmc_msgs::SrvCmd::Response& response)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);

    if (request.cmd_code != srv_cmd_code_)
    {
      response.is_success = false;
      response.resp_code = -1;
      response.resp_ii = "cmd_code mismatch";
      response.resp_json = build_runtime_config_json(get_runtime_config());

      ROS_WARN("Reject srv_cmd: request cmd_code=%d, expected=%d",
               request.cmd_code, srv_cmd_code_);
      return true;
    }

#ifndef HAVE_YAML_CPP
    response.is_success = false;
    response.resp_code = -2;
    response.resp_ii = "yaml-cpp is required to parse data_json";
    response.resp_json = build_runtime_config_json(get_runtime_config());
    return true;
#else
    try
    {
      RuntimeConfig old_cfg = get_runtime_config();
      RuntimeConfig new_cfg = old_cfg;
      const RrcmcIiRangeMode old_rrcmc_ii_range_mode = rrcmc_ii_range_mode_;
      const double old_active_x_max = get_active_level1_x_max();
      const std::string old_rrcmc_command = last_rrcmc_ii_command_;
      std::set<std::string> changed_keys;

      if (!request.data_json.empty())
      {
        YAML::Node data = YAML::Load(request.data_json);
        if (!data || !data.IsMap())
          throw std::runtime_error("data_json must be a JSON object");

        apply_json_to_config(data, new_cfg, changed_keys);
      }

      std::string validation_error;
      if (!validate_runtime_config(new_cfg, validation_error))
      {
        response.is_success = false;
        response.resp_code = -3;
        response.resp_ii = validation_error;
        response.resp_json = build_runtime_config_json(old_cfg);
        return true;
      }

      // level1_x_max defines the turning range. Updating its value must not
      // clear the current vehicle state selected by /rrcmc_ii.
      if (changed_keys.count("level1_x_max") > 0)
      {
        ROS_INFO(
            "level1_x_max updated by service %s: turning obstacle range "
            "changed to %.2f m; current /rrcmc_ii state remains unchanged",
            srv_name_.c_str(),
            new_cfg.level1_x_max);
      }

      set_runtime_config(new_cfg);
      write_runtime_config_to_rosparam();

      std::string yaml_error;
      if (!write_runtime_config_to_yaml(new_cfg, changed_keys, yaml_error))
      {
        set_runtime_config(old_cfg);
        rrcmc_ii_range_mode_ = old_rrcmc_ii_range_mode;
        last_rrcmc_ii_command_ = old_rrcmc_command;
        write_runtime_config_to_rosparam();

        response.is_success = false;
        response.resp_code = -4;
        response.resp_ii = "runtime update rolled back because YAML write failed: " + yaml_error;
        response.resp_json = build_runtime_config_json(old_cfg);
        ROS_ERROR("srv_cmd YAML write failed: %s", yaml_error.c_str());
        return true;
      }

      response.is_success = true;
      response.resp_code = 0;
      response.resp_ii = "96-line obstacle parameters updated; only changed YAML fields were saved";
      response.resp_json = build_runtime_config_json(new_cfg);

      ROS_INFO(
          "srv_cmd applied: cmd_code=%d, enable=%d, active_level1_x_max=%.2f",
          request.cmd_code,
          new_cfg.enable ? 1 : 0,
          get_active_level1_x_max());

      log_runtime_config_changes(
          old_cfg,
          get_runtime_config(),
          "WEB",
          old_active_x_max,
          get_active_level1_x_max(),
          old_rrcmc_command,
          last_rrcmc_ii_command_);
      return true;
    }
    catch (const std::exception& e)
    {
      response.is_success = false;
      response.resp_code = -5;
      response.resp_ii = e.what();
      response.resp_json = build_runtime_config_json(get_runtime_config());
      ROS_ERROR("srv_cmd processing failed: %s", e.what());
      return true;
    }
#endif
  }

  void publish_result(int status)
  {
    last_status_ = status;
    std_msgs::Int32 msg;
    msg.data = status;
    pub_.publish(msg);
  }

  void publish_status_msg()
  {
    std_msgs::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(last_status_),
      static_cast<float>(get_active_level1_x_max()),
      static_cast<float>(level2_x_max_),
      enable_ ? 1.0f : 0.0f,
      static_cast<float>(last_level1_count_),
      static_cast<float>(last_level2_count_),
      static_cast<float>(level1_threshold_),
      static_cast<float>(level2_threshold_),
      static_cast<float>(level1_x_min_),
      static_cast<float>(level2_x_min_),
      static_cast<float>(y_min_),
      static_cast<float>(y_max_),
      static_cast<float>(z_min_),
      static_cast<float>(z_max_),
      static_cast<float>(get_active_level1_delay()),
      delay_active_ ? 1.0f : 0.0f,
      static_cast<float>(voxel_size_),
      static_cast<float>(last_valid_voxel_count_),
      static_cast<float>(last_raw_points_count_),
      ignore_push_plate_ ? 1.0f : 0.0f,
      static_cast<float>(last_ignored_voxel_count_),
      static_cast<float>(push_plate_x_min_),
      static_cast<float>(push_plate_x_max_),
      static_cast<float>(push_plate_y_min_),
      static_cast<float>(push_plate_y_max_),
      static_cast<float>(push_plate_z_min_),
      static_cast<float>(push_plate_z_max_)
    };
    status_pub_.publish(msg);
  }

  visualization_msgs::Marker make_box_marker(
      const std_msgs::Header& header,
      int marker_id,
      const std::string& ns,
      double x_min, double x_max,
      double y_min, double y_max,
      double z_min, double z_max,
      float r, float g, float b, float a) const
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (x_min + x_max) / 2.0;
    marker.pose.position.y = (y_min + y_max) / 2.0;
    marker.pose.position.z = (z_min + z_max) / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::fabs(x_max - x_min);
    marker.scale.y = std::fabs(y_max - y_min);
    marker.scale.z = std::fabs(z_max - z_min);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = ros::Duration(marker_lifetime_);
    return marker;
  }

  void publish_roi_markers(const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(make_box_marker(
        header, 0, "level1_roi",
        level1_x_min_, get_active_level1_x_max(),
        get_active_y_min(), y_max_, z_min_, z_max_,
        1.0f, 0.0f, 0.0f, 0.15f));

    if (level2_x_max_ > level2_x_min_)
    {
      marker_array.markers.push_back(make_box_marker(
          header, 1, "level2_roi",
          level2_x_min_, level2_x_max_,
          get_active_y_min(), y_max_, z_min_, z_max_,
          1.0f, 0.8f, 0.0f, 0.15f));
    }
    else
    {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "level2_roi";
      marker.id = 1;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(marker);
    }

    roi_marker_pub_.publish(marker_array);
  }

  void publish_ignore_marker(const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray marker_array;

    if (ignore_push_plate_)
    {
      marker_array.markers.push_back(make_box_marker(
          header, 0, "push_plate_ignore_box",
          push_plate_x_min_, push_plate_x_max_,
          push_plate_y_min_, push_plate_y_max_,
          push_plate_z_min_, push_plate_z_max_,
          0.5f, 0.5f, 0.5f, 0.35f));
    }
    else
    {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "push_plate_ignore_box";
      marker.id = 0;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(marker);
    }

    ignore_marker_pub_.publish(marker_array);
  }

  void publish_voxel_cloud(
      const std_msgs::Header& header,
      const std::set<VoxelKey>& level1_voxels,
      const std::set<VoxelKey>& level2_voxels)
  {
    std::vector<sensor_msgs::PointField> fields(4);
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    fields[2].count = 1;
    fields[3].name = "rgb";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    fields[3].count = 1;

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = header;
    cloud_msg.height = 1;
    cloud_msg.width = level1_voxels.size() + level2_voxels.size();
    cloud_msg.is_dense = false;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.fields = fields;
    cloud_msg.data.resize(cloud_msg.row_step);

    const float level1_rgb = rgb_to_float(255, 0, 0);
    const float level2_rgb = rgb_to_float(255, 200, 0);
    std::size_t offset = 0;

    const std::set<VoxelKey>* groups[2] = {&level1_voxels, &level2_voxels};
    const float colors[2] = {level1_rgb, level2_rgb};

    for (int group = 0; group < 2; ++group)
    {
      for (std::set<VoxelKey>::const_iterator it = groups[group]->begin();
           it != groups[group]->end(); ++it)
      {
        double x, y, z;
        voxel_key_to_center(*it, x, y, z);
        const float values[4] = {
          static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(z),
          colors[group]
        };

        for (int index = 0; index < 4; ++index)
        {
          std::memcpy(&cloud_msg.data[offset], &values[index], sizeof(float));
          offset += sizeof(float);
        }
      }
    }

    voxel_cloud_pub_.publish(cloud_msg);
  }

  bool is_in_level1_region(double x) const
  {
    return x >= level1_x_min_ && x < get_active_level1_x_max();
  }

  bool is_in_level2_region(double x) const
  {
    return level2_x_max_ > level2_x_min_ &&
           x >= level2_x_min_ && x <= level2_x_max_;
  }

  bool is_in_push_plate_ignore_box(double x, double y, double z) const
  {
    if (!ignore_push_plate_)
      return false;

    return x >= push_plate_x_min_ && x <= push_plate_x_max_ &&
           y >= push_plate_y_min_ && y <= push_plate_y_max_ &&
           z >= push_plate_z_min_ && z <= push_plate_z_max_;
  }

  bool is_in_level1_delay(const ros::Time& now) const
  {
    const double active_delay = get_active_level1_delay();
    if (last_level1_time_.isZero() || active_delay <= 0.0)
      return false;
    return (now - last_level1_time_).toSec() < active_delay;
  }

  int compute_status(int level1_count, int level2_count)
  {
    const ros::Time now = ros::Time::now();
    delay_active_ = false;

    if (!enable_)
    {
      last_level1_time_ = ros::Time(0);
      return 0;
    }

    if (level1_count >= level1_threshold_)
    {
      last_level1_time_ = now;
      return 1;
    }

    if (is_in_level1_delay(now))
    {
      delay_active_ = true;
      return 1;
    }

    if (level2_x_max_ > level2_x_min_ &&
        level2_count >= level2_threshold_)
    {
      return 2;
    }

    return 0;
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);

    const std::chrono::system_clock::time_point wall_time =
        std::chrono::system_clock::now();

    const bool recovered_from_timeout = input_timeout_active_;
    const std::chrono::system_clock::time_point previous_cloud_time =
        has_received_cloud_ ? last_cloud_wall_time_ : node_start_wall_time_;
    const double recovery_gap_ms =
        elapsed_wall_seconds(wall_time, previous_cloud_time) * 1000.0;

    has_received_cloud_ = true;
    last_cloud_wall_time_ = wall_time;
    input_timeout_active_ = false;

    const int previous_status = last_status_;
    const bool previous_delay = delay_active_;

    std::set<VoxelKey> level1_voxels;
    std::set<VoxelKey> level2_voxels;
    std::set<VoxelKey> ignored_voxels;
    int raw_points_count = 0;

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        ++raw_points_count;

        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);

        if (!is_valid_number(x) || !is_valid_number(y) || !is_valid_number(z))
          continue;
        if (y < get_active_y_min() || y > y_max_)
          continue;
        if (z < z_min_ || z > z_max_)
          continue;

        const VoxelKey key = make_voxel_key(x, y, z);

        if (is_in_push_plate_ignore_box(x, y, z))
        {
          ignored_voxels.insert(key);
          continue;
        }

        if (is_in_level1_region(x))
          level1_voxels.insert(key);
        else if (is_in_level2_region(x))
          level2_voxels.insert(key);
      }

      const int level1_count = static_cast<int>(level1_voxels.size());
      const int level2_count = static_cast<int>(level2_voxels.size());
      const int ignored_count = static_cast<int>(ignored_voxels.size());
      const int status = compute_status(level1_count, level2_count);

      last_level1_count_ = level1_count;
      last_level2_count_ = level2_count;
      last_valid_voxel_count_ = level1_count + level2_count;
      last_raw_points_count_ = raw_points_count;
      last_ignored_voxel_count_ = ignored_count;

      publish_result(status);

      const std_msgs::Header header = make_header(msg->header);
      publish_roi_markers(header);
      publish_ignore_marker(header);
      publish_voxel_cloud(header, level1_voxels, level2_voxels);
      publish_status_msg();

      if (recovered_from_timeout)
      {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(0);
        stream << "[INPUT_RECOVERED] time=" << format_wall_time(wall_time)
               << " gap_ms=" << recovery_gap_ms
               << " status=" << status
               << " L1=" << level1_count
               << " L2=" << level2_count
               << " rrcmc_ii=\"" << last_rrcmc_ii_command_ << "\""
               << std::setprecision(2)
               << " X=[" << level1_x_min_ << ","
               << get_active_level1_x_max() << "]";
        write_log_line_at(stream.str(), wall_time, true);
        last_result_log_wall_time_ = wall_time;
      }

      if (status != previous_status)
      {
        log_state_change(
            wall_time,
            previous_status,
            status,
            level1_count,
            level2_count);
      }
      else if (delay_active_ != previous_delay)
      {
        log_delay_change(
            wall_time,
            previous_delay,
            delay_active_,
            status,
            level1_count,
            level2_count);
      }

      log_periodic_result(
          wall_time,
          status,
          level1_count,
          level2_count);

      ROS_INFO_THROTTLE(
          0.5,
          "level1_count=%d, level2_count=%d, ignored_count=%d, status=%d, "
          "enable=%d, delay=%d, rrcmc_ii='%s', X=[%.2f, %.2f], "
          "feeding_lane=%d, Y=[%.2f, %.2f], yaml_y_min=%.2f",
          level1_count,
          level2_count,
          ignored_count,
          status,
          enable_ ? 1 : 0,
          delay_active_ ? 1 : 0,
          last_rrcmc_ii_command_.c_str(),
          level1_x_min_,
          get_active_level1_x_max(),
          feeding_lane_ ? 1 : 0,
          get_active_y_min(),
          y_max_,
          y_min_);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("PointCloud process failed: %s", e.what());

      std::ostringstream stream;
      stream << "[PROCESS_ERROR] time=" << format_wall_time(wall_time)
             << " error=\"" << e.what() << "\"";
      write_log_line_at(stream.str(), wall_time, true);
    }
  }

  void status_timer_callback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    publish_status_msg();
    log_maintenance();
  }

  void reload_timer_callback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    reload_config_if_needed();
  }

  void rosparam_timer_callback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    read_runtime_config_from_rosparam();
  }

  void log_startup_info() const
  {
    ROS_INFO("points_obstacle_m_96_detector started");
    ROS_INFO("Config path          : %s", config_path_.c_str());
    ROS_INFO("Enable               : %s", enable_ ? "true" : "false");
    ROS_INFO("Reload YAML          : %s", reload_yaml_ ? "true" : "false");
    ROS_INFO("Reload interval      : %.2f s", reload_interval_);
    ROS_INFO("Sync rosparam        : %s", sync_rosparam_ ? "true" : "false");
    ROS_INFO("Rosparam interval    : %.2f s", rosparam_sync_interval_);
    ROS_INFO("Service name         : %s", srv_name_.c_str());
    ROS_INFO("Service cmd_code     : %d", srv_cmd_code_);
    ROS_INFO("Subscribe pointcloud : %s", input_topic_.c_str());
    ROS_INFO("Subscribe dynamic ROI: /rrcmc_ii (std_msgs/Header.frame_id)");
    ROS_INFO("Subscribe feeding lane: /rrcmc_status field0[73]");
    ROS_INFO("Publish result       : %s", output_topic_.c_str());
    ROS_INFO("Publish status       : %s", status_topic_.c_str());
    ROS_INFO("Publish ROI markers  : %s", roi_marker_topic_.c_str());
    ROS_INFO("Publish voxel cloud  : %s", voxel_cloud_topic_.c_str());
    ROS_INFO("Publish ignore marker: %s", ignore_marker_topic_.c_str());
    ROS_INFO("Level1 X (turning)   : %.2f ~ %.2f m", level1_x_min_, level1_x_max_);
    ROS_INFO("Level1 X (normal)    : %.2f ~ %.2f m",
             level1_x_min_, level1_normal_x_max_);
    ROS_INFO("Level1 X (active)    : %.2f ~ %.2f m",
             level1_x_min_, get_active_level1_x_max());
    ROS_INFO("/rrcmc_ii mapping    : 81003;999 -> turning %.2f m, "
             "81003;0/not received -> normal %.2f m, unsupported -> keep previous",
             level1_x_max_, level1_normal_x_max_);
    ROS_INFO("Level2 X             : %.2f ~ %.2f m", level2_x_min_, level2_x_max_);
    ROS_INFO("Detect Y (YAML/out)  : %.2f ~ %.2f m", y_min_, y_max_);
    ROS_INFO("Detect Y (feeding)   : %.2f ~ %.2f m", feeding_lane_y_min_, y_max_);
    ROS_INFO("Detect Z             : %.2f ~ %.2f m", z_min_, z_max_);
    ROS_INFO("Voxel size           : %.3f m", voxel_size_);
    ROS_INFO("Threshold            : level1=%d, level2=%d", level1_threshold_, level2_threshold_);
    ROS_INFO("Level1 delay turning : %.2f s", level1_delay_);
    ROS_INFO("Level1 delay normal  : %.2f s", level1_normal_delay_);
    ROS_INFO("Level1 delay active  : %.2f s", get_active_level1_delay());
    ROS_INFO("Ignore push plate    : %s", ignore_push_plate_ ? "true" : "false");
    ROS_INFO("Decision log enable  : %s", log_enable_ ? "true" : "false");
    ROS_INFO("Decision log dir     : %s", log_dir_.c_str());
    ROS_INFO("Decision log period  : %.2f s", log_result_period_sec_);
    ROS_INFO("Decision log segment : %d min", log_segment_minutes_);
    ROS_INFO("Decision log retain  : %.1f h", log_retention_hours_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_obstacle_m_96_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PointsObstacleM96Detector detector(nh, pnh);
  ros::spin();
  return 0;
}
