// /scan for slam_toolbox / EasyNav from the last second of FAST-LIO clouds.
// C++ port of scripts/lio_scan.py (same parameters, same algorithm, same /scan up to float
// rounding); see that file for the reasoning. Launch with go2w.launch.py lio_scan_cpp:=true.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

Eigen::Matrix4d matrix(double x, double y, double z, double w, double tx, double ty, double tz)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), tx,
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), ty,
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), tz,
    0, 0, 0, 1;
  return m;
}

int64_t nanoseconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

class LioScan : public rclcpp::Node
{
public:
  LioScan()
  : Node("lio_scan")
  {
    window_ = declare_parameter("window", 1.0);
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    lidar_frame_ = declare_parameter("lidar_frame", std::string("utlidar_lidar"));
    min_height_ = declare_parameter("min_height", -0.35);
    max_height_ = declare_parameter("max_height", 1.0);
    self_box_ = declare_parameter("self_box", std::vector<double>{-0.55, 0.45, -0.35, 0.35});
    angle_increment_ = declare_parameter("angle_increment", 0.0087);
    range_min_ = declare_parameter("range_min", 0.3);
    range_max_ = declare_parameter("range_max", 15.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    n_beams_ = static_cast<int>(std::ceil(2.0 * M_PI / angle_increment_));

    // reliable: EasyNav subscribes to LaserScan as reliable (no param), best effort never reaches it
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS().reliable());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odometry(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", rclcpp::QoS(20),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_cloud(*msg);});
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    poses_.emplace_back(nanoseconds(msg.header.stamp), matrix(q.x, q.y, q.z, q.w, p.x, p.y, p.z));
    if (poses_.size() > 50) {
      poses_.pop_front();
    }
  }

  static std::vector<Eigen::Vector3d> xyz(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : cloud.fields) {
      if (f.name == "x") {ox = f.offset;}
      if (f.name == "y") {oy = f.offset;}
      if (f.name == "z") {oz = f.offset;}
    }
    std::vector<Eigen::Vector3d> out;
    if (ox < 0 || oy < 0 || oz < 0) {
      return out;
    }
    const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
    out.reserve(n);
    float v[3];
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * pt = cloud.data.data() + i * cloud.point_step;
      std::memcpy(&v[0], pt + ox, 4);
      std::memcpy(&v[1], pt + oy, 4);
      std::memcpy(&v[2], pt + oz, 4);
      if (std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2])) {
        out.emplace_back(v[0], v[1], v[2]);
      }
    }
    return out;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!have_base_to_lidar_) {
      try {
        auto t = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
        const auto & r = t.transform.rotation;
        const auto & tr = t.transform.translation;
        base_to_lidar_ = matrix(r.x, r.y, r.z, r.w, tr.x, tr.y, tr.z);
        have_base_to_lidar_ = true;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for TF %s -> %s: %s",
          base_frame_.c_str(), lidar_frame_.c_str(), e.what());
        return;
      }
    }

    // FAST-LIO publishes /Odometry before the cloud, both stamped with the scan end
    const int64_t stamp = nanoseconds(msg.header.stamp);
    const Eigen::Matrix4d * pose = nullptr;
    for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
      if (it->first == stamp) {
        pose = &it->second;
        break;
      }
    }
    if (pose == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No /Odometry with the stamp of the cloud");
      return;
    }

    // camera_init -> L1 now -> base_link
    const Eigen::Matrix4d world_to_base = base_to_lidar_ * pose->inverse();
    const Eigen::Matrix3d rot = world_to_base.topLeftCorner<3, 3>();
    const Eigen::Vector3d tr = world_to_base.topRightCorner<3, 1>();
    const double x0 = self_box_[0], x1 = self_box_[1], y0 = self_box_[2], y1 = self_box_[3];

    // The robot's own points are dropped from each cloud at its own pose, before it is kept
    // (see lio_scan.py): otherwise the body of the older clouds trails behind the moving robot.
    std::vector<Eigen::Vector3d> cloud = xyz(msg);
    cloud.erase(std::remove_if(cloud.begin(), cloud.end(),
      [&](const Eigen::Vector3d & pw) {
        const Eigen::Vector3d p = rot * pw + tr;
        return p.x() > x0 && p.x() < x1 && p.y() > y0 && p.y() < y1;
      }), cloud.end());
    clouds_.emplace_back(stamp, std::move(cloud));
    while (!clouds_.empty() && stamp - clouds_.front().first > window_ * 1e9) {
      clouds_.pop_front();
    }

    std::vector<float> ranges(n_beams_, std::numeric_limits<float>::infinity());
    for (const auto & c : clouds_) {
      for (const auto & pw : c.second) {
        const Eigen::Vector3d p = rot * pw + tr;
        if (!(p.z() > min_height_ && p.z() < max_height_)) {continue;}
        if (p.x() > x0 && p.x() < x1 && p.y() > y0 && p.y() < y1) {continue;}
        const double r = std::hypot(p.x(), p.y());
        if (r < range_min_ || r > range_max_) {continue;}
        int beam = static_cast<int>((std::atan2(p.y(), p.x()) + M_PI) / angle_increment_);
        beam = std::min(std::max(beam, 0), n_beams_ - 1);
        ranges[beam] = std::min(ranges[beam], static_cast<float>(r));
      }
    }

    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = msg.header.stamp;
    scan->header.frame_id = base_frame_;
    scan->angle_min = -M_PI;
    scan->angle_max = -M_PI + (n_beams_ - 1) * angle_increment_;
    scan->angle_increment = angle_increment_;
    scan->scan_time = 0.0;
    scan->range_min = range_min_;
    scan->range_max = range_max_;
    scan->ranges = std::move(ranges);
    scan_pub_->publish(std::move(scan));
  }

  double window_, min_height_, max_height_, angle_increment_, range_min_, range_max_;
  std::string base_frame_, lidar_frame_;
  std::vector<double> self_box_;
  int n_beams_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix4d base_to_lidar_;
  bool have_base_to_lidar_ = false;
  std::deque<std::pair<int64_t, Eigen::Matrix4d>> poses_;
  std::deque<std::pair<int64_t, std::vector<Eigen::Vector3d>>> clouds_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioScan>());
  rclcpp::shutdown();
  return 0;
}
