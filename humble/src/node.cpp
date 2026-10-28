#include "configuration.h"
#include "message_conversion.h"
#include <fast_livo2/mapper.h>
#include <cv_bridge/cv_bridge.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

class FastLivoNode : public rclcpp::Node {
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using OdomMsg = nav_msgs::msg::Odometry;
public:
  FastLivoNode() : Node("laserMapping"), config_(fast_livo2::adapter::readConfig(
      [this](const char *key, auto &value) { parameter(key, value); },
      [this](const char *key, auto &value) { parameter(key, value); })),
      mapper_(config_.core, [this](const fast_livo2::Odometry &state) { publishOdometry(state, true); }),
      broadcaster_(this) {
    world_ = create_publisher<CloudMsg>("/cloud_registered", 10);
    body_ = create_publisher<CloudMsg>("/n3m_input/cloud_body", 10);
    effect_ = create_publisher<CloudMsg>("/cloud_effected", 10);
    odom_ = create_publisher<OdomMsg>("/n3m_input/odometry", 10);
    legacy_odom_ = create_publisher<OdomMsg>("/aft_mapped_to_init", 10);
    imu_odom_ = create_publisher<OdomMsg>("/LIVO2/imu_propagate", 20);
    vision_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
    path_ = create_publisher<nav_msgs::msg::Path>("/path", 2);
    image_ = create_publisher<sensor_msgs::msg::Image>("/rgb_img", 2);
    planes_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 2);
    if (config_.core.lidar_type == 1) {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(config_.lid_topic, rclcpp::SensorDataQoS().keep_last(20),
          [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg) {
            if (fast_livo2::adapter::livoxScan(*msg, scan_)) mapper_.pushLivox(rclcpp::Time(msg->header.stamp).nanoseconds(), scan_);
            else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Livox point_num does not match points.size");
            drain();
          });
    } else {
      cloud_sub_ = create_subscription<CloudMsg>(config_.lid_topic, rclcpp::SensorDataQoS().keep_last(20), [this](const CloudMsg::ConstSharedPtr &msg) {
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(*msg, cloud);
        mapper_.pushPointCloud(rclcpp::Time(msg->header.stamp).nanoseconds(), cloud);
        drain();
      });
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(config_.imu_topic, rclcpp::SensorDataQoS().keep_last(2000),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
          mapper_.pushImu(fast_livo2::adapter::imuSample(*msg, rclcpp::Time(msg->header.stamp).nanoseconds())); drain();
        });
    if (config_.core.img_en)
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(config_.img_topic, rclcpp::SensorDataQoS().keep_last(20),
          [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            mapper_.pushImage(rclcpp::Time(msg->header.stamp).nanoseconds(), cv_bridge::toCvCopy(msg, "bgr8")->image); drain();
          });
  }
  void finish() { mapper_.finish(); }
private:
  template<class T> void parameter(const char *key, T &value) {
    if constexpr (std::is_same_v<T, std::vector<int>>) {
      const auto result = declare_parameter<std::vector<int64_t>>(key, {value.begin(), value.end()});
      value.assign(result.begin(), result.end());
    } else if constexpr (std::is_same_v<T, double>) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      const auto result = declare_parameter(key, rclcpp::ParameterValue(value), descriptor);
      value = result.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                  ? static_cast<double>(result.get<int64_t>()) : result.get<double>();
    } else value = declare_parameter<T>(key, value);
  }
  void publishOdometry(const fast_livo2::Odometry &state, bool imu) {
    OdomMsg msg;
    msg.header.stamp = rclcpp::Time(state.stamp_ns);
    msg.header.frame_id = config_.world_frame;
    msg.child_frame_id = config_.body_frame;
    fast_livo2::adapter::odometry(state, msg);
    if (imu) { imu_odom_->publish(msg); return; }
    odom_->publish(msg); legacy_odom_->publish(msg);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = msg.child_frame_id;
    transform.transform.translation.x = state.position.x();
    transform.transform.translation.y = state.position.y();
    transform.transform.translation.z = state.position.z();
    transform.transform.rotation = msg.pose.pose.orientation;
    broadcaster_.sendTransform(transform);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header; pose.pose = msg.pose.pose;
    vision_pose_->publish(pose);
  }
  void drain() {
    for (;;) {
      auto out = mapper_.process();
      if (!out.diagnostic.empty()) RCLCPP_INFO(get_logger(), "%s", out.diagnostic.c_str());
      if (!out.consumed()) return;
      if (out.status == fast_livo2::FrameStatus::Mapped) publish(out);
    }
  }
  void publish(const fast_livo2::FrameOutput &out) {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(out.odometry.stamp_ns);
    header.frame_id = config_.world_frame;
    CloudMsg body;
    pcl::toROSMsg(*out.body_cloud, body);
    body.header = header; body.header.frame_id = config_.body_frame;
    body_->publish(body);
    publishOdometry(out.odometry, false);
    if (out.append_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header; fast_livo2::adapter::pose(out.odometry, pose.pose);
      path_message_.header = header;
      path_message_.poses.push_back(pose);
      if (path_message_.poses.size() > 10000) path_message_.poses.erase(path_message_.poses.begin());
      path_->publish(path_message_);
    }
    if (out.world_cloud || out.color_cloud) {
      CloudMsg msg;
      if (out.color_cloud) pcl::toROSMsg(*out.color_cloud, msg);
      else pcl::toROSMsg(*out.world_cloud, msg);
      msg.header = header; world_->publish(msg);
    }
    if (out.effect_cloud) {
      CloudMsg msg; pcl::toROSMsg(*out.effect_cloud, msg);
      msg.header = header; effect_->publish(msg);
    }
    if (!out.image.empty()) image_->publish(*cv_bridge::CvImage(header, "bgr8", out.image).toImageMsg());
    if (!out.planes.empty()) {
      visualization_msgs::msg::MarkerArray markers;
      markers.markers.reserve(out.planes.size());
      for (const auto &plane : out.planes) {
        visualization_msgs::msg::Marker marker;
        marker.header = header; marker.ns = "plane"; marker.id = plane.id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER; marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = plane.position.x(); marker.pose.position.y = plane.position.y(); marker.pose.position.z = plane.position.z();
        marker.pose.orientation.x = plane.orientation.x(); marker.pose.orientation.y = plane.orientation.y();
        marker.pose.orientation.z = plane.orientation.z(); marker.pose.orientation.w = plane.orientation.w();
        marker.scale.x = plane.scale.x(); marker.scale.y = plane.scale.y(); marker.scale.z = plane.scale.z();
        marker.color.r = plane.color.x(); marker.color.g = plane.color.y(); marker.color.b = plane.color.z(); marker.color.a = plane.alpha;
        markers.markers.push_back(marker);
      }
      planes_->publish(markers);
    }
  }
  fast_livo2::adapter::NodeConfig config_;
  fast_livo2::Mapper mapper_;
  fast_livo2::LivoxScan scan_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::msg::Path path_message_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr world_, body_, effect_;
  rclcpp::Publisher<OdomMsg>::SharedPtr odom_, legacy_odom_, imu_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<FastLivoNode>();
    rclcpp::spin(node); node->finish();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("laserMapping"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
