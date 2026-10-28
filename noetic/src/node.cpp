#include "configuration.h"
#include "message_conversion.h"
#include <fast_livo2/mapper.h>
#include <cv_bridge/cv_bridge.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

class FastLivoNode {
  using CloudMsg = sensor_msgs::PointCloud2;
  using OdomMsg = nav_msgs::Odometry;
public:
  FastLivoNode() : private_("~"), config_(fast_livo2::adapter::readConfig(
      [this](const char *name, auto &value) {
        std::string key = name;
        std::replace(key.begin(), key.end(), '.', '/');
        if (!private_.getParam(key, value)) node_.param(key, value, value);
      }, [this](const char *key, auto &value) { private_.param(key, value, value); })),
      mapper_(config_.core, [this](const fast_livo2::Odometry &state) { publishOdometry(state, true); }) {
    world_ = node_.advertise<CloudMsg>("/cloud_registered", 10);
    body_ = node_.advertise<CloudMsg>("/n3m_input/cloud_body", 10);
    effect_ = node_.advertise<CloudMsg>("/cloud_effected", 10);
    odom_ = node_.advertise<OdomMsg>("/n3m_input/odometry", 10);
    legacy_odom_ = node_.advertise<OdomMsg>("/aft_mapped_to_init", 10);
    imu_odom_ = node_.advertise<OdomMsg>("/LIVO2/imu_propagate", 20);
    vision_pose_ = node_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    path_ = node_.advertise<nav_msgs::Path>("/path", 2);
    image_ = node_.advertise<sensor_msgs::Image>("/rgb_img", 2);
    planes_ = node_.advertise<visualization_msgs::MarkerArray>("/planes", 2);
    if (config_.core.lidar_type == 1) {
      cloud_sub_ = node_.subscribe<livox_ros_driver::CustomMsg>(config_.lid_topic, 20,
          [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) {
            if (fast_livo2::adapter::livoxScan(*msg, scan_)) mapper_.pushLivox(msg->header.stamp.toNSec(), scan_);
            else ROS_WARN_THROTTLE(1., "Livox point_num does not match points.size");
            drain();
          });
    } else {
      cloud_sub_ = node_.subscribe<CloudMsg>(config_.lid_topic, 20, [this](const CloudMsg::ConstPtr &msg) {
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(*msg, cloud);
        mapper_.pushPointCloud(msg->header.stamp.toNSec(), cloud);
        drain();
      });
    }
    imu_sub_ = node_.subscribe<sensor_msgs::Imu>(config_.imu_topic, 2000,
        [this](const sensor_msgs::Imu::ConstPtr &msg) {
          mapper_.pushImu(fast_livo2::adapter::imuSample(*msg, msg->header.stamp.toNSec())); drain();
        });
    if (config_.core.img_en)
      image_sub_ = node_.subscribe<sensor_msgs::Image>(config_.img_topic, 20,
          [this](const sensor_msgs::Image::ConstPtr &msg) {
            mapper_.pushImage(msg->header.stamp.toNSec(), cv_bridge::toCvCopy(msg, "bgr8")->image); drain();
          });
  }
  void finish() { mapper_.finish(); }
private:
  void publishOdometry(const fast_livo2::Odometry &state, bool imu) {
    OdomMsg msg;
    msg.header.stamp.fromNSec(state.stamp_ns);
    msg.header.frame_id = config_.world_frame;
    msg.child_frame_id = config_.body_frame;
    fast_livo2::adapter::odometry(state, msg);
    if (imu) { imu_odom_.publish(msg); return; }
    odom_.publish(msg); legacy_odom_.publish(msg);
    geometry_msgs::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = msg.child_frame_id;
    transform.transform.translation.x = state.position.x();
    transform.transform.translation.y = state.position.y();
    transform.transform.translation.z = state.position.z();
    transform.transform.rotation = msg.pose.pose.orientation;
    broadcaster_.sendTransform(transform);
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header; pose.pose = msg.pose.pose;
    vision_pose_.publish(pose);
  }
  void drain() {
    for (;;) {
      auto out = mapper_.process();
      if (!out.diagnostic.empty()) ROS_INFO("%s", out.diagnostic.c_str());
      if (!out.consumed()) return;
      if (out.status == fast_livo2::FrameStatus::Mapped) publish(out);
    }
  }
  void publish(const fast_livo2::FrameOutput &out) {
    std_msgs::Header header;
    header.stamp.fromNSec(out.odometry.stamp_ns);
    header.frame_id = config_.world_frame;
    CloudMsg body;
    pcl::toROSMsg(*out.body_cloud, body);
    body.header = header; body.header.frame_id = config_.body_frame;
    body_.publish(body);
    publishOdometry(out.odometry, false);
    if (out.append_path) {
      geometry_msgs::PoseStamped pose;
      pose.header = header; fast_livo2::adapter::pose(out.odometry, pose.pose);
      path_message_.header = header;
      path_message_.poses.push_back(pose);
      if (path_message_.poses.size() > 10000) path_message_.poses.erase(path_message_.poses.begin());
      path_.publish(path_message_);
    }
    if (out.world_cloud || out.color_cloud) {
      CloudMsg msg;
      if (out.color_cloud) pcl::toROSMsg(*out.color_cloud, msg);
      else pcl::toROSMsg(*out.world_cloud, msg);
      msg.header = header; world_.publish(msg);
    }
    if (out.effect_cloud) {
      CloudMsg msg; pcl::toROSMsg(*out.effect_cloud, msg);
      msg.header = header; effect_.publish(msg);
    }
    if (!out.image.empty()) image_.publish(cv_bridge::CvImage(header, "bgr8", out.image).toImageMsg());
    if (!out.planes.empty()) {
      visualization_msgs::MarkerArray markers;
      markers.markers.reserve(out.planes.size());
      for (const auto &plane : out.planes) {
        visualization_msgs::Marker marker;
        marker.header = header; marker.ns = "plane"; marker.id = plane.id;
        marker.type = visualization_msgs::Marker::CYLINDER; marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = plane.position.x(); marker.pose.position.y = plane.position.y(); marker.pose.position.z = plane.position.z();
        marker.pose.orientation.x = plane.orientation.x(); marker.pose.orientation.y = plane.orientation.y();
        marker.pose.orientation.z = plane.orientation.z(); marker.pose.orientation.w = plane.orientation.w();
        marker.scale.x = plane.scale.x(); marker.scale.y = plane.scale.y(); marker.scale.z = plane.scale.z();
        marker.color.r = plane.color.x(); marker.color.g = plane.color.y(); marker.color.b = plane.color.z(); marker.color.a = plane.alpha;
        markers.markers.push_back(marker);
      }
      planes_.publish(markers);
    }
  }
  ros::NodeHandle node_, private_;
  fast_livo2::adapter::NodeConfig config_;
  fast_livo2::Mapper mapper_;
  fast_livo2::LivoxScan scan_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::Path path_message_;
  ros::Subscriber cloud_sub_, imu_sub_, image_sub_;
  ros::Publisher world_, body_, effect_, odom_, legacy_odom_, imu_odom_, vision_pose_, path_, image_, planes_;
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  try {
    FastLivoNode node;
    ros::spin(); node.finish();
  } catch (const std::exception &error) {
    ROS_FATAL("%s", error.what()); return 1;
  }
  return 0;
}
