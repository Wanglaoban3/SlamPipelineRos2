// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port of scan_matching_odometry_nodelet.cpp (standalone node, tf2)
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_graph_slam/param_helper.hpp>
#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam_ros2/msg/scan_matching_status.hpp>

namespace hdl_graph_slam {

class ScanMatchingOdometryNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNode() : Node("scan_matching_odometry_node") {
    initialize_params();

    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points",
      rclcpp::QoS(256).best_effort(),
      std::bind(&ScanMatchingOdometryNode::cloud_callback, this, std::placeholders::_1));
    odom_pub = create_publisher<nav_msgs::msg::Odometry>(published_odom_topic, 32);
    trans_pub = create_publisher<geometry_msgs::msg::TransformStamped>("/scan_matching_odometry/transform", 32);
    status_pub = create_publisher<hdl_graph_slam_ros2::msg::ScanMatchingStatus>("/scan_matching_odometry/status", 8);
    aligned_points_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 32);
  }

  // must be called after the node is owned by a shared_ptr
  void init() {
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    published_odom_topic = param_or<std::string>(this, "published_odom_topic", "/odom");
    odom_frame_id = param_or<std::string>(this, "odom_frame_id", "odom");
    robot_odom_frame_id = param_or<std::string>(this, "robot_odom_frame_id", "robot_odom");

    // The minimum translational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = param_or<double>(this, "keyframe_delta_trans", 0.25);
    keyframe_delta_angle = param_or<double>(this, "keyframe_delta_angle", 0.15);
    keyframe_delta_time = param_or<double>(this, "keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    transform_thresholding = param_or<bool>(this, "transform_thresholding", false);
    max_acceptable_trans = param_or<double>(this, "max_acceptable_trans", 1.0);
    max_acceptable_angle = param_or<double>(this, "max_acceptable_angle", 1.0);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = param_or<std::string>(this, "downsample_method", "VOXELGRID");
    double downsample_resolution = param_or<double>(this, "downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    ParamNodeHandle pnh(this);
    registration = select_registration_method(pnh);
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
    if(!rclcpp::ok()) {
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Matrix4f pose = matching(stamp, cloud);
    publish_odometry(stamp, cloud_msg->header.frame_id, pose);
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4f matching(const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    registration->setInputSource(filtered);

    std::string msf_source;
    Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

    if(param_or<bool>(this, "enable_robot_odometry_init_guess", false) && prev_time.nanoseconds() != 0) {
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer.lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, tf2::durationFromSec(0.0));
      } catch(tf2::TransformException& e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id << ": " << e.what());
      }

      if(transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0) {
        msf_source = "odometry";
        msf_delta = transform2isometry(transform).cast<float>();
      }
    }

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, prev_trans * msf_delta.matrix());

    publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    if(!registration->hasConverged()) {
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "scan matching has not converged!!");
      return keyframe_pose * prev_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;

    if(transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

      if(dx > max_acceptable_trans || da > max_acceptable_angle) {
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "too large transform!! " << dx << "[m] " << da << "[rad]");
        return keyframe_pose * prev_trans;
      }
    }

    prev_time = stamp;
    prev_trans = trans;

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).seconds();
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_time = stamp;
      prev_trans.setIdentity();
    }

    if(aligned_points_pub->get_subscription_count() > 0) {
      pcl::PointCloud<PointT>::Ptr aligned_out(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*cloud, *aligned_out, odom);
      aligned_out->header.frame_id = odom_frame_id;
      aligned_out->header.stamp = cloud->header.stamp;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned_out, aligned_msg);
      aligned_points_pub->publish(aligned_msg);
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const rclcpp::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // publish transform stamped for IMU integration
    geometry_msgs::msg::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    trans_pub->publish(odom_trans);

    // broadcast the transform over tf
    odom_broadcaster->sendTransform(odom_trans);

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub->publish(odom);
  }

  /**
   * @brief publish scan matching status
   */
  void publish_scan_matching_status(const rclcpp::Time& stamp, const std::string& frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source, const Eigen::Isometry3f& msf_delta) {
    if(!status_pub->get_subscription_count()) {
      return;
    }

    hdl_graph_slam_ros2::msg::ScanMatchingStatus status;
    status.header.frame_id = frame_id;
    status.header.stamp = stamp;
    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(size_t i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    status.relative_pose = isometry2pose(Eigen::Isometry3f(registration->getFinalTransformation()).cast<double>());

    if(!msf_source.empty()) {
      status.prediction_labels.resize(1);
      status.prediction_labels[0].data = msf_source;

      status.prediction_errors.resize(1);
      Eigen::Isometry3f error = Eigen::Isometry3f(registration->getFinalTransformation()).inverse() * msf_delta;
      status.prediction_errors[0] = isometry2pose(error.cast<double>());
    }

    status_pub->publish(status);
  }

private:
  // ROS topics
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr trans_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_pub;
  rclcpp::Publisher<hdl_graph_slam_ros2::msg::ScanMatchingStatus>::SharedPtr status_pub;

  tf2_ros::Buffer tf_buffer{get_clock()};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  std::string published_odom_topic;
  std::string odom_frame_id = "odom";
  std::string robot_odom_frame_id = "robot_odom";

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  rclcpp::Time keyframe_stamp{0, 0, RCL_ROS_TIME};  // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace hdl_graph_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hdl_graph_slam::ScanMatchingOdometryNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
