// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port of hdl_localization_nodelet.cpp (standalone node)
// - hdl_global_localization services dropped (global localization via initialpose / init pose params)
// - global map loaded from globalmap_filename param directly
#include <mutex>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization_ros2/msg/scan_matching_status.hpp>
#include <hdl_localization/param_helper.hpp>

namespace hdl_localization {

class HdlLocalizationNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNode() : Node("hdl_localization_node") {
    initialize_params();

    robot_odom_frame_id = param_or<std::string>(this, "robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = param_or<std::string>(this, "odom_child_frame_id", "base_link");

    use_imu = param_or<bool>(this, "use_imu", true);
    invert_acc = param_or<bool>(this, "invert_acc", false);
    invert_gyro = param_or<bool>(this, "invert_gyro", false);
    if(use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      imu_sub = create_subscription<sensor_msgs::msg::Imu>(imu_topic, 256, std::bind(&HdlLocalizationNode::imu_callback, this, std::placeholders::_1));
    }
    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic,
      rclcpp::QoS(5).best_effort(),
      std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));
    initialpose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalizationNode::initialpose_callback, this, std::placeholders::_1));

    pose_pub = create_publisher<nav_msgs::msg::Odometry>("/hdl_localization/odom", 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/hdl_localization/aligned_points", 5);
    status_pub = create_publisher<hdl_localization_ros2::msg::ScanMatchingStatus>("/hdl_localization/status", 5);

    relocalize_server = create_service<std_srvs::srv::Empty>("/relocalize", std::bind(&HdlLocalizationNode::relocalize, this, std::placeholders::_1, std::placeholders::_2));
  }

  // must be called after the node is owned by a shared_ptr
  void init() {
    tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    std::string reg_method = param_or<std::string>(const_cast<HdlLocalizationNode*>(this), "reg_method", "NDT_OMP");
    std::string ndt_neighbor_search_method = param_or<std::string>(const_cast<HdlLocalizationNode*>(this), "ndt_neighbor_search_method", "DIRECT7");
    double ndt_resolution = param_or<double>(const_cast<HdlLocalizationNode*>(this), "ndt_resolution", 1.0);

    if(reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if(ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if(ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if(ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    imu_topic = param_or<std::string>(this, "imu_topic", "/imu/data");
    points_topic = param_or<std::string>(this, "points_topic", "/velodyne_points");

    // intialize scan matching method
    double downsample_resolution = param_or<double>(this, "downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(get_logger(), "create registration method for localization");
    registration = create_registration();

    // global localization fallback (delta estimation during relocalization)
    RCLCPP_INFO(get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // load global map
    std::string globalmap_filename = param_or<std::string>(this, "globalmap_filename", "globalmap.pcd");
    RCLCPP_INFO_STREAM(get_logger(), "loading global map from " << globalmap_filename);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if(pcl::io::loadPCDFile(globalmap_filename, *cloud) != 0) {
      RCLCPP_ERROR(get_logger(), "failed to load the global map!!");
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "global map loaded: " << cloud->size() << " points");
      globalmap = cloud;
      registration->setInputTarget(globalmap);
    }

    // initialize pose estimator
    if(param_or<bool>(this, "specify_init_pose", true)) {
      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
        Eigen::Vector3f(param_or<double>(this, "init_pos_x", 0.0), param_or<double>(this, "init_pos_y", 0.0), param_or<double>(this, "init_pos_z", 0.0)),
        Eigen::Quaternionf(param_or<double>(this, "init_ori_w", 1.0), param_or<double>(this, "init_ori_x", 0.0), param_or<double>(this, "init_ori_y", 0.0), param_or<double>(this, "init_ori_z", 0.0)),
        param_or<double>(this, "cool_time_duration", 0.5)
      ));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg) {
    if(!globalmap) {
      RCLCPP_ERROR(get_logger(), "globalmap has not been loaded!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if(pcl_cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    rclcpp::Time stamp_t(stamp);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if(tf_buffer.canTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp_t, tf2::durationFromSec(0.1))) {
      try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp_t, tf2::durationFromSec(0.1));
        Eigen::Isometry3f mat = tf2::transformToEigen(transform).cast<float>();
        pcl::transformPointCloud(*pcl_cloud, *cloud, mat.matrix());
        cloud->header.frame_id = odom_child_frame_id;
        cloud->header.stamp = pcl_cloud->header.stamp;
      } catch(tf2::TransformException& e) {
        RCLCPP_ERROR_STREAM(get_logger(), "point cloud cannot be transformed into target frame!!: " << e.what());
        return;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "cannot transform point cloud into base_link (no tf)");
      return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    if(relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      RCLCPP_ERROR(get_logger(), "waiting for initial pose input!!");
      return;
    }
    Eigen::Matrix4f before = pose_estimator->matrix();

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp_t);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc ? -1.0 : 1.0;
        double gyro_sign = invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict(rclcpp::Time((*imu_iter)->header.stamp), acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if(param_or<bool>(this, "enable_robot_odometry_prediction", false) && last_correction_time.nanoseconds() != 0) {
      geometry_msgs::msg::TransformStamped odom_delta;
      if(tf_buffer.canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp_t, robot_odom_frame_id, tf2::durationFromSec(0.1))) {
        odom_delta = tf_buffer.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp_t, robot_odom_frame_id, tf2::durationFromSec(0.1));
      }

      if(odom_delta.header.stamp.sec == 0 && odom_delta.header.stamp.nanosec == 0) {
        RCLCPP_WARN_STREAM(get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator->correct(stamp_t, filtered);

    if(aligned_pub->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_pub->publish(aligned_msg);
    }

    if(status_pub->get_subscription_count()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief perform relocalization with the last scan (without global localization this simply restarts from the configured initial pose)
   */
  void relocalize(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    if(last_scan == nullptr) {
      RCLCPP_INFO(get_logger(), "no scan has been received");
      return;
    }

    relocalizing = true;
    delta_estimater->reset();

    RCLCPP_INFO(get_logger(), "relocalization requested (global localization not ported: restart from initial pose)");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    pose_estimator.reset(new hdl_localization::PoseEstimator(
      registration,
      Eigen::Vector3f(param_or<double>(this, "init_pos_x", 0.0), param_or<double>(this, "init_pos_y", 0.0), param_or<double>(this, "init_pos_z", 0.0)),
      Eigen::Quaternionf(param_or<double>(this, "init_ori_w", 1.0), param_or<double>(this, "init_ori_x", 0.0), param_or<double>(this, "init_ori_y", 0.0), param_or<double>(this, "init_ori_z", 0.0)),
      param_or<double>(this, "cool_time_duration", 0.5)));

    relocalizing = false;
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            param_or<double>(this, "cool_time_duration", 0.5))
    );
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const builtin_interfaces::msg::Time& stamp_msg, const Eigen::Matrix4f& pose) {
    rclcpp::Time stamp(stamp_msg);
    // broadcast the transform over tf
    if(tf_buffer.canTransform(robot_odom_frame_id, odom_child_frame_id, tf2::TimePointZero)) {
      geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = odom_child_frame_id;
      map_wrt_frame.child_frame_id = "map";

      geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform(robot_odom_frame_id, odom_child_frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::msg::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      Eigen::Isometry3d odom_wrt_map = tf2::transformToEigen(map_wrt_odom).inverse();

      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.transform = tf2::eigenToTransform(odom_wrt_map).transform;
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = robot_odom_frame_id;

      tf_broadcaster->sendTransform(odom_trans);
    } else {
      geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = odom_child_frame_id;
      tf_broadcaster->sendTransform(odom_trans);
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    odom.pose.pose.orientation.w = quat.w();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  /**
   * @brief publish scan matching status information
   */
  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    hdl_localization_ros2::msg::ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = 0.0;

    const double max_correspondence_dist = param_or<double>(this, "status_max_correspondence_dist", 0.5);
    const double max_valid_point_dist = param_or<double>(this, "status_max_valid_point_dist", 25.0);

    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(size_t i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      if(pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      num_valid_points++;

      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= std::max(1, num_inliers);
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    if(pose_estimator->wo_prediction_error()) {
      std_msgs::msg::String label;
      label.data = "without_pred";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if(pose_estimator->imu_prediction_error()) {
      std_msgs::msg::String label;
      label.data = use_imu ? "imu" : "motion_model";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }

    if(pose_estimator->odom_prediction_error()) {
      std_msgs::msg::String label;
      label.data = "odom";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub->publish(status);
  }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  std::string imu_topic;
  std::string points_topic;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization_ros2::msg::ScanMatchingStatus>::SharedPtr status_pub;

  tf2_ros::Buffer tf_buffer{get_clock()};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // relocalization
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;
};

}  // namespace hdl_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hdl_localization::HdlLocalizationNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
