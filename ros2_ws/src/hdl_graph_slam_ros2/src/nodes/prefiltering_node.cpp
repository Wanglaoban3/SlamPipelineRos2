// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port of prefiltering_nodelet.cpp (standalone node, tf1 -> tf2)
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_graph_slam/param_helper.hpp>

namespace hdl_graph_slam {

class PrefilteringNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNode() : Node("prefiltering_node") {
    initialize_params();

    if(param_or<bool>(this, "deskewing", false)) {
      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 1,
        std::bind(&PrefilteringNode::imu_callback, this, std::placeholders::_1));
    }

    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic,
      rclcpp::QoS(64).best_effort(),
      std::bind(&PrefilteringNode::cloud_callback, this, std::placeholders::_1));
    points_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 32);

    tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
  }

private:
  void initialize_params() {
    points_topic = param_or<std::string>(this, "points_topic", "/velodyne_points");
    imu_topic = param_or<std::string>(this, "imu_topic", "/imu/data");

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
    }

    std::string outlier_removal_method = param_or<std::string>(this, "outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = param_or<int>(this, "statistical_mean_k", 20);
      double stddev_mul_thresh = param_or<double>(this, "statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = param_or<double>(this, "radius_radius", 0.8);
      int min_neighbors = param_or<int>(this, "radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter = param_or<bool>(this, "use_distance_filter", true);
    distance_near_thresh = param_or<double>(this, "distance_near_thresh", 1.0);
    distance_far_thresh = param_or<double>(this, "distance_far_thresh", 100.0);

    base_link_frame = param_or<std::string>(this, "base_link_frame", "");
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu_queue.push_back(imu_msg);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
    pcl::PointCloud<PointT>::Ptr src_cloud_raw(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *src_cloud_raw);
    if(src_cloud_raw->empty()) {
      return;
    }

    pcl::PointCloud<PointT>::ConstPtr src_cloud = deskewing(src_cloud_raw);

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!base_link_frame.empty()) {
      if(!tf_buffer.canTransform(base_link_frame, src_cloud->header.frame_id, tf2::TimePointZero)) {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
        return;
      }

      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer.lookupTransform(base_link_frame, src_cloud->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
      } catch(tf2::TransformException& e) {
        std::cerr << "lookupTransform failed: " << e.what() << std::endl;
        return;
      }

      Eigen::Isometry3f mat = tf2::transformToEigen(transform).cast<float>();
      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*src_cloud, *transformed, mat);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = src_cloud->header.stamp;
      src_cloud = transformed;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered, filtered_msg);
    points_pub->publish(filtered_msg);
  }

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

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      return d > distance_near_thresh && d < distance_far_thresh;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    if(imu_queue.empty()) {
      return cloud;
    }

    // pcl header stamp is in MICROSECONDS (pcl_conversions convention), rclcpp::Time takes ns
    rclcpp::Time stamp(cloud->header.stamp * 1000ULL, RCL_ROS_TIME);

    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    auto loc = imu_queue.begin();
    for(; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);
      if(rclcpp::Time((*loc)->header.stamp) > stamp) {
        break;
      }
    }

    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    double scan_period = param_or<double>(this, "scan_period", 0.1);
    for(size_t i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

private:
  std::string points_topic;
  std::string imu_topic;

  std::mutex imu_mutex;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;

  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

  tf2_ros::Buffer tf_buffer{get_clock()};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

}  // namespace hdl_graph_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hdl_graph_slam::PrefilteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
