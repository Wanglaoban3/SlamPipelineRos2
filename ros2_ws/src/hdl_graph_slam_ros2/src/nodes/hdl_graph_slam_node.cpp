// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port of hdl_graph_slam_nodelet.cpp (standalone node)
// - geodesy(UTM) -> GeographicLib
// - WallTimers -> rclcpp wall timers
// - services: hdl_graph_slam_ros2/SaveMap, LoadGraph, DumpGraph
// - NMEA input dropped (use sensor_msgs/NavSatFix on gps_navsat_topic)
// - auto save map on shutdown (auto_save_map / map_file_path params)
#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <deque>
#include <sstream>
#include <fstream>
#include <array>
#include <filesystem>

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <hdl_graph_slam_ros2/msg/floor_coeffs.hpp>

#include <hdl_graph_slam_ros2/srv/save_map.hpp>
#include <hdl_graph_slam_ros2/srv/load_graph.hpp>
#include <hdl_graph_slam_ros2/srv/dump_graph.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <GeographicLib/UTMUPS.hpp>

#include <hdl_graph_slam/param_helper.hpp>
#include <hdl_graph_slam/ros_utils.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

namespace hdl_graph_slam {

class HdlGraphSlamNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;
  HdlGraphSlamNode() : Node("hdl_graph_slam_node") {
    // init parameters
    published_odom_topic = param_or<std::string>(this, "published_odom_topic", "/odom");
    map_frame_id = param_or<std::string>(this, "map_frame_id", "map");
    odom_frame_id = param_or<std::string>(this, "odom_frame_id", "odom");
    map_cloud_resolution = param_or<double>(this, "map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();

    max_keyframes_per_update = param_or<int>(this, "max_keyframes_per_update", 10);

    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(param_or<std::string>(this, "g2o_solver_type", "lm_var")));
    ParamNodeHandle pnh(this);
    keyframe_updater.reset(new KeyframeUpdater(pnh));
    loop_detector.reset(new LoopDetector(pnh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(pnh));

    gps_time_offset = param_or<double>(this, "gps_time_offset", 0.0);
    gps_edge_stddev_xy = param_or<double>(this, "gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = param_or<double>(this, "gps_edge_stddev_z", 10.0);
    floor_edge_stddev = param_or<double>(this, "floor_edge_stddev", 10.0);

    imu_time_offset = param_or<double>(this, "imu_time_offset", 0.0);
    enable_imu_orientation = param_or<bool>(this, "enable_imu_orientation", false);
    enable_imu_acceleration = param_or<bool>(this, "enable_imu_acceleration", false);
    imu_orientation_edge_stddev = param_or<double>(this, "imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = param_or<double>(this, "imu_acceleration_edge_stddev", 3.0);
    imu_topic = param_or<std::string>(this, "imu_topic", "/imu/data");

    // subscribers (message_filters needs the node as shared_ptr -> created in init())
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1024, std::bind(&HdlGraphSlamNode::imu_callback, this, std::placeholders::_1));
    floor_sub = create_subscription<hdl_graph_slam_ros2::msg::FloorCoeffs>("/floor_detection/floor_coeffs", 1024, std::bind(&HdlGraphSlamNode::floor_coeffs_callback, this, std::placeholders::_1));

    if(param_or<bool>(this, "enable_gps", true)) {
      std::string navsat_topic = param_or<std::string>(this, "gps_navsat_topic", "/gps/navsat");
      navsat_sub = create_subscription<sensor_msgs::msg::NavSatFix>(navsat_topic, 1024, std::bind(&HdlGraphSlamNode::navsat_callback, this, std::placeholders::_1));
    }

    // publishers
    markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = create_publisher<geometry_msgs::msg::TransformStamped>("/hdl_graph_slam/odom2map", 16);
    map_points_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/hdl_graph_slam/map_points", rclcpp::QoS(1).transient_local());

    load_service_server = create_service<hdl_graph_slam_ros2::srv::LoadGraph>("/hdl_graph_slam/load", std::bind(&HdlGraphSlamNode::load_service, this, std::placeholders::_1, std::placeholders::_2));
    dump_service_server = create_service<hdl_graph_slam_ros2::srv::DumpGraph>("/hdl_graph_slam/dump", std::bind(&HdlGraphSlamNode::dump_service, this, std::placeholders::_1, std::placeholders::_2));
    save_map_service_server = create_service<hdl_graph_slam_ros2::srv::SaveMap>("/hdl_graph_slam/save_map", std::bind(&HdlGraphSlamNode::save_map_service, this, std::placeholders::_1, std::placeholders::_2));

    graph_updated = false;
    double graph_update_interval = param_or<double>(this, "graph_update_interval", 3.0);
    double map_cloud_update_interval = param_or<double>(this, "map_cloud_update_interval", 10.0);
    optimization_timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(graph_update_interval)), std::bind(&HdlGraphSlamNode::optimization_timer_callback, this));
    map_publish_timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(map_cloud_update_interval)), std::bind(&HdlGraphSlamNode::map_points_publish_timer_callback, this));

    auto_save_map = param_or<bool>(this, "auto_save_map", true);
    map_file_path = param_or<std::string>(this, "map_file_path", "globalmap.pcd");
    fix_first_node = param_or<bool>(this, "fix_first_node", false);
    fix_first_node_stddev = param_or<std::string>(this, "fix_first_node_stddev", "1 1 1 1 1 1");
  }

  ~HdlGraphSlamNode() override {
    // called after spin() returns on Ctrl+C: save the final map
    if(auto_save_map && graph_updated && !keyframes_snapshot.empty()) {
      std::cout << "auto saving map to " << map_file_path << std::endl;
      hdl_graph_slam_ros2::srv::SaveMap::Request req;
      hdl_graph_slam_ros2::srv::SaveMap::Response res;
      req.resolution = map_cloud_resolution;
      req.utm = false;
      req.destination = map_file_path;
      save_map_impl(req, res);
      if(res.success) {
        std::cout << "map saved: " << map_file_path << std::endl;
      } else {
        std::cerr << "failed to save map!!" << std::endl;
      }
    }
  }

private:
  /**
   * @brief received point clouds are paired with the nearest buffered odometry and pushed to #keyframe_queue.
   * Plain subscriptions + nearest-time pairing replace message_filters: robust to DDS arrival-order races.
   */
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    static int odom_count = 0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "odom recv #%d", ++odom_count);
    std::lock_guard<std::mutex> lock(odom_buffer_mutex);
    double t = rclcpp::Time(odom_msg->header.stamp).seconds();
    odom_buffer.push_back({t, odom_msg});
    if(odom_buffer.size() > 64) {
      odom_buffer.pop_front();
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    static int sync_count = 0;
    const double stamp_s = rclcpp::Time(cloud_msg->header.stamp).seconds();

    double best_dt = 1e9;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    {
      std::lock_guard<std::mutex> lock(odom_buffer_mutex);
      for(const auto& entry : odom_buffer) {
        double dt = std::abs(entry.first - stamp_s);
        if(dt < best_dt) {
          best_dt = dt;
          odom_msg = entry.second;
        }
      }
    }
    if(!odom_msg || best_dt > 0.2) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "cloud dropped: have_odom=%d best_dt=%.3f", odom_msg != nullptr, best_dt);
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "sync callback #%d (odom t=%.3f)", ++sync_count, rclcpp::Time(odom_msg->header.stamp).seconds());
    const rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        return;
      }
      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  void navsat_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navsat_msg) {
    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp = rclcpp::Time(gps_msg->header.stamp) + rclcpp::Duration::from_seconds(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief add gps prior edges for keyframes
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > rclcpp::Time(gps_queue.back()->header.stamp)) {
        break;
      }

      if(keyframe->stamp < rclcpp::Time((*gps_cursor)->header.stamp) || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = (rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*gps)->header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if(0.2 < std::abs((rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      double lat = (*closest_gps)->position.latitude;
      double lon = (*closest_gps)->position.longitude;
      double alt = (*closest_gps)->position.altitude;
      int zone;
      bool northp;
      double easting, northing;
      try {
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
      } catch(std::exception& e) {
        std::cerr << "warning: failed to convert GPS to UTM: " << e.what() << std::endl;
        continue;
      }
      Eigen::Vector3d xyz(easting, northing, alt);

      // the first gps data position will be the origin of the map
      if(!zero_utm) {
        zero_utm = xyz;
      }
      xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(xyz.z())) {
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "gps_edge_robust_kernel", "NONE"), param_or<double>(this, "gps_edge_robust_kernel_size", 1.0));

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const rclcpp::Time& stamp, const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr& geopoint) { return stamp < rclcpp::Time(geopoint->header.stamp); });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    if(!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp = rclcpp::Time(imu_msg->header.stamp) + rclcpp::Duration::from_seconds(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > rclcpp::Time(imu_queue.back()->header.stamp)) {
        break;
      }

      if(keyframe->stamp < rclcpp::Time((*imu_cursor)->header.stamp) || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = (rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*imu)->header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs((rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      // IMU data is assumed to be already expressed in the base_link frame
      // (our converter publishes /imu/data with frame_id = base_link)
      keyframe->acceleration = Eigen::Vector3d(imu_acc.x, imu_acc.y, imu_acc.z);
      keyframe->orientation = Eigen::Quaterniond(imu_ori.w, imu_ori.x, imu_ori.y, imu_ori.z);
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if(enable_imu_orientation) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "imu_orientation_edge_robust_kernel", "NONE"), param_or<double>(this, "imu_orientation_edge_robust_kernel_size", 1.0));
      }

      if(enable_imu_acceleration) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "imu_acceleration_edge_robust_kernel", "NONE"), param_or<double>(this, "imu_acceleration_edge_robust_kernel_size", 1.0));
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::ConstSharedPtr& imu) { return stamp < rclcpp::Time(imu->header.stamp); });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam_ros2::msg::FloorCoeffs::ConstSharedPtr& floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(rclcpp::Time(floor_coeffs->header.stamp) > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(rclcpp::Time(floor_coeffs->header.stamp).nanoseconds());
      if(found == keyframe_hash.end()) {
        continue;
      }

      if(!floor_plane_node) {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "floor_edge_robust_kernel", "NONE"), param_or<double>(this, "floor_edge_robust_kernel_size", 1.0));

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const rclcpp::Time& stamp, const hdl_graph_slam_ros2::msg::FloorCoeffs::ConstSharedPtr& coeffs) { return stamp < rclcpp::Time(coeffs->header.stamp); });
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   */
  void map_points_publish_timer_callback() {
    if(!map_points_pub->get_subscription_count() || !graph_updated) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub->publish(*cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   */
  void optimization_timer_callback() {
    static int tick = 0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "optimization tick #%d", ++tick);
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "loop_closure_edge_robust_kernel", "NONE"), param_or<double>(this, "loop_closure_edge_robust_kernel_size", 1.0));
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    if(anchor_node && param_or<bool>(this, "fix_first_node_adaptive", true)) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    // optimize the pose graph
    int num_iterations = param_or<int>(this, "g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);

    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    if(odom2map_pub->get_subscription_count()) {
      geometry_msgs::msg::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub->publish(ts);
    }
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    std::cout << "flush_keyframe_queue - keyframes len:" << keyframes.size() << std::endl;
    int num_processed = 0;
    for(size_t i = 0; i < std::min<size_t>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp.nanoseconds()] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(fix_first_node) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::stringstream sst(fix_first_node_stddev);
          for(int j = 0; j < 6; j++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(j, j) = 1.0 / stddev;
          }

          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, param_or<std::string>(this, "odometry_edge_robust_kernel", "NONE"), param_or<double>(this, "odometry_edge_robust_kernel_size", 1.0));
    }

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  /**
   * @brief dump all data to a directory
   */
  void dump_service(const std::shared_ptr<hdl_graph_slam_ros2::srv::DumpGraph::Request> req, std::shared_ptr<hdl_graph_slam_ros2::srv::DumpGraph::Response> res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req->destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!std::filesystem::is_directory(directory)) {
      std::filesystem::create_directory(directory);
    }

    std::cout << "dumping data to:" << directory << std::endl;
    // save graph
    graph_slam->save(directory + "/graph.g2o");

    // save keyframes
    for(size_t i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
    ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

    res->success = true;
  }

  /**
   * @brief save map data as pcd
   */
  void save_map_service(const std::shared_ptr<hdl_graph_slam_ros2::srv::SaveMap::Request> req, std::shared_ptr<hdl_graph_slam_ros2::srv::SaveMap::Response> res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req->resolution);
    if(!cloud) {
      res->success = false;
      return;
    }

    if(zero_utm && req->utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req->destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req->destination, *cloud);
    res->success = ret == 0;
  }

  // non-service variant used by the auto-save destructor hook
  void save_map_impl(const hdl_graph_slam_ros2::srv::SaveMap::Request& req, hdl_graph_slam_ros2::srv::SaveMap::Response& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return;
    }

    if(zero_utm && req.utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;
  }

  /**
   * @brief load all data from a directory
   */
  void load_service(const std::shared_ptr<hdl_graph_slam_ros2::srv::LoadGraph::Request> req, std::shared_ptr<hdl_graph_slam_ros2::srv::LoadGraph::Response> res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req->path;

    std::cout << "loading data from:" << directory << std::endl;

    // Load graph.
    graph_slam->load(directory + "/graph.g2o");

    // Iterate over the items in this directory and count how many sub directories there are.
    size_t max_directory_count = 0;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
      if(entry.is_directory()) {
        max_directory_count++;
      }
    }

    // Load keyframes by looping through key frame indexes that we expect to see.
    for(size_t i = 0; i < max_directory_count; i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;
      std::string key_frame_directory = sst.str();

      // If key_frame_directory doesnt exist, then we have run out so lets stop looking.
      if(!std::filesystem::is_directory(key_frame_directory)) {
        break;
      }

      KeyFrame::Ptr keyframe(new KeyFrame(key_frame_directory, graph_slam->graph.get()));
      keyframes.push_back(keyframe);
    }
    std::cout << "loaded " << keyframes.size() << " keyframes" << std::endl;

    // Load special nodes.
    std::ifstream ifs(directory + "/special_nodes.csv");
    if(!ifs) {
      return;
    }
    while(!ifs.eof()) {
      std::string token;
      ifs >> token;
      if(token == "anchor_node") {
        int id = 0;
        ifs >> id;
        anchor_node = static_cast<g2o::VertexSE3*>(graph_slam->graph->vertex(id));
      } else if(token == "anchor_edge") {
        int id = 0;
        ifs >> id;
        if(anchor_node) {
          auto edges = anchor_node->edges();

          for(auto e : edges) {
            int edgeID = e->id();
            if(edgeID == id) {
              anchor_edge = static_cast<g2o::EdgeSE3*>(e);

              break;
            }
          }
        }
      } else if(token == "floor_node") {
        int id = 0;
        ifs >> id;
        floor_plane_node = static_cast<g2o::VertexPlane*>(graph_slam->graph->vertex(id));
      }
    }

    // Update our keyframe snapshot so we can publish a map update
    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());

    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    res->success = true;

    std::cout << "snapshot updated" << std::endl << "loading successful" << std::endl;
  }

public:
  // must be called after the node is owned by a shared_ptr
  void init() {
    RCLCPP_INFO(get_logger(), "subscribing odometry topic: %s (must match the odometry source!)", published_odom_topic.c_str());
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(published_odom_topic, rclcpp::QoS(256), std::bind(&HdlGraphSlamNode::odom_callback, this, std::placeholders::_1));
    cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_points", rclcpp::QoS(64).best_effort(), std::bind(&HdlGraphSlamNode::cloud_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

  std::mutex odom_buffer_mutex;
  std::deque<std::pair<double, nav_msgs::msg::Odometry::ConstSharedPtr>> odom_buffer;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<hdl_graph_slam_ros2::msg::FloorCoeffs>::SharedPtr floor_sub;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;

  std::string published_odom_topic;
  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;

  rclcpp::Service<hdl_graph_slam_ros2::srv::LoadGraph>::SharedPtr load_service_server;
  rclcpp::Service<hdl_graph_slam_ros2::srv::DumpGraph>::SharedPtr dump_service_server;
  rclcpp::Service<hdl_graph_slam_ros2::srv::SaveMap>::SharedPtr save_map_service_server;

  // params for first node fixing
  bool fix_first_node;
  std::string fix_first_node_stddev;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::msg::GeoPointStamped::ConstSharedPtr> gps_queue;

  // imu queue
  std::string imu_topic;
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue;

  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<hdl_graph_slam_ros2::msg::FloorCoeffs::ConstSharedPtr> floor_coeffs_queue;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  bool auto_save_map;
  std::string map_file_path;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<int64_t, KeyFrame::Ptr> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}  // namespace hdl_graph_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<hdl_graph_slam::HdlGraphSlamNode>();
    node->init();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
