#include "mapping_manager.h"
#include <algorithm>
#include <cmath>

namespace mot_mapping {

MappingRos::MappingRos(ros::NodeHandle &nh):nh(nh) {}

MappingRos::~MappingRos() {std::cout << "Exit Tracker" << std::endl;}

void MappingRos::init() {
  t_start = ros::Time::now();
  frame_num = 1;
  id = 0;

  Remaining_Points.reset(new Points);

  mp_ = std::make_unique<MapParam>();
  md_ = std::make_unique<MapData>();

  nh.param("dbscan/core_pts", mp_->core_pts, 4);
  nh.param("dbscan/tolerance", mp_->tolerance, 0.02);
  nh.param("dbscan/min_cluster", mp_->min_cluster, 20);
  nh.param("dbscan/max_cluster", mp_->max_cluster, 800);
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
  nh.param("detection/enable_size_filter", mp_->enable_size_filter, false);
  nh.param("detection/size_filter_keep_tracked", mp_->size_filter_keep_tracked, true);
  nh.param("detection/min_size_x", mp_->min_object_size.x(), 0.0);
  nh.param("detection/min_size_y", mp_->min_object_size.y(), 0.0);
  nh.param("detection/min_size_z", mp_->min_object_size.z(), 0.0);
  nh.param("detection/max_size_x", mp_->max_object_size.x(), 100.0);
  nh.param("detection/max_size_y", mp_->max_object_size.y(), 100.0);
  nh.param("detection/max_size_z", mp_->max_object_size.z(), 100.0);
  nh.param("detection/enable_roi_filter", mp_->enable_roi_filter, false);
  nh.param("detection/roi_min_x", mp_->roi_min.x(), -1000.0);
  nh.param("detection/roi_min_y", mp_->roi_min.y(), -1000.0);
  nh.param("detection/roi_min_z", mp_->roi_min.z(), -1000.0);
  nh.param("detection/roi_max_x", mp_->roi_max.x(), 1000.0);
  nh.param("detection/roi_max_y", mp_->roi_max.y(), 1000.0);
  nh.param("detection/roi_max_z", mp_->roi_max.z(), 1000.0);
  for (int i = 0; i < 3; ++i) {
    if (mp_->roi_min[i] > mp_->roi_max[i]) {
      ROS_WARN_STREAM("detection ROI min > max on axis " << i << ", swapping values.");
      std::swap(mp_->roi_min[i], mp_->roi_max[i]);
    }
  }
  nh.param("lidar/range_x", mp_->Range[0], 15.0);
  nh.param("lidar/range_y", mp_->Range[1], 15.0);
  nh.param("lidar/range_z", mp_->Range[2], 1.0);
  nh.param("lidar/input_cloud_in_world", mp_->input_cloud_in_world, true);
  nh.param("filter/filter_leaf_size", mp_->filter_leaf_size, 0.1);
  nh.param("filter/cluster_leaf_size", mp_->cluster_leaf_size, 0.05);
  nh.param("filter/output_leaf_size", mp_->output_leaf_size, 0.05);
  nh.param("ikdtree/resolution", mp_->ikdtree_resolution, 0.1);
  nh.param("ikdtree/history_window_size", mp_->history_window_size, 20);
  nh.param("mapping/cloud_buffer_size", mp_->cloud_buffer_size, 2);
  nh.param("detection/max_avg_dist", mp_->max_avg_dist, 5.0);
  nh.param("tracking/on_track_gate", mp_->on_track_gate, 0.55);
  nh.param("tracking/on_track_min_age", mp_->on_track_min_age, 3);
  nh.param("tracking/on_track_min_vel", mp_->on_track_min_vel, 0.05);
  nh.param("tracking/association_gate", mp_->association_gate, 0.8);
  nh.param("tracking/tracker_timeout_frames", mp_->tracker_timeout_frames, 20);
  nh.param("tracking/velocity_blend", mp_->velocity_blend, 0.5);
  nh.param("tracking/horizontal_motion_prior", mp_->horizontal_motion_prior, false);
  nh.param("tracking/vertical_velocity_scale", mp_->vertical_velocity_scale, 1.0);
  nh.param("tracking/max_vertical_velocity", mp_->max_vertical_velocity, -1.0);
  mp_->vertical_velocity_scale = std::max(0.0, std::min(1.0, mp_->vertical_velocity_scale));
  nh.param("tracking/ekf_dt", mp_->ekf_dt, 0.1);
  nh.param("tracking/ekf_predict_period", mp_->ekf_predict_period, 0.1);
  nh.param("tracking/static_map_publish_period", mp_->static_map_publish_period, 0.1);
  nh.param("tracking/ekf_q_pos", mp_->ekf_q_pos, 0.1);
  nh.param("tracking/ekf_q_vel", mp_->ekf_q_vel, 0.1);
  nh.param("tracking/ekf_r_pos", mp_->ekf_r_pos, 0.09);
  nh.param("tracking/ekf_r_vel", mp_->ekf_r_vel, 0.4);
  nh.param("tracking/ekf_adaptive_q", mp_->ekf_adaptive_q, true);
  nh.param("tracking/ekf_innovation_window", mp_->ekf_innovation_window, 20);
  nh.param("tracking/ekf_q_min", mp_->ekf_q_min, 1e-4);
  nh.param("tracking/ekf_q_max", mp_->ekf_q_max, 10.0);
  nh.param("tracking/ekf_max_vel", mp_->ekf_max_vel, 4.0);
  nh.param("debug/verbose", mp_->verbose, false);
  nh.param("ros/cloud_queue_size", mp_->cloud_queue_size, 50);
  nh.param("ros/odom_queue_size", mp_->odom_queue_size, 25);
  nh.param("ros/sync_queue_size", mp_->sync_queue_size, 100);
  nh.param("ros/frame_id", mp_->frame_id, string("world"));
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));
  nh.param("ros/dynamic_points_topic", mp_->dynamic_points_topic, string("/dynamic_points"));
  nh.param("ros/map_topic", mp_->map_topic, string("/map_ros"));
  nh.param("ros/static_map_topic", mp_->static_map_topic, string("/static_map"));
  nh.param("ros/box_edge_topic", mp_->box_edge_topic, string("/box_edge"));
  nh.param("ros/object_pose_topic", mp_->object_pose_topic, string("/object_pose"));
  nh.param("ros/states_topic", mp_->states_topic, string("/states"));

  ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, mp_->ikdtree_resolution));

  cloudPub = nh.advertise<sensor_msgs::PointCloud2>(mp_->dynamic_points_topic, 10);
  mapPub = nh.advertise<sensor_msgs::PointCloud2>(mp_->map_topic, 10);
  staticMapPub = nh.advertise<sensor_msgs::PointCloud2>(mp_->static_map_topic, 10);
  edgePub = nh.advertise<visualization_msgs::MarkerArray>(mp_->box_edge_topic, 1000);
  objectPosePub = nh.advertise<visualization_msgs::MarkerArray>(mp_->object_pose_topic, 10);
  statesPub = nh.advertise<obj_state_msgs::ObjectsStates>(mp_->states_topic, 10);

  std::cout << "INIT!" << std::endl;

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, mp_->lidar_topic, mp_->cloud_queue_size));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, mp_->odom_topic, mp_->odom_queue_size));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(mp_->sync_queue_size), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));

  ekf_predict_timer_ = nh.createTimer(ros::Duration(mp_->ekf_predict_period), &MappingRos::ekfPredictCallback, this);
  map_pub_timer_ = nh.createTimer(ros::Duration(mp_->static_map_publish_period), &MappingRos::mapPubCallback, this);
}


float MappingRos::calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w) {
  int num = p_b->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_b->points[i].x, 
         p_b->points[i].y, 
         p_b->points[i].z;
    if (!mp_->input_cloud_in_world) {
      p = md_->R * p + md_->T;
    }
    const Eigen::Vector3d rel = p - md_->T;
    if (std::abs(rel.x()) < mp_->Range.x() &&
        std::abs(rel.y()) < mp_->Range.y() &&
        std::abs(rel.z()) < mp_->Range.z()) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

void MappingRos::pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T) {
  int num = p_w->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_w->points[i].x, 
         p_w->points[i].y, 
         p_w->points[i].z;
    p = (p - md_->T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, 
                                   const nav_msgs::OdometryConstPtr& odom) {
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;
       
  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  if (mp_->verbose) {
    printf("\033[2J");
    printf("\033[1;1H");
  }
  
  // Pointcloud Filter
  PointsPtr cloud_filtered(new Points);
  vox.setInputCloud(latest_cloud);
  vox.setLeafSize(mp_->filter_leaf_size, mp_->filter_leaf_size, mp_->filter_leaf_size);
  vox.filter(*cloud_filtered);

  PointsPtr cloud_world(new Points);
  PointsPtr nonfilter_pts(new Points);
  PointsPtr PointToAdd(new Points);
  PointsPtr ClusterPoints(new Points);
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts);

  buffer.push_back(nonfilter_pts);
  while ((int)buffer.size() > std::max(1, mp_->cloud_buffer_size)) {
    buffer.pop_front();
  }
  for (size_t i = 0; i < buffer.size(); ++i) {
    *ClusterPoints += *buffer[i];
  }
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(mp_->cluster_leaf_size, mp_->cluster_leaf_size, mp_->cluster_leaf_size);
  vox.filter(*ClusterPoints);

  Remaining_Points = ClusterPoints;
  
  PointsPtr All_Points(new Points);

  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  if (mp_->verbose) {
    std::cout << "Filter Time Cost (ms): " << compTime << std::endl;
  }
  tic = std::chrono::high_resolution_clock::now();

  if (mp_->verbose) {
    std::cout << "Input Size:" << ClusterPoints->size() << std::endl;
  }
  
  sensor_msgs::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(mp_->output_leaf_size, mp_->output_leaf_size, mp_->output_leaf_size);
  vox.filter(*OutputPoints);

  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = mp_->frame_id;
  map_ros.header.stamp = msg->header.stamp;
  mapPub.publish(map_ros);

  // DBSCAN Cluster
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan.setCorePointMinPts(mp_->core_pts);
  dbscan.setClusterTolerance(mp_->tolerance);
  dbscan.setMinClusterSize(mp_->min_cluster);
  dbscan.setMaxClusterSize(mp_->max_cluster);
  dbscan.setInputCloud(ClusterPoints);
  dbscan.setSearchMethod();
  dbscan.extractNano(cluster_indices);

  if (mp_->verbose) {
    std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  if (mp_->verbose) {
    std::cout << "Cluster Time Cost (ms): " << compTime << std::endl;
  }
  tic = std::chrono::high_resolution_clock::now();

  if (frame_num < 2){
    previous_points.push_back(cloud_world);
    frame_num++;
    return;
  }

  int pt_size = previous_points[0]->points.size();
  //ikd-tree
  if (ikdtree_ptr->Root_Node == nullptr) {
    ikdtree_ptr->Build(previous_points[0]->points);
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  else {
    PointVector PointNoNeedDownsample;

    for (int i = 0; i < pt_size; ++i) {
      PointType mid_point; 
      const double res = mp_->ikdtree_resolution;
      mid_point.x = floor(previous_points[0]->points[i].x / res) * res + 0.5 * res;
      mid_point.y = floor(previous_points[0]->points[i].y / res) * res + 0.5 * res;
      mid_point.z = floor(previous_points[0]->points[i].z / res) * res + 0.5 * res;
      // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

      if (points_near.empty() ||
          fabs(points_near[0].x - mid_point.x) > 0.5 * res || 
          fabs(points_near[0].y - mid_point.y) > 0.5 * res || 
          fabs(points_near[0].z - mid_point.z) > 0.5 * res) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
        PointToAdd->points.push_back(previous_points[0]->points[i]);
      }
    }
    // int add_point_size = ikdtree_ptr->Add_Points(PointToAdd, true);
    ikdtree_ptr->Add_Points(PointNoNeedDownsample, false); 
    input_point.push_back(PointToAdd);
    while ((int)input_point.size() > std::max(1, mp_->history_window_size)) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr->Delete_Points(PointDelete);
      input_point.pop_front();
    }

    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  if (mp_->verbose) {
    std::cout << "ikd-Tree Time Cost (ms): " << compTime << std::endl;
  }
  tic = std::chrono::high_resolution_clock::now();
  if (mp_->verbose) {
    std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;
  }
  // ikdtree_ptr->flatten(ikdtree_ptr->Root_Node, ikdtree_ptr->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;

  // sensor_msgs::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloudPub.publish(previous_cloud);

  PointsPtr dynamic_points(new Points);

  int k = 0;
  detections.clear();
  deleted_indices.clear();
  for (auto& getIndices: cluster_indices) {
    PointsPtr cluster(new Points);
    PointVector PointDelete;
    double avg_dist = 0;
    Eigen::Vector3d cluster_center;
    cluster_center.setZero();
    std::vector<double> avg_buffer;
    for (auto& index : getIndices.indices) {
      cluster->points.push_back(ClusterPoints->points[index]);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(ClusterPoints->points[index], 3, points_near, pointSearchSqDis);
      if (pointSearchSqDis.empty()) {
        continue;
      }
      avg_dist += sqrt(pointSearchSqDis[0]);
      avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
      cluster_center += Eigen::Vector3d(ClusterPoints->points[index].x, ClusterPoints->points[index].y, ClusterPoints->points[index].z);
    }

    if (cluster->points.empty() || avg_buffer.empty()) {
      continue;
    }
    avg_dist = avg_dist / avg_buffer.size();
    cluster_center = cluster_center / avg_buffer.size();
    double var_dist = 0;
    for (auto& dist : avg_buffer) {
      const double avg_dist2 = std::max(avg_dist * avg_dist, 1e-6);
      var_dist += (dist - avg_dist) * (dist - avg_dist) / avg_dist2;
    }
    var_dist = var_dist/getIndices.indices.size();
    
    bool on_track = false;
    for (size_t i = 0; i < trackers.size(); ++i) {
      Eigen::Vector3d tracker_center = trackers[i]->pos();
      double dist = sqrt((cluster_center(0) - tracker_center(0)) * (cluster_center(0) - tracker_center(0)) + 
                         (cluster_center(1) - tracker_center(1)) * (cluster_center(1) - tracker_center(1)) + 
                         (cluster_center(2) - tracker_center(2)) * (cluster_center(2) - tracker_center(2)));
      Eigen::Vector3d tracker_vel = trackers[i]->vel();
      if (mp_->horizontal_motion_prior) {
        tracker_vel.z() = 0.0;
      }
      if (dist < mp_->on_track_gate &&
          trackers[i]->age > mp_->on_track_min_age &&
          tracker_vel.norm() > mp_->on_track_min_vel) {
        on_track = true;
        break;
      }
    }

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    pcl::PointXYZ minPt, maxPt;
	  pcl::getMinMax3D(*cluster, minPt, maxPt);

    PointType center_pt;
    center_pt.x = (maxPt.x + minPt.x)/2;
    center_pt.y = (maxPt.y + minPt.y)/2;
    center_pt.z = (maxPt.z + minPt.z)/2;

    const Eigen::Vector3d cluster_size(maxPt.x - minPt.x,
                                       maxPt.y - minPt.y,
                                       maxPt.z - minPt.z);
    const Eigen::Vector3d bbox_center(center_pt.x, center_pt.y, center_pt.z);
    bool roi_ok = true;
    if (mp_->enable_roi_filter) {
      roi_ok = (bbox_center.array() >= mp_->roi_min.array()).all() &&
               (bbox_center.array() <= mp_->roi_max.array()).all();
    }

    bool size_ok = true;
    if (mp_->enable_size_filter) {
      size_ok = (cluster_size.array() >= mp_->min_object_size.array()).all() &&
                (cluster_size.array() <= mp_->max_object_size.array()).all();
      if (on_track && mp_->size_filter_keep_tracked) {
        size_ok = true;
      }
    }

    if (roi_ok && size_ok &&
        ((avg_dist > mp_->thresh_dist && avg_dist < mp_->max_avg_dist && var_dist < mp_->thresh_var) || on_track)) {
      // std::cout << "on_track: " << on_track << std::endl;
      // std::cout << "var_dist: " << var_dist << std::endl;
      // std::cout << "avg_dist: " << avg_dist << std::endl;
      ObjectState state;
      state.id = k;
      state.position = cluster_center;
      state.size = cluster_size;
      detections.push_back(state);
      deleted_indices.push_back(getIndices);
      for (auto& index : getIndices.indices) {
        dynamic_points->points.push_back(ClusterPoints->points[index]);
      }
      // BoxPointType box;
      // box.vertex_min[0] = minPt.x-0.2; box.vertex_min[1] = minPt.y-0.2; box.vertex_min[2] = minPt.z-0.2;
      // box.vertex_max[0] = maxPt.x+0.2; box.vertex_max[1] = maxPt.y+0.2; box.vertex_max[2] = maxPt.z+0.2;
      // delete_boxes.push_back(box);
    }
  }

  sensor_msgs::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = mp_->frame_id;
  dynamic_pts.header.stamp = msg->header.stamp;
  cloudPub.publish(dynamic_pts);

  obj_state_msgs::ObjectsStates states;
  states.header.stamp = ros::Time::now();
  states.header.frame_id = mp_->frame_id;
  std::vector<std::pair<int, int>> matchedPairs;

  for (auto& tracker : trackers) {
    tracker->age += 1;
  }

  // Tracking & EKF
  if(trackers.size() == 0) {
    for(size_t i = 0; i < detections.size(); ++i) {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(mp_->ekf_dt);
      ekfPtr->configure(mp_->ekf_q_pos, mp_->ekf_q_vel,
                        mp_->ekf_r_pos, mp_->ekf_r_vel,
                        mp_->ekf_adaptive_q, mp_->ekf_innovation_window,
                        mp_->ekf_q_min, mp_->ekf_q_max, mp_->ekf_max_vel);
      ekfPtr->reset(detections[i].position, id);
      id++;
      trackers.push_back(ekfPtr);
      previous_p.push_back(detections[i].position);
      previous_v.push_back(Eigen::Vector3d::Zero());
    }
    statesPub.publish(states);
    return;
  }

  std::vector<bool> tracker_matched(trackers.size(), false);
  for (size_t i = 0; i < detections.size(); ++i) {
    double min_dist = 1000000;
    int min_index = -1;
    Eigen::Vector3d det_pos = detections[i].position;
    for (size_t j = 0; j < trackers.size(); ++j) {
      if (tracker_matched[j]) {
        continue;
      }
      Eigen::Vector3d track_pos = trackers[j]->pos();
      double dist = (det_pos - track_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_index = static_cast<int>(j);
      }
    }

    if (min_index >= 0 && min_dist < mp_->association_gate) {
      // std::cout << "id:" << trackers[min_index]->id << std::endl;
      const double dt = std::max(mp_->ekf_dt, (ros::Time::now() - trackers[min_index]->last_update_stamp_).toSec());
      Eigen::Vector3d prev_pos = min_index < (int)previous_p.size() ? previous_p[min_index] : trackers[min_index]->pos();
      Eigen::Vector3d prev_vel = min_index < (int)previous_v.size() ? previous_v[min_index] : Eigen::Vector3d::Zero();
      Eigen::Vector3d vel_detect = (detections[i].position - prev_pos) / dt;
      if (mp_->horizontal_motion_prior) {
        vel_detect.z() *= mp_->vertical_velocity_scale;
        prev_vel.z() *= mp_->vertical_velocity_scale;
      }
      Eigen::Vector3d meas_vel = mp_->velocity_blend * vel_detect + (1.0 - mp_->velocity_blend) * prev_vel;
      if (mp_->horizontal_motion_prior && mp_->max_vertical_velocity >= 0.0) {
        meas_vel.z() = std::min(mp_->max_vertical_velocity,
                                std::max(-mp_->max_vertical_velocity, meas_vel.z()));
      }
      trackers[min_index]->update(detections[i].position, meas_vel);
      if (mp_->horizontal_motion_prior) {
        trackers[min_index]->constrainVerticalVelocity(mp_->vertical_velocity_scale, mp_->max_vertical_velocity);
      }
      tracker_matched[min_index] = true;
      matchedPairs.push_back(std::make_pair(i, min_index));

      ObjectState state;
      state.position = trackers[min_index]->pos();
      state.velocity = trackers[min_index]->vel();
      obj_state_msgs::State statemsg;
      statemsg.header.stamp = ros::Time::now();
      statemsg.header.frame_id = mp_->frame_id;
      statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
      statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
      statemsg.size.x = detections[i].size[0]; statemsg.size.y = detections[i].size[1]; statemsg.size.z = detections[i].size[2];
      const Eigen::MatrixXd& cov = trackers[min_index]->cov();
      statemsg.poscov.x = cov(0, 0); statemsg.poscov.y = cov(1, 1); statemsg.poscov.z = cov(2, 2);
      states.states.push_back(statemsg);

    } else {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(mp_->ekf_dt);
      ekfPtr->configure(mp_->ekf_q_pos, mp_->ekf_q_vel,
                        mp_->ekf_r_pos, mp_->ekf_r_vel,
                        mp_->ekf_adaptive_q, mp_->ekf_innovation_window,
                        mp_->ekf_q_min, mp_->ekf_q_max, mp_->ekf_max_vel);
      ekfPtr->reset(detections[i].position, id);
      id++;
      trackers.push_back(ekfPtr);
      tracker_matched.push_back(true);
    }
  }

  statesPub.publish(states);

  visualizeFunction(matchedPairs);

  for (auto it = trackers.begin(); it != trackers.end();) {
      // std::cout << "dt:" << (*it)->age - (*it)->update_num << std::endl;
      if ((*it)->age - (*it)->update_num > mp_->tracker_timeout_frames)
        it = trackers.erase(it);
      else 
        it++;
  }    

  previous_p.clear();
  previous_v.clear();
  for (size_t i = 0; i < trackers.size(); ++i ) {
    previous_p.push_back(trackers[i]->pos());
    previous_v.push_back(trackers[i]->vel());
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  if (mp_->verbose) {
    std::cout << "Tracking Time Cost (ms): " << compTime << std::endl;
  }
}

void MappingRos::ekfPredictCallback(const ros::TimerEvent& e) {
  if (trackers.size() == 0)
    return;

  for (size_t i = 0; i < trackers.size(); ++i) {
    const double dt = std::max(mp_->ekf_predict_period, (e.current_real - e.last_real).toSec());
    trackers[i]->setDt(dt);
    trackers[i]->predict();
    if (mp_->horizontal_motion_prior) {
      trackers[i]->constrainVerticalVelocity(mp_->vertical_velocity_scale, mp_->max_vertical_velocity);
    }
  }
}

void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
  if (Remaining_Points->points.size() == 0)
    return;
  pcl::PointIndices::Ptr dynamic_indices(new pcl::PointIndices());
  for (const auto& indices : deleted_indices) {
    dynamic_indices->indices.insert(dynamic_indices->indices.end(), indices.indices.begin(), indices.indices.end());
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  Points static_pts;
  extract.setInputCloud(Remaining_Points);
  extract.setIndices(dynamic_indices);
  extract.setNegative(true);
  extract.filter(static_pts);
  static_pts.width = static_pts.points.size();
  static_pts.height = 1;
  static_pts.is_dense = true;

  sensor_msgs::PointCloud2 map_static;
  pcl::toROSMsg(static_pts, map_static);
  map_static.header.frame_id = mp_->frame_id;
  map_static.header.stamp = ros::Time::now();
  staticMapPub.publish(map_static);
}


double MappingRos::iou(ObjectState state1, ObjectState state2) {
  double distance = (state1.position - state2.position).norm();
  double iou = atan(distance)*2/M_PI;
  return iou;
}

void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> pairs) {
  visualization_msgs::MarkerArray poses;
  visualization_msgs::MarkerArray boxes;

  for (auto pair : pairs) {
    int detectionIndex = pair.first;
    int trackerIndex = pair.second;
    pcl::PointXYZ minPt, maxPt;
    // if (detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2 > 1.7 ||
    //     detections[detectionIndex].position[1] < -2.0 ||
    //     detections[detectionIndex].position[2] > 1.8)
    //   continue;
    minPt.x = detections[detectionIndex].position[0] - detections[detectionIndex].size[0]/2;
    minPt.y = detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2;
    minPt.z = detections[detectionIndex].position[2] - detections[detectionIndex].size[2]/2;
    maxPt.x = detections[detectionIndex].position[0] + detections[detectionIndex].size[0]/2;
    maxPt.y = detections[detectionIndex].position[1] + detections[detectionIndex].size[1]/2;
    maxPt.z = detections[detectionIndex].position[2] + detections[detectionIndex].size[2]/2;
	    visualization_msgs::Marker edgeMarker;
	    edgeMarker.id = detectionIndex;
	    edgeMarker.header.stamp = ros::Time::now();
	    edgeMarker.header.frame_id = mp_->frame_id;
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime = ros::Duration(0.1);
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;
    edgeMarker.scale.x = 0.1;
    geometry_msgs::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker); 

    visualization_msgs::Marker poseMarker;
    ObjectState state;
    state.position = trackers[trackerIndex]->pos();
    state.velocity = trackers[trackerIndex]->vel();
	    poseMarker.id = trackerIndex+100;
	    poseMarker.header.stamp = ros::Time::now();
	    poseMarker.header.frame_id = mp_->frame_id;
    poseMarker.lifetime = ros::Duration(0.1);
    poseMarker.type = visualization_msgs::Marker::ARROW;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;
    poseMarker.scale.y = 0.18;
    poseMarker.scale.z = 0.30;
    poseMarker.pose.orientation.w = 1.0;
    geometry_msgs::Point arrow[2];
	    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];
	    const double vel_norm = std::max(state.velocity.norm(), 1e-3);
	    arrow[1].x = state.position[0] + state.velocity[0]/vel_norm; 
	    arrow[1].y = state.position[1] + state.velocity[1]/vel_norm; 
	    arrow[1].z = state.position[2] + state.velocity[2]/vel_norm;
    // std::cout << "vx:" << state.velocity[0] << std::endl;
    // std::cout << "vy:" << state.velocity[1] << std::endl;
    // std::cout << "vz:" << state.velocity[2] << std::endl;
    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);
    if (state.velocity.norm() > 0.01)
      poses.markers.push_back(poseMarker);
  }
  objectPosePub.publish(poses);
  edgePub.publish(boxes);

}

}
