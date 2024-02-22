#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  ros::init(argc, argv, "localizer_node");
  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  map_ptr = std::make_shared<Map>();

  //
  /**
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   *
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  ros::Subscriber sub_map =
      nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, callback_map);

  ros::Subscriber sub_initialpose =
      nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "/initialpose", 10, callback_initialpose);

  ros::Subscriber sub_scan =
      nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, callback_scan);

  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.
  if (!map_ptr->initialized()) {
    map_ptr->loadOccupancyGrid(msg_);
    if (map_ptr->initialized()) {
      ROS_INFO("Map Received.");
      localizer.setMap(map_ptr);
    }
  }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   */
  Eigen::Isometry2f initial_pose;
  pose2isometry(msg_->pose.pose, initial_pose);
  localizer.setInitialPose(initial_pose);
}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  Localizer2D::ContainerType scan;
  scan2eigen(msg_, scan);
  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  localizer.setLaserParams(msg_->range_min, msg_->range_max, msg_->angle_min,
                           msg_->angle_max, msg_->angle_increment);
  localizer.process(scan);

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_msg;
  isometry2transformStamped(localizer.X(), tf_msg, FRAME_WORLD, FRAME_LASER,
                            msg_->header.stamp);
  br.sendTransform(tf_msg);

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  nav_msgs::Odometry odom_msg;
  transformStamped2odometry(tf_msg, odom_msg);
  pub_odom.publish(odom_msg);

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}