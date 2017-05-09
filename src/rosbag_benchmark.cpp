#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

#include <pal_rosbag_reader/pal_rosbag_reader.h>
#include <laser_odometry_core/laser_odometry_core.h>
#include <laser_odometry_core/laser_odometry_instantiater.h>
#include <laser_odometry_core/laser_odometry_utils.h>

#include <benchmark_2d_laser_odom/timer.h>

std::vector<sensor_msgs::LaserScanPtr> laser_scan_msgs;
std::vector<nav_msgs::Odometry> odometry_msgs;
std::vector<nav_msgs::Odometry> odometry_gt_msgs;

using OdometryIt = std::vector<nav_msgs::Odometry>::iterator;

bool getBaseToLaser(const tf::tfMessagePtr tf_msg, tf::Transform& tf)
{
  return laser_odometry::utils::getTf(tf_msg, "base_link", "base_laser_link", tf);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_odometry_icp");

  ros::NodeHandle nh("~");

  std::string laser_odometry_type;
  if(!nh.getParam("laser_odometry_type", laser_odometry_type))
  {
    ROS_ERROR("No laser odometry type specified: use param _laser_odometry_type:=[type]");
    return EXIT_FAILURE;
  }

  laser_odometry::LaserOdometryPtr laser_odom_ptr =
      laser_odometry::make_laser_odometry(laser_odometry_type);

  bool publish_tf = true;
  nh.param("publish_tf", publish_tf, publish_tf);
//  laser_odom_ptr->broadcastTf(publish_tf);

  bool use_odom_prior = false;
  nh.param("use_odom_prior", use_odom_prior, use_odom_prior);

  double sleep(0.005);
  nh.param("visualization_sleep", sleep, sleep);
  ros::Duration sleep_duration(sleep);

  std::string rosbag_file;
  if(!nh.getParam("input_file", rosbag_file))
  {
    ROS_ERROR("No input file specified: use param _input_file:=[scans]");
    return EXIT_FAILURE;
  }

  std::string laser_scan_topic;
  if(!nh.getParam("laser_scan_topic", laser_scan_topic))
  {
    ROS_ERROR("No input file specified: use param _laser_scan_topic:=[topic]");
    return EXIT_FAILURE;
  }

  std::string odometry_topic;
  if(!nh.getParam("odometry_topic", odometry_topic))
  {
    ROS_ERROR("No input file specified: use param _odometry_topic:=[topic]");
    return EXIT_FAILURE;
  }

  std::string odometry_gt_topic;
  if(!nh.getParam("odometry_gt_topic", odometry_gt_topic))
  {
    ROS_ERROR("No input file specified: use param _odometry_gt_topic:=[topic]");
    return EXIT_FAILURE;
  }

  std::shared_ptr<pal::RosbagReader> rosbag_reader_ptr_;
  rosbag_reader_ptr_ = std::make_shared<pal::RosbagReader>(rosbag_file);

  bool read_ok = true;

  sensor_msgs::LaserScanPtr laser = boost::make_shared<sensor_msgs::LaserScan>();
  nav_msgs::OdometryPtr odometry  = boost::make_shared<nav_msgs::Odometry>();

  // Load all scans
  do
  {
    read_ok = rosbag_reader_ptr_->readNext(laser_scan_topic, laser);

    if (read_ok) laser_scan_msgs.emplace_back(laser);

  } while (read_ok);

  std::cout << "Loaded " << laser_scan_msgs.size() << " scans." << std::endl;

  // Load all odometry
  do
  {
    read_ok = rosbag_reader_ptr_->readNext(odometry_topic, odometry);

    if (read_ok) odometry_msgs.emplace_back(*odometry);

  } while (read_ok);

  // Load all odometry groundtruth
  do
  {
    read_ok = rosbag_reader_ptr_->readNext(odometry_gt_topic, odometry);

    if (read_ok) odometry_gt_msgs.emplace_back(*odometry);

  } while (read_ok);

  std::cout << "Loaded " << odometry_gt_msgs.size() << " odom groundtruth." << std::endl;

  tf::Transform base_to_laser;
  // Retrieve tf laser to base
  do
  {
    tf::tfMessagePtr tf_msg = boost::make_shared<tf::tfMessage>();
    read_ok = rosbag_reader_ptr_->readNext("/tf", tf_msg);

    if (read_ok)
    {
      if (getBaseToLaser(tf_msg, base_to_laser))
      {
        //laser_odometry::utils::print(base_to_laser, "laser_link ");
        read_ok = false;
      }
    }

  } while (read_ok);

  if (laser_scan_msgs.empty())  return EXIT_SUCCESS;
  if (odometry_msgs.empty())    return EXIT_SUCCESS;
  if (odometry_gt_msgs.empty()) return EXIT_SUCCESS;

  laser_odom_ptr->setLaserPose(base_to_laser);

  geometry_msgs::PoseArray corrected_poses, estimated_poses;
  ros::Publisher publisher_corrected_poses, publisher_estimated_poses;
  publisher_corrected_poses = nh.advertise<geometry_msgs::PoseArray>("/benchmark/corrected_poses", 1);
  publisher_estimated_poses = nh.advertise<geometry_msgs::PoseArray>("/benchmark/estimated_poses", 1);

  // Get initial pose
  const tf::Transform origin = tf::Transform::getIdentity();//localized_scans.begin()->second;
  const tf::Transform origin_inv = origin.inverse();

  //laser_odom_ptr->setOrigin(origin);

  tf::Transform tf_prev = tf::Transform::getIdentity();
  tf::Transform tf_est  = tf::Transform::getIdentity();

  pal::TimerU timer;

  tf::Transform corrected_pose;

  tf::Transform initial_guess = tf::Transform::getIdentity();

  OdometryIt odometry_gt_it = odometry_gt_msgs.begin();

  std::size_t scan_id = 0;
  // Pass all scans
  for (const auto ros_scan : laser_scan_msgs)
  {
    if (!ros::ok())
    {
      std::cerr << "Ros down, exit." << std::endl;
      break;
    }

    std::cout << "\n\n------------------------------------"
              << "\n\tProcessing scan " << scan_id++
              << "\n------------------------------------\n" << std::endl;

    // Get tf with respect to initial pose being I &
    // convert to lsr format.
    corrected_pose = origin_inv /** p.second*/;

    // Get current tf with respect to previous one
//    const tf::Transform corrected_relative_pose = tf_prev.inverse() * corrected_pose;

    /* ----------- Everything Takes Place Here ---------- */

    nav_msgs::OdometryPtr estimated_pose;
    estimated_pose = boost::make_shared<nav_msgs::Odometry>();

    /// @todo
    if (use_odom_prior) laser_odom_ptr->setInitialGuess(initial_guess);

    /*bool processed =*/ laser_odom_ptr->process(ros_scan, estimated_pose);

    //  @todo.process(ros_scan, global_pose, relative_pose);

    /* -------------------------------------------------- */

   timer.tic();

//    tf_est = @todo.getEstimatedPose();

//   geometry_msgs::Transform corrected_pose_msg;
//   tf::transformTFToMsg(corrected_pose, corrected_pose_msg);

//    std::cout << "corrected_pose : "
//              << corrected_pose_msg.translation.x << " "
//              << corrected_pose_msg.translation.y << " "
//              << tf::getYaw(corrected_pose_msg.rotation) << std::endl;
    std::cout << "estimated_pose : "
              << estimated_pose->pose.pose.position.x << " "
              << estimated_pose->pose.pose.position.y << " "
              << tf::getYaw(estimated_pose->pose.pose.orientation) << std::endl;

//    tf_prev = corrected_pose;

    //std::cout << "\n\n------Publishing------\n\n" << std::endl;
    bool sleep_pub = false;
    if (publisher_corrected_poses.getNumSubscribers()>0)
    {
      while (odometry_gt_it->header.stamp<ros_scan->header.stamp)
      {
        corrected_poses.poses.push_back(odometry_gt_it->pose.pose);
        corrected_poses.header.frame_id = estimated_pose->header.frame_id;
        publisher_corrected_poses.publish(corrected_poses);

        ++odometry_gt_it;
      }

      sleep_pub = true;
    }

    if (publisher_estimated_poses.getNumSubscribers()>0)
    {
      estimated_poses.poses.push_back(estimated_pose->pose.pose);
      estimated_poses.header.frame_id = estimated_pose->header.frame_id;
      publisher_estimated_poses.publish(estimated_poses);

      sleep_pub = true;
    }

    ros::spinOnce();

    if (sleep_pub) sleep_duration.sleep();
  }

  std::cout << "Took in average : " << timer.avg_tic() << " us." << std::endl;

  return EXIT_SUCCESS;
}
