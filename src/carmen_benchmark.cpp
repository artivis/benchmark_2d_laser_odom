#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

#include <pal_carmen_reader/pal_carmen_reader.h>
#include <laser_odometry_core/laser_odometry_core.h>
#include <laser_odometry_core/laser_odometry_instantiater.h>
#include <laser_odometry_core/laser_odometry_utils.h>

#include <benchmark_2d_laser_odom/timer.h>

using LocalizedRangeScan = std::pair<sensor_msgs::LaserScan, tf::Transform>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_odometry_icp");

  ros::NodeHandle nh("~");

  std::string laser_odometry_type;
  if(!nh.getParam("laser_odometry_type", laser_odometry_type))
  {
    ROS_ERROR("No laser odometry type specified: use param _ilaser_odometry_type:=[type]");
    return EXIT_FAILURE;
  }

  laser_odometry::LaserOdometryPtr laser_odom_ptr =
      laser_odometry::make_laser_odometry(laser_odometry_type);

  bool publish_tf = true;
  nh.param("publish_tf", publish_tf, publish_tf);
//  laser_odom_ptr->broadcastTf(publish_tf);

  bool use_odom_prior = true;
  nh.param("use_odom_prior", use_odom_prior, use_odom_prior);

  double sleep(0.005);
  nh.param("visualization_sleep", sleep, sleep);
  ros::Duration sleep_duration(sleep);

  std::string scans_file;
  if(!nh.getParam("input_file", scans_file))
  {
    ROS_ERROR("No input file specified: use param _input_file:=[scans]");
    return EXIT_FAILURE;
  }

  std::vector<LocalizedRangeScan> localized_scans;

  std::shared_ptr<pal::CarmenReader> carmen_reader_ptr_;
  carmen_reader_ptr_ = std::make_shared<pal::CarmenReader>(scans_file);

  bool read_ok = true;

  bool laser_transform_init = false;
  geometry_msgs::TransformStamped laser_transform;

  sensor_msgs::LaserScanPtr laser = boost::make_shared<sensor_msgs::LaserScan>();
  nav_msgs::OdometryPtr odometry  = boost::make_shared<nav_msgs::Odometry>();
  geometry_msgs::TransformStampedPtr transform_msg = boost::make_shared<geometry_msgs::TransformStamped>();

  do
  {
    read_ok = carmen_reader_ptr_->readNext("ROBOTLASER1", laser, odometry, transform_msg);

    if (read_ok)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(odometry->pose.pose.orientation, q);

      tf::Transform transform(q, tf::Vector3{odometry->pose.pose.position.x,
                                             odometry->pose.pose.position.y,
                                             0});

      localized_scans.emplace_back(*laser, transform);

      // Retrieve transform laser to base
      if (!laser_transform_init)
      {
        //std::cout << "laser " << laser << std::endl;

        tf::Transform laser_transform_tf;
        tf::transformMsgToTF(transform_msg->transform, laser_transform_tf);

        auto rel = transform.inverseTimes(laser_transform_tf);

        tf::transformTFToMsg(rel, laser_transform.transform);

        laser_transform_init = true;
      }
    }
  } while (read_ok);

  std::cout << "Loaded " << localized_scans.size() << " scans." << std::endl;

  if (localized_scans.empty()) return EXIT_SUCCESS;

  geometry_msgs::PoseArray corrected_poses, estimated_poses;

  ros::Publisher publisher_corrected_poses, publisher_estimated_poses;
  publisher_corrected_poses = nh.advertise<geometry_msgs::PoseArray>("corrected_poses", 1);
  publisher_estimated_poses = nh.advertise<geometry_msgs::PoseArray>("estimated_poses", 1);

  // Get origin
  const tf::Transform origin = localized_scans.begin()->second;
  const tf::Transform origin_inv = origin.inverse();

  //laser_odom_ptr->setOrigin(origin);

  tf::Transform tf_prev = tf::Transform::getIdentity();
//  tf::Transform tf_est  = tf::Transform::getIdentity();

  pal::TimerU timer;

//  lsr::geometry::RigidTransform3s tf_rel_est = lsr::geometry::Idendity3ds;
  tf::Transform corrected_pose = tf::Transform::getIdentity();

  std::size_t scan_id = 0;
  // Pass all scans
  for (const auto &p : localized_scans)
  {
    if (!ros::ok())
    {
      std::cerr << "Ros down, exit." << std::endl;
      break;
    }

    std::cout << "\n\n------------------------------------"
              << "\n\t Processing scan "      << scan_id++
              << "\n------------------------------------\n" << std::endl;

    const sensor_msgs::LaserScanPtr ros_scan = boost::make_shared<sensor_msgs::LaserScan>(p.first);

    // Get tf with respect to initial pose being I
    corrected_pose = origin_inv * p.second;

    // Get current tf with respect to previous one
    tf::Transform corrected_relative_pose = tf_prev.inverse() * corrected_pose;

    /* ----------- Everything Takes Place Here ---------- */

//    print(corrected_relative_pose, "corrected_relative_pose : ");
    std::cout << "corrected_relative_pose : "
              << corrected_relative_pose.getOrigin().getX()
              << " " << corrected_relative_pose.getOrigin().getX()
              << " " << tf::getYaw(corrected_relative_pose.getRotation()) << std::endl;

    /// @todo
    if (use_odom_prior) laser_odom_ptr->setInitialGuess(corrected_relative_pose);

    nav_msgs::OdometryPtr estimated_pose;
    estimated_pose = boost::make_shared<nav_msgs::Odometry>();

    /*bool processed =*/ laser_odom_ptr->process(ros_scan, estimated_pose);

    //std::cout << "Process done." << std::endl;

    //  @todo.process(ros_scan, global_pose, relative_pose);

    /* -------------------------------------------------- */

   timer.tic();

//    tf_est = @todo.getEstimatedPose();

    geometry_msgs::Transform corrected_pose_msg;
    tf::transformTFToMsg(corrected_pose, corrected_pose_msg);

    std::cout << "corrected_pose : "
              << corrected_pose_msg.translation.x << " "
              << corrected_pose_msg.translation.y << " "
              << tf::getYaw(corrected_pose_msg.rotation) << std::endl;
    std::cout << "estimated_pose : "
              << estimated_pose->pose.pose.position.x << " "
              << estimated_pose->pose.pose.position.y << " "
              << tf::getYaw(estimated_pose->pose.pose.orientation) << std::endl;

//    std::cout << "tf\n"  << tf_w.matrix() << std::endl;
//    std::cout << "tf_rel "
//              << " yaw " << extractYaw(tf_rel)
//              << " t " << tf_rel.translation().norm() << "\n"
//              << tf_rel.matrix() << std::endl;
//    std::cout << "tf estimated\n" << tf_est.matrix() << std::endl;
//    std::cout << "tf_rel estimated "
//              << " yaw " << extractYaw(tf_rel_est)
//              << " t " << tf_rel_est.translation().norm() << "\n"
//              << tf_rel_est.matrix() << std::endl;

    tf_prev = corrected_pose;

    //std::cout << "\n\n------Publishing------\n\n" << std::endl;
    bool sleep_pub = false;
    if (publisher_corrected_poses.getNumSubscribers()>0)
    {
      geometry_msgs::Pose corrected_pose_msg2;
      corrected_pose_msg2.position.x = corrected_pose_msg.translation.x;
      corrected_pose_msg2.position.y = corrected_pose_msg.translation.y;
      corrected_pose_msg2.position.z = corrected_pose_msg.translation.z;
      corrected_pose_msg2.orientation = corrected_pose_msg.rotation;

      corrected_poses.poses.push_back(corrected_pose_msg2);
      corrected_poses.header.frame_id = estimated_pose->header.frame_id;
      publisher_corrected_poses.publish(corrected_poses);

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

//    if (scan_id > 70) break;
  }

  std::cout << "Took in average : " << timer.avg_tic() << " us." << std::endl;

  std::cout << "Done." << std::endl;

  return EXIT_SUCCESS;
}
