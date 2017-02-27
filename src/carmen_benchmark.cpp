#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

#include <pal_carmen_reader/pal_carmen_reader.h>
#include <laser_odometry_core/laser_odometry.h>

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

  laser_odometry::LaserOdometryPtr laser_odom_ptr = std::make_shared<laser_odometry::LaserOdometry>(laser_odometry_type);

  bool publish_tf = true;
  nh.param("publish_tf", publish_tf, publish_tf);
  laser_odom_ptr->broadcastTf(publish_tf);

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

  do
  {
    sensor_msgs::LaserScan laser;
    nav_msgs::Odometry odometry;
    geometry_msgs::TransformStamped transform_msg;

    read_ok = carmen_reader_ptr_->readNext("ROBOTLASER1", &laser, &odometry, &transform_msg);

    if (read_ok)
    {
      tf::Vector3 basis(odometry.pose.pose.position.x,
                        odometry.pose.pose.position.y,
                        odometry.pose.pose.position.z);

      tf::Transform transform;
      transform.setOrigin(basis);

      tf::Quaternion q;
      tf::quaternionMsgToTF(odometry.pose.pose.orientation, q);
      transform.setRotation(q);

      localized_scans.emplace_back(laser, transform);

      if (!laser_transform_init)
      {
        //std::cout << "laser " << laser << std::endl;

        tf::Transform laser_transform_tf;
        tf::transformMsgToTF(transform_msg.transform, laser_transform_tf);

        auto rel = transform.inverseTimes(laser_transform_tf);

        tf::transformTFToMsg(rel, laser_transform.transform);

        laser_transform_init = true;
      }
    }
  } while (read_ok);

  std::cout << "Loaded " << localized_scans.size() << " scans." << std::endl;

  if (localized_scans.empty()) return EXIT_SUCCESS;

  geometry_msgs::PoseArray corrected_poses, estimated_poses;
//  corrected_poses.header.frame_id = "map";
//  estimated_poses.header.frame_id = "map";

  ros::Publisher publisher_corrected_poses, publisher_estimated_poses;
  publisher_corrected_poses = nh.advertise<geometry_msgs::PoseArray>("corrected_poses", 1);
  publisher_estimated_poses = nh.advertise<geometry_msgs::PoseArray>("estimated_poses", 1);

  // Get initial pose
  const tf::Transform origin = localized_scans.begin()->second;
  const tf::Transform origin_inv = origin.inverse();

  //laser_odom_ptr->setOrigin(origin);

  tf::Transform tf_prev; tf_prev.setIdentity();
  tf::Transform tf_est;  tf_est.setIdentity();

  //nav_msgs::Odometry odometry_w;
  nav_msgs::Path path;
  path.header.frame_id = "map";

//  lsr::geometry::RigidTransform3s tf_rel_est = lsr::geometry::Idendity3ds;

//  pal_robot_tools::TimerData timer("toto");

  tf::Transform corrected_pose;

  std::size_t scan_id = 0;
  // Pass all scans
  for (const auto &p : localized_scans)
  {
    if (!ros::ok())
    {
      std::cerr << "Ros down, exit." << std::endl;
      break;
    }

    std::cout << "\n\n ------------------------------------"
              << " \n\tProcessing scan " << scan_id++
              << "\n ------------------------------------\n" << std::endl;

//    const sensor_msgs::LaserScan& ros_scan = p.first;
    const sensor_msgs::LaserScanPtr ros_scan = boost::make_shared<sensor_msgs::LaserScan>(p.first);

    // Get tf with respect to initial pose being I &
    // convert to lsr format.
    corrected_pose = origin_inv * p.second;

    // Get current tf with respect to previous one
//    const tf::Transform corrected_relative_pose = tf_prev.inverse() * corrected_pose;

//    timer.startTimer();

    /* ----------- Everything Takes Place Here ---------- */

    geometry_msgs::PoseWithCovarianceStampedPtr estimated_pose;
    estimated_pose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

    /*bool processed =*/ laser_odom_ptr->process(ros_scan, estimated_pose);

    //  @todo.process(ros_scan, global_pose, relative_pose);

    /* -------------------------------------------------- */

//   timer.stopTimer();

//    tf_est = @todo.getEstimatedPose();

    geometry_msgs::Transform corrected_pose_msg;
    tf::transformTFToMsg(corrected_pose, corrected_pose_msg);

    std::cout << "corrected_pose\n"  << corrected_pose_msg << std::endl;
    std::cout << "estimated_pose\n"  << estimated_pose->pose.pose << std::endl;

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

//    tf_prev = corrected_pose;

    //std::cout << "\n\n------Publishing Logs------\n\n" << std::endl;



    geometry_msgs::Pose corrected_pose_msg2;
    corrected_pose_msg2.position.x = corrected_pose_msg.translation.x;
    corrected_pose_msg2.position.y = corrected_pose_msg.translation.y;
    corrected_pose_msg2.position.z = corrected_pose_msg.translation.z;

    corrected_pose_msg2.orientation = corrected_pose_msg.rotation;

    corrected_poses.poses.push_back(corrected_pose_msg2);
    estimated_poses.poses.push_back(estimated_pose->pose.pose);

    corrected_poses.header.frame_id = estimated_pose->header.frame_id;
    estimated_poses.header.frame_id = estimated_pose->header.frame_id;

    publisher_estimated_poses.publish(estimated_poses);
    publisher_corrected_poses.publish(corrected_poses);

    ros::spinOnce();
    sleep_duration.sleep();

//    if (scan_id > 70) break;
  }

//  std::cout << "Took in average : " << timer.getAverageCycleTime() << std::endl;

  std::cout << "Done." << std::endl;

  return EXIT_SUCCESS;
}
