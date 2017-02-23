#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
//#include <tf_conversions/tf_eigen.h>

#include <pal_carmen_reader/pal_carmen_reader.h>

using LocalizedRangeScan = std::pair<sensor_msgs::LaserScan, tf::Transform>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_odometry_icp");

  ros::NodeHandle nh("~");

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
  while (read_ok)
  {
    sensor_msgs::LaserScan laser;
    nav_msgs::Odometry odometry;
    geometry_msgs::TransformStamped transform_msg;

    read_ok = carmen_reader_ptr_->readNext("ROBOTLASER1", &laser, &odometry, &transform_msg);

//    std::cout << "size " << laser.ranges.size() << std::endl;
//    for (auto i : laser.ranges)
//      std::cout << " " << i;
//    std::cout << "\n" << std::endl;

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
  }

  std::cout << "Loaded " << localized_scans.size() << " scans." << std::endl;

  if (localized_scans.size()) return EXIT_SUCCESS;

  // Some parameters

  std::size_t max_iteration = (std::size_t)nh.param("registration_max_iteration", 20);
  std::size_t num_reg_scan  = (std::size_t)nh.param("scan_window_size", 1);

  std::size_t matcher_type  = (std::size_t)nh.param("matcher_type", 0);

  std::vector<int> prunner_type_list  = nh.param("prunner_type_list", std::vector<int>{1});

  std::size_t predictor_type  = (std::size_t)nh.param("predictor_type", 1);

  std::size_t objective_type  = (std::size_t)nh.param("objective_type", 0);

  std::size_t control_type  = (std::size_t)nh.param("control_type", 0);

  ros::Publisher publisher_corrected_poses;
  geometry_msgs::PoseArray corrected_poses; corrected_poses.header.frame_id = "map";
  publisher_corrected_poses = nh.advertise<geometry_msgs::PoseArray>("corrected_poses", 1);

  // Get initial pose
  const tf::Transform init_pose_inv = localized_scans.begin()->second.inverse();

//  lsr::geometry::RigidTransform3s tf_prev = lsr::geometry::Idendity3ds;
//  lsr::geometry::RigidTransform3s tf_est  = lsr::geometry::Idendity3ds;

  //nav_msgs::Odometry odometry_w;
  nav_msgs::Path path;
  path.header.frame_id = "map";

//  lsr::geometry::RigidTransform3s tf_rel_est = lsr::geometry::Idendity3ds;

//  pal_robot_tools::TimerData timer("toto");

  tf::Transform global_pose, relative_pose;
  geometry_msgs::Transform global_pose_msg;

  std::size_t scan_id = 0;
  // Pass all scans
  for (const auto &p : localized_scans)
  {
    if (!ros::ok()) break;

    std::cout << "\n\n ------------------------------------"
              << " \n\tProcessing scan " << scan_id++
              << "\n ------------------------------------\n" << std::endl;

    const sensor_msgs::LaserScan& ros_scan = p.first;

    // Get tf with respect to initial pose being I &
    // convert to lsr format.
//    lsr::geometry::RigidTransform3s tf_w =
//        conversion::fromRos(init_pose_inv * p.second);

    // Get current tf with respect to previous one
//    const lsr::geometry::RigidTransform3s tf_rel = tf_prev.inverse() * tf_w;

//    timer.startTimer();

    /* ----------- Everything Takes Place Here ---------- */

    // bool processed = @todo.process(ros_scan, global_pose, relative_pose);

    /* -------------------------------------------------- */

//   timer.stopTimer();

//    tf_est = @todo.getEstimatedPose();

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

//    tf_prev = tf_w;

    //std::cout << "\n\n------Publishing Logs------\n\n" << std::endl;

    tf::transformTFToMsg(global_pose, global_pose_msg);
//    corrected_poses.poses.push_back(global_pose_msg);
    publisher_corrected_poses.publish(corrected_poses);

  }

//  std::cout << "Took in average : " << timer.getAverageCycleTime() << std::endl;

  return EXIT_SUCCESS;
}
