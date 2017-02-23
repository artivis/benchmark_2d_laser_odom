#ifndef _BENCHMARK_2D_LASER_ODOM_UTILS_H_
#define _BENCHMARK_2D_LASER_ODOM_UTILS_H_

#include <vector>
#include <pal_carmen_reader/pal_carmen_reader.h>

namespace pal
{
namespace benchmark
{

//  template <>
//  tf::Quaternion toRos(const lsr::geometry::RigidTransform3s& tf_e)
//  {
//    tf::Quaternion tf_quat;
//    tf::quaternionEigenToTF(lsr::geometry::Quaternions(tf_e.rotation()), tf_quat);
//    return tf_quat;
//  }

//  template <>
//  geometry_msgs::Quaternion toRos(const lsr::geometry::RigidTransform3s& tf_e)
//  {
//    tf::Quaternion tf_quat = toRos<tf::Quaternion>(tf_e);
//    geometry_msgs::Quaternion msg_quat;
//    tf::quaternionTFToMsg(tf_quat, msg_quat);
//    return msg_quat;
//  }

//  template <>
//  geometry_msgs::Pose toRos(const lsr::geometry::RigidTransform3s& tf_e)
//  {
//    geometry_msgs::Pose pose;
//    pose.position.x = tf_e.translation()(0);
//    pose.position.y = tf_e.translation()(1);
//    pose.position.z = tf_e.translation()(2);
//    pose.orientation = conversion::toRos<geometry_msgs::Quaternion>(tf_e);
//    return pose;
//  }

} /* namespace benchmark */
} /* namespace pal */

#endif /* _BENCHMARK_2D_LASER_ODOM_UTILS_H_ */

