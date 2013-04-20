#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>


#include "chai3d.h"

namespace chai_tools
{

  tf::Vector3 cVectorToTF(const cVector3d &v);

  tf::Matrix3x3 cMatrixToTF(const cMatrix3d &rot);

  tf::Quaternion cMatrixToTFQuaternion(const cMatrix3d &m);

} // namespace chai_tools


#endif
