#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>

ros::Publisher sph_pcl_pub;
ros::Publisher occ_srfc_pub;
ros::Publisher rbt_pose_pub;

std::string lidar_frame = "os_sensor";
float oc_srfc_rds = 5.0f;
float org_oc_srfc_rds_viz = 5.0f;
float proj_lfrq = 10.0f;
double t_prev = 0.0;

void print_param()
{
  ROS_INFO("######### lste_oc_srfc: param ########");
  ROS_INFO_STREAM("lidar_frame          : " << lidar_frame);
  ROS_INFO_STREAM("oc_srfc_rds          : " << oc_srfc_rds);
  ROS_INFO_STREAM("org_oc_srfc_rds_viz  : " << org_oc_srfc_rds_viz);
  ROS_INFO_STREAM("proj_lfrq            : " << proj_lfrq);
  ROS_INFO("#####################################");
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& pose_in)
{
  tf::Quaternion q(
      pose_in->pose.pose.orientation.x,
      pose_in->pose.pose.orientation.y,
      pose_in->pose.pose.orientation.z,
      pose_in->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  geometry_msgs::Pose2D odom_out;
  odom_out.x = pose_in->pose.pose.position.x;
  odom_out.y = pose_in->pose.pose.position.y;
  odom_out.theta = static_cast<float>(yaw);
  rbt_pose_pub.publish(odom_out);
}

void pts_cb(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
  const double t_now = pcl_in->header.stamp.toSec();
  if (t_now - t_prev <= (1.0 / proj_lfrq))
  {
    return;
  }
  t_prev = t_now;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pcl_in, *cloud);

  pcl::PointCloud<pcl::PointXYZI> sph_pcl;
  pcl::PointCloud<pcl::PointXYZI> occ_srfc_pcl;

  for (const auto& pt : cloud->points)
  {
    const float dst = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (dst > oc_srfc_rds)
    {
      continue;
    }
    const float th = std::atan2(pt.y, pt.x);
    const float al = std::acos(pt.z / dst);

    pcl::PointXYZI shp_pt;
    shp_pt.x = th;
    shp_pt.y = al;
    shp_pt.z = dst;
    shp_pt.intensity = oc_srfc_rds - dst;
    sph_pcl.push_back(shp_pt);

    pcl::PointXYZI oc_pt;
    oc_pt.x = org_oc_srfc_rds_viz * std::sin(al) * std::cos(th);
    oc_pt.y = org_oc_srfc_rds_viz * std::sin(al) * std::sin(th);
    oc_pt.z = org_oc_srfc_rds_viz * std::cos(al);
    oc_pt.intensity = dst;
    occ_srfc_pcl.push_back(oc_pt);
  }

  sph_pcl.header.frame_id = lidar_frame;
  occ_srfc_pcl.header.frame_id = lidar_frame;

  sensor_msgs::PointCloud2 sph_pcl_msg;
  pcl::toROSMsg(sph_pcl, sph_pcl_msg);
  sph_pcl_msg.header.stamp = pcl_in->header.stamp;
  sph_pcl_pub.publish(sph_pcl_msg);

  sensor_msgs::PointCloud2 occ_msg;
  pcl::toROSMsg(occ_srfc_pcl, occ_msg);
  occ_msg.header.stamp = pcl_in->header.stamp;
  occ_srfc_pub.publish(occ_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oc_srfc_proj");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("lidar_frame", lidar_frame, "os_sensor");
  // Use private parameters so launch-file settings (~oc_srfc_rds, etc.) take effect.
  pnh.param<float>("oc_srfc_rds", oc_srfc_rds, 5.0f);
  pnh.param<float>("org_oc_srfc_rds_viz", org_oc_srfc_rds_viz, 5.0f);
  pnh.param<float>("proj_lfrq", proj_lfrq, 10.0f);

  print_param();

  sph_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("sph_pcl", 1);
  occ_srfc_pub = nh.advertise<sensor_msgs::PointCloud2>("lfrq_org_oc_srfc", 1);
  rbt_pose_pub = nh.advertise<geometry_msgs::Pose2D>("rbt_pose", 1);

  ros::Subscriber pts_sub = nh.subscribe("/os_cloud_node/points", 10, pts_cb);
  ros::Subscriber odom_sub = nh.subscribe("/wheel_odom", 10, odom_cb);

  ros::spin();
  return 0;
}

