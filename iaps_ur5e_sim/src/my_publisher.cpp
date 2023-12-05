#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>


 geometry_msgs::PoseStamped get_pose_msg(float x, float y, float z, float q1, float q2, float q3, float q4, std::string frame)
 { 
    geometry_msgs::PoseStamped posestamped_;
    geometry_msgs::Point position_;
    geometry_msgs::Quaternion orientation_;
    std_msgs::Header header_;
    std_msgs::String frame_id_;
    
    frame_id_.data = frame;

    position_.x = x;
    position_.y = y;
    position_.z = z;

    orientation_.x = q1;
    orientation_.y = q2;
    orientation_.z = q3;
    orientation_.w = q4;

    posestamped_.pose.position = position_;
    posestamped_.pose.orientation = orientation_;
    posestamped_.header.frame_id = frame_id_.data;
    
    return posestamped_;
 };

 void print_screen_info(geometry_msgs::PoseStamped posestamped)
 {
    ROS_INFO("moving to \n position: [%f, %f, %f] \n orientation: [%f, %f, %f, %f]", 
    posestamped.pose.position.x, posestamped.pose.position.y, posestamped.pose.position.z,
    posestamped.pose.orientation.x, posestamped.pose.orientation.y, posestamped.pose.orientation.z, posestamped.pose.orientation.w);
 };
 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_pose_publisher");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("target_frame", 1000);

  geometry_msgs::PoseStamped posestamped_1 = get_pose_msg(0.4, -0.1, 0.4, 0.0, 0.0, 0.0, 1.0, "base_link");
  geometry_msgs::PoseStamped posestamped_2 = get_pose_msg(0.4, -0.1, 0.6, 0.0, 0.0, 0.0, 1.0, "base_link");
  geometry_msgs::PoseStamped posestamped_3 = get_pose_msg(0.4, 0.3, 0.6, 0.0, 0.0, 0.0, 1.0, "base_link");
  geometry_msgs::PoseStamped posestamped_4 = get_pose_msg(0.4, 0.3, 0.4, 0.0, 0.0, 0.0, 1.0, "base_link");

  std::array<geometry_msgs::PoseStamped,4> traj = {posestamped_1, posestamped_2, posestamped_3, posestamped_4}; 
  int i = 0;

  while (ros::ok())
  {
      geometry_msgs::PoseStamped posestamped = traj[i];
      print_screen_info(posestamped);
      pose_pub.publish(posestamped);
      ros::spinOnce();

      ros::Duration(3.0).sleep();
      i++;
      i %= traj.size();    

  }


  return 0;
}
