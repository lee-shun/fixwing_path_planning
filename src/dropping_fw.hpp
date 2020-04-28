#ifndef _DROPPING_FW_HPP_
#define _DROPPING_FW_HPP_
/*************************************************************************************
*
*Author:lee-shun & Hao Sun
*
*Date: 
*
*Description:
*去年的投弹程序的复现
*
*
*************************************************************************************/

#include <iomanip>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include "lib/syslib.hpp"
#include "lib/vector.hpp"
#include "lib/mathlib.hpp"

using namespace std;
mavros_msgs::State current_state;
mavros_msgs::WaypointList current_waypoints;
mavros_msgs::WaypointReached reached_waypoints;
sensor_msgs::NavSatFix current_gps;
mavros_msgs::WaypointList waypoint_list;
class DROPPING_FW {

public:
  void run();
  void set_planeID(int id);

private:
  int planeID{1};
  string uavID{"uav1/"};

  ros::NodeHandle nh;
  ros::Time begin_time;
  float get_ros_time(ros::Time begin);

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient waypoint_setcurrent_client;
  ros::ServiceClient waypoint_pull_client;
  ros::ServiceClient waypoint_push_client;
  ros::ServiceClient waypoint_clear_client;
  void ros_sub_pub();

  bool reached_waypoint(const struct _s_waypoint &waypoint);
  void push_waypoints_to_px4(int size, mavros_msgs::Waypoint *points);
  void clear_waypoint();

  mavros_msgs::Waypoint waypoint[100];/* 即将要发给px4的航点队列 */
  void plan_waypoint(int task_stage);
};
#endif
