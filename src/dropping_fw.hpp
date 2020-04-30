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

#include "lib/circle.hpp"
#include "lib/line.hpp"
#include "lib/mathlib.hpp"
#include "lib/syslib.hpp"
#include "lib/vector.hpp"

using namespace std;

class DROPPING_FW {

public:
  void run();
  void set_planeID(int id);

protected:
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

  ros::Subscriber state_sub;
  ros::Subscriber waypoints_sub;
  ros::Subscriber waypointsreach_sub;
  ros::Subscriber currentgps_sub;

  mavros_msgs::State current_state;
  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
  }
  mavros_msgs::WaypointList current_waypoints;
  void get_waypoints(const mavros_msgs::WaypointList::ConstPtr &msg) {
    current_waypoints = *msg;
  }
  mavros_msgs::WaypointReached reached_waypoints;
  void waypoints_reached(const mavros_msgs::WaypointReached::ConstPtr &msg) {
    reached_waypoints = *msg;
  }
  sensor_msgs::NavSatFix current_gps;
  void current_gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
  }
  mavros_msgs::WaypointList waypoint_list;
  void ros_sub_pub();

  bool reached_waypoint(const struct _s_waypoint &waypoint);
  void push_waypoints_to_px4(int size, mavros_msgs::Waypoint *points);
  void clear_waypoint();

  mavros_msgs::Waypoint waypoint[100]; /* 即将要发给px4的航点队列 */
  void plan_waypoint(int task_stage);

  int r = 30; //盘旋半径
  double wc;
  double jc;
  double jing0, wei0;
  double xc, yc, goal_x, goal_y;
  double x, y;
  const double g = 9.80665; //重力加速度
  const float pi = 3.141593;
  Point va[10], vb[10];
  Line L[10];

  double *point_tangency(double g[2]);
};
#endif
