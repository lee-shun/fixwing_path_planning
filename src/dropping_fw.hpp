#ifndef _DROPPING_FW_HPP_
#define _DROPPING_FW_HPP_

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

class DROPPING_FW {

public:
  struct _s_waypoint {
    int type{0};

    double lat{0.0};

    double lon{0.0};

    double alt{0.0};

    float vel_sp{0.0};

    float loit_radius{0.0};
  };

  void run();
  void set_planeID(int id);

private:
  int planeID{1};
  string uavID{"uav1/"};

  ros::NodeHandle nh;
  ros::Time begin_time;
  float get_ros_time(ros::Time begin);

  void ros_sub_pub();
  bool reached_waypoint(const struct _s_waypoint &waypoint);
  void push_waypoints_to_px4(const struct _s_waypoint &waypoint);

  struct _s_waypoint WayPoints[50];
  void plan_waypoint(int task_stage);
};
#endif
