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
  mavros_msgs::State current_state;
  mavros_msgs::WaypointList current_waypoints;
  mavros_msgs::WaypointReached reached_waypoints;
  sensor_msgs::NavSatFix current_gps;
  mavros_msgs::WaypointList waypoint_list;
  void ros_sub_pub();

  bool reached_waypoint(const struct _s_waypoint &waypoint);
  void push_waypoints_to_px4(int size, mavros_msgs::Waypoint *points);
  void clear_waypoint();

  mavros_msgs::Waypoint waypoint[100];/* 即将要发给px4的航点队列 */
  void plan_waypoint(int task_stage);
  
  const double eps = 1e-9;
  int r = 30; //盘旋半径
  double wc;
  double jc;
  double jing0, wei0;
  double xc, yc, goal_x, goal_y;
  double x, y;
  class Point;
  typedef Point Vec;
  const double g = 9.80665;  //重力加速度
  const float pi = 3.141593;
  
  struct Point {
	double x, y;
	Point(double _x = 0, double _y = 0) :x(_x), y(_y) {}
	//向量与常数 注意常数要放在后面
	Vec operator*(double p) { return Vec(x*p, y*p); }
	Vec operator/(double p) { return Vec(x / p, y / p); }
	Vec operator-(Vec obj) { return Vec(x - obj.x, y - obj.y); }
	Vec operator+(Vec obj) { return Vec(x + obj.x, y + obj.y); }
	//点积
	double operator*(Vec obj) { return x*obj.x + y*obj.y; }
	//叉积
	double operator^(Vec obj) { return x*obj.y - y*obj.x; }
	//两个向量的夹角 A*B=|A|*|B|*cos(th)
	double Angle(Vec B) { return acos((*this)*B / (*this).len() / B.len()); }
	//两条向量平行四边形的面积
	double Area(Vec B) { return fabs((*this) ^ B); }
	//向量旋转
	//旋转公式
	//  Nx     (cos  -sin) x
	//  Ny     (sin   cos) y
	Vec Rotate(double rad) { return Vec(x*cos(rad) - y*sin(rad), x*sin(rad) + y*cos(rad)); }
	//返回向量的法向量，即旋转pi/2
	Vec Normal() { return Vec(-y, x); }
	//返回向量的长度,或者点距离原点的距离
	double len() { return hypot(x, y); }
	double len2() { return x*x + y*y; }
	//返回两点之间的距离
	double dis(Point obj) { return hypot(x - obj.x, y - obj.y); } //hypot 给定直角三角形的两条直角边，返回斜边边长
	//向量的极角 atan2(y,x)
	bool operator==(Point obj) { return dcmp(x - obj.x) == 0 && dcmp(y - obj.y) == 0; }
	bool operator<(Point obj) { return x<obj.x || (x == obj.x&&y<obj.y); }
};
Point va[10], vb[10];



struct Line {
	Point A, B;
	double l;
	bool operator<(const Line& b)const {
		if (dcmp(A.x - b.A.x) == 0)return A.y<b.A.y;
		return A.x<b.A.x;
	}
}L[10];


struct Circle {
	Point c; double r;
	Circle(Point c, double r) :c(c), r(r) {}
	Point getpoint(double a) { return Point(c.x + cos(a)*r, c.y + sin(a)*r); }
};
};
#endif
