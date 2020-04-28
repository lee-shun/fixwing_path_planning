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
	
/*
1.精度  要学着用dcmp
2.引用  不能交换两个引用swap（）,引用中间不能变
*/
//三态函数比较;精度问题
int dcmp(double x) {
	if (fabs(x)<eps) return 0;
	return x<0 ? -1 : 1;
}

/*
求圆的公切线
*/
int getTan(Circle A, Circle B, Point*  va, Point* vb) {
	int cnt = 0;
	if (A.r<B.r) { swap(A, B); swap(va, vb); }
	double d = (A.c - B.c).len();
	double rdif = A.r - B.r, rsum = A.r + B.r;
	//内含，没有公切线
	if (dcmp(d - rdif)<0)return 0;
	//内切，有一条公切线
	double base = atan2(B.c.y - A.c.y, B.c.x - A.c.x);
	if (dcmp(d) == 0 && dcmp(A.r - B.r) == 0)return -1;
	if (dcmp(d - rdif) == 0) {
		va[cnt] = A.getpoint(base); vb[cnt] = B.getpoint(base); cnt++;
		return cnt;
	}
	//一定有两条外公切线
	double th = acos((A.r - B.r) / d);
	va[cnt] = A.getpoint(base + th); vb[cnt] = B.getpoint(base + th); cnt++;
	va[cnt] = A.getpoint(base - th); vb[cnt] = B.getpoint(base - th); cnt++;
	//可能有一条公切线
	if (dcmp(d - rsum) == 0) {
		va[cnt] = A.getpoint(base); vb[cnt] = B.getpoint(base + pi); cnt++;
	}
	else if (dcmp(d - rsum)>0) {
		double th2 = acos((A.r + B.r) / d);
		va[cnt] = A.getpoint(base + th2); vb[cnt] = B.getpoint(base + th2 + pi); cnt++;
		va[cnt] = A.getpoint(base - th2); vb[cnt] = B.getpoint(base - th2 + pi); cnt++;
	}
	return cnt;
}

//===========================用于计算切点的子函数===================
//输入的数据为纬经
double *point_tangency(double g[2])
{
	//传入目标点
	//===========================圆心坐标为侦查区的中心===================
	double x0 = xc;
	double y0 = yc;

	//==========================================================================

	double r0 = r;
	double y[2];
	double x_1, y_1, x_2, y_2;
	double k1, k2, *res = y;
	double x[2];
	double *add;
	//将用纬度和经度表示的转化为ENU系
	//    cout<<"x[0]="<< x[0] <<endl;
	//    cout<<"wei0="<< wei0 <<endl;

	//纬经高转化为ENU(xyz)
	//    x[1]= (g[0] - wei0) * 111177.0;
	//    x[0]= (g[1] - jing0) * 85155.0;
	//    cout<<"纬经高转化为ENU(xyz)1"<<endl;
	//    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
	//    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;

	//此处代码已经测试成功!!
	add = ll2xy(g[0], g[1]);
	x[0] = add[0];
	x[1] = add[1];

	//    cout<<"纬经高转化为ENU(xyz)2"<<endl;
	//    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
	//    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;


	//找出两个切点（x_1,y_1）(x_2,y_2)                                             ///此处采取的圆的半径为r0。值为r，盘旋半径
	k1 = (y0*x0 + x[1] * x[0] - y0*x[0] - x[1] * x0 + sqrt(r0*r0*(-2 * y0*x[1] - 2 * x0*x[0] + x[1] * x[1] +
		y0*y0 + x0*x0 - r0*r0 + x[0] * x[0]))) / (-r0*r0 + x0*x0 - 2 * x0*x[1] + x[0] * x[0]);
	k2 = (y0*x0 + x[1] * x[0] - y0*x[0] - x[1] * x0 - sqrt(r0*r0*(-2 * y0*x[1] - 2 * x0*x[0] + x[1] * x[1] +
		y0*y0 + x0*x0 - r0*r0 + x[0] * x[0]))) / (-r0*r0 + x0*x0 - 2 * x0*x[0] + x[0] * x[0]);
	x_1 = (-k1*x[1] + x0 + k1*k1*x[0] + y0*k1) / (1 + k1*k1);
	y_1 = -(-x[1] - k1*x0 - y0*k1*k1 + k1*x[0]) / (1 + k1*k1);
	x_2 = (-k2*x[1] + x0 + k2*k2*x[0] + y0*k2) / (1 + k2*k2);
	y_2 = -(-x[1] - k2*x0 - y0*k2*k2 + k2*x[0]) / (1 + k2*k2);

	//%%%%判断逆时针是先到哪个切点%%%%%%%%
	double w[9] = { 0, -1, y0, 1, 0, -x0, -y0, x0, 0 };
	double r1[3] = { x_1 - x0, y_1 - y0, 0 };
	double r2[3] = { x_2 - x0, y_2 - y0, 0 };
	double v1[3], v2[3];
	double s[3] = { x[0] - x0, x[1] - y0, 0 };
	double s1 = 0, s2 = 0;
	for (int i = 0; i<3; i++)
	{
		v1[i] = w[i * 3 + 0] * r1[0] + w[i * 3 + 1] * r1[1] + w[i * 3 + 2] * r1[2];
		v2[i] = w[i * 3 + 0] * r2[0] + w[i * 3 + 1] * r2[1] + w[i * 3 + 2] * r2[2];
	}
	for (int i = 0; i < 3; i++)
	{
		s1 = s1 + s[i] * v1[i];
		s2 = s2 + s[i] * v2[i];
	}
	if (s1 > 0)
	{
		y[0] = x_1;
		y[1] = y_1;
		//cout << fixed << setprecision(10) << "y[0] =" << x_1 << endl;
		//cout << fixed << setprecision(10) << "y[1] =" << y_1 << endl;

	}
	if (s2 > 0)
	{
		y[0] = x_2;
		y[1] = y_2;
		//cout << fixed << setprecision(10) << "y[0] =" << x_2 << endl;
		//cout << fixed << setprecision(10) << "y[1] =" << y_2 << endl;

	}
	//cout << fixed << setprecision(10) << "res =" << res << endl;

	//ENU(xyz)转化为纬经高
	add = xy2ll(y[0], y[1]);
	y[0] = add[0];
	y[1] = add[1];

	//    y[0] = wei0 + y[1] / 111177.0;
	//    y[1] = jing0 + y[0] / 85155.0;
	//    cout<<"求切点的函数!"<<endl;
	//    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
	//    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1] <<endl;

	return res;
}
};
#endif
