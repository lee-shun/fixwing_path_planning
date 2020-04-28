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

#include "dropping_fw.hpp"

/**
 * @Input:
 * @Output:
 * @Description:
 */
float DROPPING_FW::get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}

/**
 * @Input: int
 * @Output:
 * @Description: 设定当前飞机的ID
 */
void DROPPING_FW::set_planeID(int id) {

  planeID = id;

  switch (planeID) {
  case 1:
    uavID = "uav1/";
    break;
  case 2:
    uavID = "uav2/";
    break;
  case 3:
    uavID = "uav3/";
    break;
  }
}

/**
 * @Input:
 * @Output:
 * @Description:
 */
void DROPPING_FW::ros_sub_pub() {

  /* 与航点相关的服务 */

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
      add2str(uavID, "mavros/cmd/arming"));

  set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>(add2str(uavID, "mavros/set_mode"));

  waypoint_setcurrent_client =
      nh.serviceClient<mavros_msgs::WaypointSetCurrent>(
          add2str(uavID, "mavros/mission/set_current"));

  waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>(
      add2str(uavID, "mavros/mission/pull"));

  waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>(
      add2str(uavID, add2str(uavID, "mavros/mission/push")));

  waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>(
      add2str(uavID, "mavros/mission/clear"));
}

/**
 * @Input: 需要发送的航点的长度、发送的航点数组的指针
 * @Output:
 * @Description: waypoint按照顺序逐次发给px4
 */
void DROPPING_FW::push_waypoints_to_px4(int size, mavros_msgs::Waypoint *points) {
  mavros_msgs::WaypointPush waypoint_push;

  for (int i = 0; i < size; i++) {

    waypoint_push.request.start_index = 0;

    waypoint_push.request.waypoints.push_back(points[i]);
  }
}

/**
 * @Input:
 * @Output:
 * @Description: 清空所有航点
 */
void DROPPING_FW::clear_waypoint() 
{

  mavros_msgs::WaypointClear waypoint_clear;

  if (waypoint_clear_client.call(waypoint_clear) &&
      waypoint_clear.response.success) {

    ROS_INFO("Waypoint clear success");
  }
}

void DROPPING_FW::plan_waypoint(int task_stage)
{
	int stage = task_stage;
	switch (stage) {
	case 1://投弹机第一阶段航迹规划 起飞点加盘旋点设置
		//定义变量
		double *add;
		double home_lat, home_long, home_x, home_y;
		double takeoff_x, takeoff_y;
		double runway_takeoff_length, runway_takeoff_angular;
		wei0 = 39.9890143;
		jing0 = 116.353457;
		//home点设置？
		home_lat = 39.9891248;
		home_long = 116.3558232;
		runway_takeoff_length = 100;
		runway_takeoff_angular = 0;
		//起飞点经纬获取
		add = ll2xy(home_lat, home_long);
		home_x = add[0];
		home_y = add[1];
		takeoff_x = home_x + runway_takeoff_length*cos(runway_takeoff_angular);
		takeoff_y = home_y + runway_takeoff_length*sin(runway_takeoff_angular);
		add = xy2ll(takeoff_x, takeoff_y);
		//起飞点设置：经纬高
		waypoint[0].x_lat = add[0];
		waypoint[0].y_long = add[1];
		waypoint[0].z_alt = 30;
		waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
		//设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
		waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		//第一个点的autocontinue必设置为true()默认为false
		waypoint[0].autocontinue = true;
		waypoint[0].is_current = true;
		//设置飞机的起飞爬升角
		waypoint[0].param1 = 45.0;
		//盘旋点设置
		waypoint[1].x_lat = wei0;
		waypoint[1].y_long = jing0;
		waypoint[0].z_alt = 30; ///说明param3为盘旋半径，逆时针
		waypoint[1].param3 = -30;                                                                
		//    waypoint[1].param4 = nan;
		waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
		//    设置命令为盘旋点
		waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[1].autocontinue = true;
		waypoint[1].is_current = false;
		break;

	case 2://投弹机第二阶段航迹    size：
		//变量定义
		double v = 15;            //飞机的飞行速度(单位:m/s)
		double paolenth;
		double R, pointlenth, desire_ds, ds;
		double tan_x, tan_y;
		double w1 = 39.9897585, w2 = 39.9897833, w3 = 39.9882652, w4 = 39.9882500;
		double j1 = 116.3526900, j2 = 116.3541295, j3 = 116.3542219, j4 = 116.3527874;
		wei0 = 39.9890143;
		jing0 = 116.353457;//四个角点的中心点
		wei0 = wc;
		jing0 = jc;
		double current_lat, current_lon;
		double current_x, current_y;
		double waypoint1_x, waypoint1_y;
		double distance_circle;
		double goal[2] = { 0, 0 };
		double goal_x,goal_y;
		double bbb[2] = { 0, 0 };
		double tanlenth, aaa;
		//给出目标点
		goal[0] = 39.9916445;
		goal[1] = 116.3564010;
		//以下为航迹规划---------------------------------------------------------------------------------------------------------
		///计算飞机飞行高度对应平抛距离
		paolenth = v * sqrt((2 * 30) / g);

		///计算飞机可以直接平抛的大圆半径
		desire_ds = 80;                          //在投掷之前必须先飞多少的直线
		ds = desire_ds + paolenth;
		R = sqrt(r*r + ds*ds);                  
		//把纬经高转化为ENU
		add = ll2xy(wc, jc);
		xc = add[0];
		yc = add[1];
		//把纬经高转化为ENU
		add = ll2xy(goal[0], goal[1]);
		goal_x = add[0];
		goal_y = add[1];

		pointlenth = sqrt((xc - goal_x)*(xc - goal_x) + (yc - goal_y)*(yc - goal_y));

		cout << "pointlenth=" << pointlenth << endl;
		double *tan;
		tan = point_tangency(goal);
		tanlenth = sqrt((goal_x - tan_x)*(goal_x - tan_x) + (goal_y - tan_y)*(goal_y - tan_y));

		add = ll2xy(tan[0], tan[1]);
		tan_x = add[0];
		tan_y = add[1];
		cout << "tan_x  =" << tan_x << endl;
		cout << "tan_y  =" << tan_y << endl;

		double n1_x, n1_y;
		double xout, yout;
		//以下为3种情况下下2-41号航点的写入
		//第一种条件1111111111111111111111111111111111111111111111111111111111111111111111111111111
		if (pointlenth >= R)     //目标点在圆外且切线长足够平抛
		{
			cout << "pointlenth >= R!!!" << endl;
			//切点写入2号航点	
			add = xy2ll(tan_x, tan_y);
			waypoint[2].x_lat = add[0];
			waypoint[2].y_long = add[1];
			waypoint[2].z_alt =30;
			waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[2].autocontinue = true;
			waypoint[2].is_current = false;
			//3-39号航点的写入
			for (int i = 3; i <= 39; i++)
			{
				double x = tan_x + (tanlenth - paolenth) / tanlenth*(goal_x - tan_x)*(i - 2) / 37;
				double y = tan_y + (tanlenth - paolenth) / tanlenth*(goal_y - tan_y)*(i - 2) / 37;

				if (i == 39){
					xout = x;
					yout = y;
				}

				add = xy2ll(x, y);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;

				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//40航点的写入 ，和第39个完全一致
			waypoint[40].x_lat = waypoint[39].x_lat;   
			waypoint[40].y_long = waypoint[39].y_long;
			waypoint[40].z_alt = 30;
			waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[40].autocontinue = true;
			waypoint[40].is_current = false;
			
			//41航点的写入,为目标点 
			waypoint[41].x_lat = goal[0];
			waypoint[41].y_long = goal[1];
			waypoint[41].z_alt = 30;
			waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[41].autocontinue = true;
			waypoint[41].is_current = false;
			
		}
		//111111111111111111111111111111111111111111111111111111111111111111111111111111111111
		//第二种情况22222222222222222222222222222222222222222222222222222222222222222222222222
		if (pointlenth < R && pointlenth >= r)     //目标点在圆外然而切线长不够平抛
		{
			cout << "pointlenth < R 且 pointlenth >= r!!!" << endl;

			n1_x = xc - tan_x;
			n1_y = yc - tan_y;
			//2号航点到11号航点的写入
			for (int i = 2; i <= 11; i++)
			{
				double x, y, x1, y1;

				x = -1 * r;
				y = (2 - i)*ds / 11;
				x1 = (-1 * n1_x*x + n1_y*y) / r + xc;
				y1 = -1 * (n1_y*x + n1_x*y) / r + yc;
				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//12号航点到27号航点的写入
			for (int i = 12; i <= 27; i++)
			{

				double x, y, x1, y1;

				x = cos((i - 27)*pi / 15)*r;
				y = -1 * ds + sin((i - 27)*pi / 15)*r;
				x1 = (-1 * n1_x*x + n1_y*y) / r + xc;
				y1 = -1 * (n1_y*x + n1_x*y) / r + yc;

				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//28号航点到39号航点的写入
			for (int i = 28; i <= 39; i++)
			{
				double x, y, x1, y1;

				x = r;
				y = (tanlenth - paolenth + ds) * (i - 27) / 12 - ds;
				x1 = (-1 * n1_x*x + n1_y*y) / r + xc;
				y1 = -1 * (n1_y*x + n1_x*y) / r + yc;

				if (i == 39){
					xout = x1;
					yout = y1;
				}

				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//40号航点的写入，和第39个完全一致
			waypoint[40].x_lat = waypoint[39].x_lat;    
			waypoint[40].y_long = waypoint[39].y_long;
			waypoint[40].z_alt = 30;
			waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[40].autocontinue = true;
			waypoint[40].is_current = false;


			//41号航点的写入
			waypoint[41].x_lat = goal[0];
			waypoint[41].y_long = goal[1];
			waypoint[41].z_alt = 30;
			waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[41].autocontinue = true;
			waypoint[41].is_current = false;

		}
		//2222222222222222222222222222222222222222222222222222222222222222222222222222222222
		//第三种情况333333333333333333333333333333333333333333333333333333333333333333333333
		if (pointlenth < r)     //目标点在圆外且切线长足够平抛
		{
			cout << "pointlenth < r!!!" << endl;
			if (pointlenth < 0.0000001)
			{
				goal_x = goal_x + 0.0000001;
				goal_y = goal_y + 0.0000001;
			}

			n1_x = goal_x - xc;
			n1_y = goal_y - yc;

			double a = r + ds + pointlenth;
			//2到6号航点的写入
			for (int i = 2; i <= 6; i++)
			{

				double x, y, x1, y1;
				x = a * (i - 2) / 5;
				y = -1 * r;
				x1 = (n1_y*x + n1_x*y) / pointlenth + xc;
				y1 = (-1 * n1_x*x + n1_y*y) / pointlenth + yc;

				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//7到31号航点的写入
			for (int i = 7; i <= 31; i++)
			{

				double x, y, x1, y1;

				x = a + cos((i - 15)*pi / 16)*a;
				y = ds + pointlenth + sin((i - 15)*pi / 16)*a;
				x1 = (n1_y*x + n1_x*y) / pointlenth + xc;
				y1 = (-1 * n1_x*x + n1_y*y) / pointlenth + yc;

				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//32到39号航点的写入
			for (int i = 32; i <= 39; i++)
			{

				double x, y, x1, y1;

				x = 0;
				y = ds + pointlenth - desire_ds * (i - 31) / 8;
				x1 = (n1_y*x + n1_x*y) / pointlenth + xc;
				y1 = (-1 * n1_x*x + n1_y*y) / pointlenth + yc;

				if (i == 39){
					xout = x1;
					yout = y1;
				}

				add = xy2ll(x1, y1);
				waypoint[i].x_lat = add[0];
				waypoint[i].y_long = add[1];
				waypoint[i].z_alt = 30;
				waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				waypoint[i].autocontinue = true;
				waypoint[i].is_current = false;

			}
			//40号航点的写入
			waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
			waypoint[40].y_long = waypoint[39].y_long;
			waypoint[40].z_alt = 30;
			waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[40].autocontinue = true;
			waypoint[40].is_current = false;
			//41号航点的写入
			waypoint[41].x_lat = goal[0];
			waypoint[41].y_long = goal[1];
			waypoint[41].z_alt = 30;
			waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[41].autocontinue = true;
			waypoint[41].is_current = false;
		}
		//333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
		//#############################已完成2-41号航点的写入####################################
		//降落航线规划-----------------------------------------------------------------降落部分
		//单独给降落点赋值
		double land_lat, land_lon;
		double land_x, land_y;
		double theta, dland;
		theta = 0;//降落方向
		dland = 300;//最后的直线长
		double land_length = 20;
		land_x = home_x - land_length*cos(theta);
		land_y = home_y - land_length*sin(theta);
		add = xy2ll(land_x, land_y);
		land_lat = add[0];
		land_lon = add[1];
		//着陆点与起飞点不一致
		//    land_lat= 39.9890093;
		//    land_lon= 116.3498190;
		//    add=ll2xy(land_lat,land_lon);
		//    land_x = add[0];
		//    land_y = add[1];
		double erout1_x, erout1_y;
		erout1_x = (yout - goal_y) / sqrt((yout - goal_y)*(yout - goal_y) + (goal_x - xout)*(goal_x - xout))*r;
		erout1_y = (goal_x - xout) / sqrt((yout - goal_y)*(yout - goal_y) + (goal_x - xout)*(goal_x - xout))*r;
		double o_out1_x, o_out1_y;
		double o_out2_x, o_out2_y;

		o_out1_x = goal_x + erout1_x;
		o_out1_y = goal_y + erout1_y;
		cout << "o_out1_x=" << o_out1_x << endl;
		cout << "o_out1_y=" << o_out1_y << endl;


		double tanout3_x, tanout3_y;
		tanout3_x = land_x - dland*cos(theta);
		tanout3_y = land_y - dland*sin(theta);

		o_out2_x = tanout3_x - r*sin(theta);
		o_out2_y = tanout3_y + r*cos(theta);
		double tanout1_x, tanout1_y, tanout2_x, tanout2_y;
		double theta_goal, theta_tan1, theta_tan2, dtheta1, dtheta2, theta_i1, theta_i2, theta_tan3;
		Point A = Point(o_out1_x, o_out1_y), B = Point(o_out2_x, o_out2_y);
		Circle AA = Circle(A, r), BB = Circle(B, r);
		int num = getTan(AA, BB, va, vb);

		tanout1_x = va[1].x;
		tanout1_y = va[1].y;

		tanout2_x = vb[1].x;
		tanout2_y = vb[1].y;


		theta_goal = atan2(goal_y - o_out1_y, goal_x - o_out1_x);
		theta_tan1 = atan2(tanout1_y - o_out1_y, tanout1_x - o_out1_x);



		if (theta_tan1<theta_goal){
			theta_tan1 = theta_tan1 + 2 * pi;
		}

		dtheta1 = theta_tan1 - theta_goal;
		//################################42-51号航点的写入
	        int i;
		for (i = 42; i <= 51; i++)
		{

			theta_i1 = dtheta1*(i - 41) / 10 + theta_goal;
			x = o_out1_x + r*cos(theta_i1);
			y = o_out1_y + r*sin(theta_i1);

			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;
			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;
		}//################################
		//#################################52-61号航点的写入
		for (i = 52; i <= 61; i++)
		{
			x = (tanout2_x - tanout1_x)*(i - 51) / 10 + tanout1_x;
			y = (tanout2_y - tanout1_y)*(i - 51) / 10 + tanout1_y;
			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;
			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;
		}//################################
		theta_tan2 = atan2(tanout2_y - o_out2_y, tanout2_x - o_out2_x);
		theta_tan3 = atan2(tanout3_y - o_out2_y, tanout3_x - o_out2_x);
		if (theta_tan3<theta_tan2){
			theta_tan3 = theta_tan3 + 2 * pi;
		}
		dtheta2 = theta_tan3 - theta_tan2;
		//#################################62-71号航点的写入
		for (i = 62; i <= 71; i++){
			theta_i2 = dtheta2*(i - 61) / 10 + theta_tan2;
			x = o_out2_x + r*cos(theta_i2);
			y = o_out2_y + r*sin(theta_i2);
			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;
			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;
		}//################################
		//降落航点的写入
		waypoint[72].x_lat = land_lat;
		waypoint[72].y_long = land_lon;
		waypoint[72].z_alt = 0;
		waypoint[72].command = mavros_msgs::CommandCode::NAV_LAND;//降落命令
		waypoint[72].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[72].autocontinue = true;
		waypoint[72].is_current = false;
		//--------------------------------------------------------------------降落部分
		break;
}
}

void DROPPING_FW::run()
{
	//清空航点
	clear_waypoint();
	double *add;
	//将当前的目标航点设为0号航点
	mavros_msgs::WaypointSetCurrent Current_wp;
	Current_wp.request.wp_seq = 0;
	if (waypoint_setcurrent_client.call(Current_wp) && Current_wp.response.success)
	{
		ROS_INFO("%d", Current_wp.response.success);
		ROS_INFO("Waypoint set to 0 success");
	}
	//调用航迹规划函数规划投弹机第一阶段航线
	plan_waypoint(1);
	//完成第一段航线的push  size：2
	push_waypoints_to_px4(2, waypoint);
	//重启qgc !!!!!!!!!!!!!!!!脚本路径需修改
	std::string res;
	system("sh /home/sss/qgc_restart.sh");
	std::cout << res << '\n';
	//起飞检测--------------------------------------------------------------
	int mission_receive_flag = 1;
	int wait_flag = 1;
	int trigger_flag = 1;

	while (ros::ok() && (current_state.mode != "AUTO.MISSION" || !current_state.armed))
	{
		ros::spinOnce();
		if (wait_flag)
		{
			cout << "目前还没有解锁或者到MISSION模式" << endl << "请稍等......" << endl;
			wait_flag = 0;
		}
		continue;
	}
	wait_flag = 1;
	if (trigger_flag){
		cout << "开始起飞!!!" << endl;
		trigger_flag = 0;
	}
	//----------------------------------------------------------------------
	//接收侦察机节点消息
    while(ros::ok())
    {
        /* 1. 接收消息*/
		
    }
	//接收完毕跳出循环

	//调用航迹规划函数规划投弹机第二阶段航线
	plan_waypoint(2);
	//完成第一段航线的push  size：2
	push_waypoints_to_px4(73, waypoint);
	//重启qgc !!!!!!!!!!!!!!!!脚本路径需修改
	system("sh /home/sss/qgc_restart.sh");
	std::cout << res << '\n';
	//将当前的航点设置为盘旋航点
	mavros_msgs::WaypointSetCurrent waypoint_setcurrent;
	waypoint_setcurrent.request.wp_seq = 1;
	if (waypoint_setcurrent_client.call(waypoint_setcurrent) && waypoint_setcurrent.response.success)
	{
		ROS_INFO("Waypoint set to 1 success");
	}
	//盘旋切出前往投弹航线部分-----------------------------------------------------------------------
	double distance_tan;
	double waypoint2_x, waypoint2_y;
	double current_lat, current_lon;
	double current_x, current_y;
	int begin_task_flag = 0;
	add = ll2xy(waypoint[2].x_lat, waypoint[2].y_long);
	waypoint2_x = add[0];
	waypoint2_y = add[1];


	//判断是否接近切点，接近则将flag置为1，跳出循环
	while (ros::ok() && (!begin_task_flag))
	{
		ros::spinOnce();
		current_lat = current_gps.latitude;
		current_lon = current_gps.longitude;
		add = ll2xy(current_lat, current_lon);
		current_x = add[0];
		current_y = add[1];
		distance_tan = sqrt((current_x - waypoint2_x)*(current_x - waypoint2_x) +
			(current_y - waypoint2_y)*(current_y - waypoint2_y));
		cout << "distance_tan=" << distance_tan << endl;
		if (distance_tan<3)
		{

			begin_task_flag = 1;

		}
		ros::spinOnce();
		rate.sleep();

	}
	    //改变当前目标航点，切出盘旋，进入第二段航线
		Current_wp.request.wp_seq = 5;
		if (waypoint_setcurrent_client.call(Current_wp) && Current_wp.response.success)
		{
			//ROS_INFO("%d", Current_wp.response.success);
			cout << "开始执行投弹任务!!" << endl;
		}
		
	

}
int main() { 

	return 0; }
