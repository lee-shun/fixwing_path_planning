/*************************************************************************************
*
*Author:lee-shun & Hao Sun
*
*Date: 
*
*Description:
*去年的侦查程序的复现
*
*
*************************************************************************************/

#include "zhenchaji_fw.hpp"

void DROPPING_FW::plan_waypoint(int task_stage)
{
	int stage = task_stage;
	switch (stage) {
	case 1://侦察机第一阶段航迹规划 起飞点加第一段侦察航线及最后的盘旋点设置
		//定义变量
		double *add;
		double home_lat, home_long, home_x, home_y;
		double takeoff_x, takeoff_y;
		double runway_takeoff_length, runway_takeoff_angular;
		wei0 = 39.9890143;
		jing0 = 116.353457;
		double w2 = 39.9897585, w1 = 39.9897833, w4 = 39.9882652, w3 = 39.9882500;
		double j2 = 116.3526900, j1 = 116.3541295, j4 = 116.3542219, j3 = 116.3527874;
		double *add1;
		double *add2;
		double *add3;
		double *add4;
		double w1_x, w2_x, w3_x, w4_x, w1_y, w2_y, w3_y, w4_y;
		add1 = ll2xy(w1, j1);
		add2 = ll2xy(w2, j2);
		add3 = ll2xy(w3, j3);
		add4 = ll2xy(w4, j4);
		w1_x = add1[0];
		w1_y = add1[1];
		w2_x = add2[0];
		w2_y = add2[1];
		w3_x = add3[0];
		w3_y = add3[1];
		w4_x = add4[0];
		w4_y = add4[1];
		
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

		for (int i = 1; i <= 10; i++)
		{
			double x = w1_x+(w2_x-w1_x) / 6;
			double y = w4_y-(w4_y - w1_y)*(i-1) / 9;
			
			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		for (int i = 11; i <= 20; i++)
		{
			double x = w1_x + ((w2_x - w1_x)*2)/6;
			double y = w1_y + (w4_y - w1_y)*(i - 11) / 9;

			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		for (int i = 21; i <= 30; i++)
		{
			double x = w1_x + ((w2_x - w1_x)*3) / 6;
			double y = w4_y - (w4_y - w1_y)*(i - 21) / 9;

			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		for (int i = 31; i <= 40; i++)
		{
			double x = w1_x + ((w2_x - w1_x) * 4) / 6;
			double y = w1_y + (w4_y - w1_y)*(i - 31) / 9;

			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		for (int i = 41; i <= 50; i++)
		{
			double x = w1_x + ((w2_x - w1_x) * 5) / 6;
			double y = w4_y - (w4_y - w1_y)*(i - 41) / 9;

			add = xy2ll(x, y);
			waypoint[i].x_lat = add[0];
			waypoint[i].y_long = add[1];
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		//降落点 ---------------------------
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
		//    land_lat=39.9890093;
		//    land_lon=116.3498190;

		waypoint[51].x_lat = land_lat;
		waypoint[51].y_long = land_lon;
		waypoint[51].z_alt = 0;
		waypoint[51].command = mavros_msgs::CommandCode::NAV_LAND;
		waypoint[51].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[51].autocontinue = true;
		waypoint[51].is_current = false;
	        break;
		//------------------------------------------------------------
		
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
	//调用航迹规划函数规划侦察航线
	plan_waypoint(1);
	//完成航线的push  size：52
	push_waypoints_to_px4(52, waypoint);
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
		
}

int main() { return 0; }
