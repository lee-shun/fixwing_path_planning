/*************************************************************************************
*
*Author:lee-shun & Hao Sun
*
*Date: 
*
*Description:
*去年的侦查程序的复现
*
* 51号航点：39.9880063  116.3547633
* 51号航点：39.9888508  116.3573833
*************************************************************************************/

#include "zhenchaji_fw.hpp"

void ZHENCHAJI_FW::plan_waypoint(int task_stage)
{
	int stage = task_stage;
		double *add;
		double home_lat, home_long, home_x, home_y;
		double takeoff_x, takeoff_y;
		double runway_takeoff_length, runway_takeoff_angular;
		wei0 = 39.9890143;
		jing0 = 116.353457;
		double w4 = 39.9897585, w3 = 39.9897833, w2 = 39.9882652, w1 = 39.9882500;
		double j4 = 116.3526900, j3 = 116.3541295, j2 = 116.3542219, j1 = 116.3527874;
		double *add1;
		double *add2;
		double *add3;
		double *add4;
		double w1_x, w2_x, w3_x, w4_x, w1_y, w2_y, w3_y, w4_y;
	switch (stage) {
	case 1://侦察机第一阶段航迹规划 起飞点加第一段侦察航线及最后的盘旋点设置
		//定义变量
		
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

		for (int i = 1; i <= 50; i++)
		{
			
			
			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		waypoint[1].x_lat = 39.9897585;
		waypoint[1].y_long = 116.3527874;	
		waypoint[2].x_lat = 39.9892556666667;
		waypoint[2].y_long = 116.3527874;
		waypoint[3].x_lat = 39.9887528333333;
		waypoint[3].y_long = 116.3527874;
		waypoint[4].x_lat = 39.98825;
		waypoint[4].y_long = 116.3527874;
		waypoint[5].x_lat = 39.9879985833333;
		waypoint[5].y_long = 116.353146025;
		waypoint[6].x_lat = 39.98825;
		waypoint[6].y_long = 116.35350465;
		waypoint[7].x_lat = 39.9887528333333;
		waypoint[7].y_long = 116.35350465;
		waypoint[8].x_lat = 39.9892556666667;
		waypoint[8].y_long = 116.35350465;
		waypoint[9].x_lat = 39.9897585;
		waypoint[9].y_long = 116.35350465;
		waypoint[10].x_lat = 39.9900099166667;
		waypoint[10].y_long = 116.353863275;
		waypoint[11].x_lat = 39.9897585;
		waypoint[11].y_long = 116.3542219;
		waypoint[12].x_lat = 39.9892556666667;
		waypoint[12].y_long = 116.3542219;
		waypoint[13].x_lat = 39.9887528333333;
		waypoint[13].y_long = 116.3542219;
		waypoint[14].x_lat = 39.98825;
		waypoint[14].y_long = 116.3542219;
		waypoint[15].x_lat = 39.9880063;
		waypoint[15].y_long = 116.3547633;
		
	
		
		//降落点 ---------------------------
		

		waypoint[16].x_lat = 39.9888508;
		waypoint[16].y_long = 116.3573833;
		waypoint[16].z_alt = 0;
		waypoint[16].command = mavros_msgs::CommandCode::NAV_LAND;
		waypoint[16].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[16].autocontinue = true;
		waypoint[16].is_current = false;
	        break;
		//------------------------------------------------------------
	case 2://直线轨迹跟踪
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
		for (int i = 1; i <= 12; i++)
		{


			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		waypoint[1].x_lat = 39.9891248;
		waypoint[1].y_long = 116.358171858329;
		waypoint[2].x_lat = 39.9891248;
		waypoint[2].y_long = 116.359346187493;
		waypoint[3].x_lat = 39.9891248;
		waypoint[3].y_long = 116.360520516658;
		waypoint[4].x_lat = 39.9891248;
		waypoint[4].y_long = 116.361694845822;
		waypoint[5].x_lat = 39.9891248;
		waypoint[5].y_long = 116.362869174987;
		waypoint[6].x_lat = 39.9876856534139;
		waypoint[6].y_long = 116.363808638318;
		waypoint[7].x_lat = 39.9876856534139;
		waypoint[7].y_long = 116.36474810165;
		waypoint[8].x_lat = 39.9876856534139;
		waypoint[8].y_long = 116.365687564982;
		waypoint[9].x_lat = 39.9876856534139;
		waypoint[9].y_long = 116.366627028313;
		waypoint[10].x_lat = 39.9876856534139;
		waypoint[10].y_long = 116.367566491645;
		waypoint[11].x_lat = 39.9876856534139;
		waypoint[11].y_long = 116.368505954976;
		waypoint[12].x_lat = 39.9880063;
		waypoint[12].y_long = 116.3547633;

		//降落点 ---------------------------
		

		waypoint[13].x_lat = 39.9888508;
		waypoint[13].y_long = 116.3573833;
		waypoint[13].z_alt = 0;
		waypoint[13].command = mavros_msgs::CommandCode::NAV_LAND;
		waypoint[13].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[13].autocontinue = true;
		waypoint[13].is_current = false;
		break;
	case 3://圆轨迹跟踪
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
		for (int i = 1; i <= 25; i++)
		{


			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		waypoint[1].x_lat = 39.9895745333082;
		waypoint[1].y_long = 116.356997529164;
		waypoint[2].x_lat = 39.9900242666163;
		waypoint[2].y_long = 116.356997529164;
		waypoint[3].x_lat = 39.9904739999245;
		waypoint[3].y_long = 116.356997529164;
		waypoint[4].x_lat = 39.9909237332326;
		waypoint[4].y_long = 116.356997529164;
		waypoint[5].x_lat = 39.9913734665407;
		waypoint[5].y_long = 116.356997529164;
		waypoint[6].x_lat = 39.9918231998489;
		waypoint[6].y_long = 116.356997529164;
		waypoint[7].x_lat = 39.9922612843482;
		waypoint[7].y_long = 116.356902086575;
		waypoint[8].x_lat = 39.9926518955944;
		waypoint[8].y_long = 116.35662610149;
		waypoint[9].x_lat = 39.9929527047981;
		waypoint[9].y_long = 116.356199481178;
		waypoint[10].x_lat = 39.9931311146145;
		waypoint[10].y_long = 116.355668456568;
		waypoint[11].x_lat = 39.9931677915716;
		waypoint[11].y_long = 116.355090572417;
		waypoint[12].x_lat = 39.993058761152;
		waypoint[12].y_long = 116.354528451439;
		waypoint[13].x_lat = 39.9928158384936;
		waypoint[13].y_long = 116.354043008165;
		waypoint[14].x_lat = 39.9924653480356;
		waypoint[14].y_long = 116.353686847905;
		waypoint[15].x_lat = 39.9920452708577;
		waypoint[15].y_long = 116.35349856615;
		waypoint[16].x_lat = 39.9916011288401;
		waypoint[16].y_long = 116.35349856615;
		waypoint[17].x_lat = 39.9911810516622;
		waypoint[17].y_long = 116.353686847905;
		waypoint[18].x_lat = 39.9908305612042;
		waypoint[18].y_long = 116.354043008165;
		waypoint[19].x_lat = 39.9905876385458;
		waypoint[19].y_long = 116.354528451439;
		waypoint[20].x_lat = 39.9904786081262;
		waypoint[20].y_long = 116.355090572417;
		waypoint[21].x_lat = 39.9905152850833;
		waypoint[21].y_long = 116.355668456568;
		waypoint[22].x_lat = 39.9906936948997;
		waypoint[22].y_long = 116.356199481178;
		waypoint[23].x_lat = 39.9909945041033;
		waypoint[23].y_long = 116.35662610149;
		waypoint[24].x_lat = 39.9913851153496;
		waypoint[24].y_long = 116.356902086575;
		waypoint[25].x_lat = 39.9880063;
		waypoint[25].y_long = 116.3547633;

		//降落点 ---------------------------

		waypoint[26].x_lat = 39.9888508;
		waypoint[26].y_long = 116.3573833;
		waypoint[26].z_alt = 0;
		waypoint[26].command = mavros_msgs::CommandCode::NAV_LAND;
		waypoint[26].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[26].autocontinue = true;
		waypoint[26].is_current = false;
		break;
	case 4:
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
		for (int i = 1; i <= 40; i++)
		{


			waypoint[i].z_alt = 30;

			waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			waypoint[i].autocontinue = true;
			waypoint[i].is_current = false;

		}
		waypoint[1].x_lat = 39.9890143;
		waypoint[1].y_long = 116.356979987493;
		waypoint[2].x_lat = 39.9899137666163;
		waypoint[2].y_long = 116.356979987493;
		waypoint[3].x_lat = 39.9908132332326;
		waypoint[3].y_long = 116.356979987493;
		waypoint[4].x_lat = 39.9912629665407;
		waypoint[4].y_long = 116.356979987493;
		waypoint[5].x_lat = 39.9917126998489;
		waypoint[5].y_long = 116.356979987493;
		waypoint[6].x_lat = 39.992162433157;
		waypoint[6].y_long = 116.356979987493;
		waypoint[7].x_lat = 39.9929629584455;
		waypoint[7].y_long = 116.356756864952;
		waypoint[8].x_lat = 39.9939613663896;
		waypoint[8].y_long = 116.356392822911;
		waypoint[9].x_lat = 39.994590993021;
		waypoint[9].y_long = 116.356040524162;
		waypoint[10].x_lat = 39.9951306729908;
		waypoint[10].y_long = 116.355570792496;
		waypoint[11].x_lat = 39.995418502308;
		waypoint[11].y_long = 116.355247851976;
		waypoint[12].x_lat = 39.9959815684098;
		waypoint[12].y_long = 116.353973704832;
		waypoint[13].x_lat = 39.9964798729153;
		waypoint[13].y_long = 116.352846348834;
		waypoint[14].x_lat = 39.9969296062234;
		waypoint[14].y_long = 116.351830554107;
		waypoint[15].x_lat = 39.9972893928699;
		waypoint[15].y_long = 116.351014395338;
		waypoint[16].x_lat = 39.9975592328548;
		waypoint[16].y_long = 116.350403744172;
		waypoint[17].x_lat = 39.9980925411791;
		waypoint[17].y_long = 116.349191097112;
		waypoint[18].x_lat = 39.998389890873;
		waypoint[18].y_long = 116.348306056095;
		waypoint[19].x_lat = 39.9984510191348;
		waypoint[19].y_long = 116.347342915843;
		waypoint[20].x_lat = 39.9982693017689;
		waypoint[20].y_long = 116.346406047547;
		waypoint[21].x_lat = 39.9978644306715;
		waypoint[21].y_long = 116.345596975422;
		waypoint[22].x_lat = 39.9972802799082;
		waypoint[22].y_long = 116.345003374989;
		waypoint[23].x_lat = 39.9965801512783;
		waypoint[23].y_long = 116.344689572064;
		waypoint[24].x_lat = 39.9958399145824;
		waypoint[24].y_long = 116.344689572064;
		waypoint[25].x_lat = 39.9951397859525;
		waypoint[25].y_long = 116.345003374989;
		waypoint[26].x_lat = 39.9945556351892;
		waypoint[26].y_long = 116.345596975422;
		waypoint[27].x_lat = 39.9941507640918;
		waypoint[27].y_long = 116.346406047547;
		waypoint[28].x_lat = 39.9939690467259;
		waypoint[28].y_long = 116.347342915843;
		waypoint[29].x_lat = 39.9940301749878;
		waypoint[29].y_long = 116.348306056095;
		waypoint[30].x_lat = 39.9943275246817;
		waypoint[30].y_long = 116.349191097112;
		waypoint[31].x_lat = 39.9948288733545;
		waypoint[31].y_long = 116.349902130965;

		waypoint[32].x_lat = 39.9948288733545;
		waypoint[32].y_long = 116.35108708731;
		waypoint[33].x_lat = 39.9948288733545;
		waypoint[33].y_long = 116.352272043655;
		waypoint[34].x_lat = 39.9948288733545;
		waypoint[34].y_long = 116.353457;
		waypoint[35].x_lat = 39.9933752300158;
		waypoint[35].y_long = 116.353457	;
		waypoint[36].x_lat = 39.9919215866772;
		waypoint[36].y_long = 116.353457;
		waypoint[37].x_lat = 39.9904679433386;
		waypoint[37].y_long = 116.353457;
		waypoint[38].x_lat = 39.9890143;
		waypoint[38].y_long = 116.353457;
		waypoint[39].x_lat = 39.98825;
		waypoint[39].y_long = 116.3542219;
		waypoint[40].x_lat = 39.9880063;
		waypoint[40].y_long = 116.3547633;
		//降落点 ---------------------------
		waypoint[41].x_lat = 39.9888508;
		waypoint[41].y_long = 116.3573833;
		waypoint[41].z_alt = 0;
		waypoint[41].command = mavros_msgs::CommandCode::NAV_LAND;
		waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		waypoint[41].autocontinue = true;
		waypoint[41].is_current = false;
		break;
		//------------------------------------------------------------	
}
}

void ZHENCHAJI_FW::run()
{
    ros_sub_pub();
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
	int flag;
	cin >> flag;
	if (flag == 1)
	{
		//调用航迹规划函数规划侦察航线
		plan_waypoint(1);
		//完成航线的push  size：17
		push_waypoints_to_px4(17, waypoint);
	}
	if (flag == 2)
	{
		//调用航迹规划函数规划侦察航线
		plan_waypoint(2);
		//完成航线的push  size：14
		push_waypoints_to_px4(14, waypoint);
	}
	if (flag == 3)
	{
		//调用航迹规划函数规划侦察航线
		plan_waypoint(3);
		//完成航线的push  size：27
		push_waypoints_to_px4(27, waypoint);
	}
	if (flag == 4)
	{
		//调用航迹规划函数规划侦察航线
		plan_waypoint(4);
		//完成航线的push  size：17
		push_waypoints_to_px4(42, waypoint);
	}
	//重启qgc !!!!!!!!!!!!!!!!脚本路径需修改
	std::string res;
	/* system("sh /home/sss/qgc_restart.sh"); */
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

int main(int argc, char **argv) {

  ros::init(argc, argv, "zhenchaji_fw");

  ZHENCHAJI_FW zhenchaji_fw;

  zhenchaji_fw.set_planeID(-1);

  zhenchaji_fw.run();

  return 0;
}
