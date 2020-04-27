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
void DROPPING_FW::clear_waypoint() {

  mavros_msgs::WaypointClear waypoint_clear;

  if (waypoint_clear_client.call(waypoint_clear) &&
      waypoint_clear.response.success) {

    ROS_INFO("Waypoint clear success");
  }
}

void DROPPING_FW::run()
{
    while(ros::ok())
    {
        /* 1. 规划起飞盘旋 */
        /* 2.  */
    }

}
int main() { return 0; }
