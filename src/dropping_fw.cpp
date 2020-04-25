
#include "dropping_fw.hpp"

/**
 * @Input: 
 * @Output: 
 * @Description: 
 */
float DROPPING_FW::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
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

int main()
{
    return 0;
}
