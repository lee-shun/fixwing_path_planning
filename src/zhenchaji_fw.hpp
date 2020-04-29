#ifndef _ZHENCHAJI_FW_HPP_
#define _ZHENCHAJI_FW_HPP_
/*************************************************************************************
*
*Author:
*
*Date:
*
*Description:
*
*
*
*************************************************************************************/
#include "dropping_fw.hpp"

class ZHENCHAJI_FW : public DROPPING_FW {
public:
  void run(); /* 重定义父类中的run函数*/

private:
  void plan_waypoint(int task_stage); /* 重定义父类中的run函数*/
};
#endif
