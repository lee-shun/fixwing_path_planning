
/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

/************************************************

注意事项:
1 注意经纬度转化
2 注意在push的时候一定要注意尺寸!!
3 不能对地址进行cout,否则会出现0等错误




************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include<iomanip>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/commander_msgs.h>
#include <mavros_msgs/sub_plane_msgs.h>





using namespace std;
using namespace mavros_msgs;


///注意!!!!!
/// 最后所有的经纬转米的公式里面的参数一定要改!!!

//一些常值，暂定为全局常值变量
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量+
const double g = 9.80665;  //重力加速度
const float pi = 3.141593;
bool mission_received = 0;
int r = 30; //盘旋半径
//投弹区中心点的经纬度
double wc;
double jc;
//jing0,wei0 的含义待定
double jing0,wei0;
double xc,yc,goal_x,goal_y;
double x,y;

//waypoint list的大小
int size;


//>------------------------用于求解两个园的外切点----------------------------------


#define rep(i,a,b) for(int i=a;i<b;i++)
#define per(i,a,b) for(int i=b-1;i>=a;i--)

const double eps = 1e-9;

class Point;
typedef Point Vec;


/*
1.精度  要学着用dcmp
2.引用  不能交换两个引用swap（）,引用中间不能变
*/
//三态函数比较;精度问题
int dcmp(double x) {
    if (fabs(x)<eps) return 0;
    return x<0 ? -1 : 1;
}



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







//>----------------------------------------------------------

































//计算原理
//2*pi*R(地球半径)/360
//经度的话要乘以一个cos纬度

//将ENU(xyz)转换为经纬度(LLA)
//使用方式
//add=ll2xy(x[0],x[1]);
//x[0]=add[0];
//x[1]=add[1];

double *xy2ll(double x, double y)
{
    double LL[2];
    double *ret= LL ;

    //纬度
    LL[0] = wei0 + y / 111177.0;                                                                       ///?????????????
    //经度
    LL[1] = jing0 + x / 85155.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}

//将经纬度(LLA)转换为ENU(xyz)
//使用方式
//add=xy2ll(y[0],y[1]);
//y[0] = add[0];
//y[1] = add[1];
double *ll2xy(double w, double j)
{
    double XY[2];
    double *ret= XY;
    //X
    XY[0] = (j - jing0) * 85155.0;
    //Y
    XY[1] = (w - wei0) * 111177.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}


WaypointPush waypoint_push;



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
    double x_1,y_1,x_2,y_2;
    double k1,k2,*res= y ;
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
    add=ll2xy(g[0],g[1]);
    x[0]=add[0];
    x[1]=add[1];

    //    cout<<"纬经高转化为ENU(xyz)2"<<endl;
    //    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;


    //找出两个切点（x_1,y_1）(x_2,y_2)                                             ///此处采取的圆的半径为r0。值为r，盘旋半径
    k1 = (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 + sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0]))) / (-r0*r0 + x0*x0 - 2*x0*x[1] + x[0]*x[0]);
    k2= (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 - sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0])))/(-r0*r0 + x0*x0 - 2*x0*x[0] + x[0]*x[0]);
    x_1 = (-k1*x[1] + x0 + k1*k1*x[0] + y0*k1) / (1 + k1*k1);
    y_1 =-(-x[1] - k1*x0 - y0*k1*k1 + k1*x[0]) / (1 + k1*k1);
    x_2 = (-k2*x[1] + x0 + k2*k2*x[0] + y0*k2) / (1 + k2*k2);
    y_2 =-(-x[1] - k2*x0 - y0*k2*k2 + k2*x[0]) / (1 + k2*k2);

    //%%%%判断逆时针是先到哪个切点%%%%%%%%
    double w[9] = {0,-1,y0,1,0,-x0,-y0,x0,0};
    double r1[3] = {x_1 - x0, y_1-y0, 0};
    double r2[3] = {x_2 - x0, y_2-y0, 0};
    double v1[3],v2[3];
    double s[3] = {x[0] - x0, x[1] - y0, 0};
    double s1=0,s2=0;
    for (int i = 0;i<3;i++)
    {
        v1[i] = w[i*3+0]*r1[0] + w[i*3+1]*r1[1] + w[i*3+2]*r1[2];
        v2[i] = w[i*3+0]*r2[0] + w[i*3+1]*r2[1] + w[i*3+2]*r2[2];
    }
    for (int i = 0;i < 3;i++)
    {
        s1 = s1 + s[i]*v1[i];
        s2 = s2 + s[i]*v2[i];
    }
    if (s1 > 0)
    {
        y[0] = x_1;
        y[1] = y_1;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_1 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_1 <<endl;

    }
    if (s2 > 0)
    {
        y[0] = x_2;
        y[1] = y_2;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_2 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_2 <<endl;

    }
    cout<<fixed<< setprecision(10)<<"res ="<< res <<endl;

    //ENU(xyz)转化为纬经高
    add=xy2ll(y[0],y[1]);
    y[0] = add[0];
    y[1] = add[1];

    //    y[0] = wei0 + y[1] / 111177.0;
    //    y[1] = jing0 + y[0] / 85155.0;
    //    cout<<"求切点的函数!"<<endl;
    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1] <<endl;

    return res;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//geometry_msgs::Twist att_tw;
//void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
//{
//    att_tw = *msg;
//    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
//}

/*
# Waypoint.msg
#
# ROS representation of MAVLink MISSION_ITEM
# See mavlink documentation



# see enum MAV_FRAME
uint8 frame
uint8 FRAME_GLOBAL = 0
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_MISSION = 2
uint8 FRAME_GLOBAL_REL_ALT = 3
uint8 FRAME_LOCAL_ENU = 4

# see enum MAV_CMD and CommandCode.msg
uint16 command

bool is_current
bool autocontinue
# meaning of this params described in enum MAV_CMD
float32 param1
float32 param2
float32 param3
float32 param4
float64 x_lat
float64 y_long
float64 z_alt


*/

//用于打印航迹点的子函数
void printwaypoint(const mavros_msgs::WaypointList points)
{
    cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {

        cout<<i<<" "<<points.waypoints[i].command<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;



    }

}




mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
    
    //printwaypoint(current_waypoints);
}



mavros_msgs::WaypointReached reached_waypoints;
void waypoints_reached(const mavros_msgs::WaypointReached::ConstPtr& msg)
{
    reached_waypoints = *msg;

    //printwaypoint(current_waypoints);
}


sensor_msgs::NavSatFix current_gps;
void current_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    //cout<<current_gps<<endl;

    //printwaypoint(current_waypoints);
}




//#################################接收串口发过来的消息
mavros_msgs::commander_msgs commander;
void commander_callback(const mavros_msgs::commander_msgs::ConstPtr& msg)
{
    commander = *msg;
    //cout<<current_gps<<endl;

    //printwaypoint(current_waypoints);
}





//////////////////////////////////
////////// 主函数//////////////////
//////////////////////////////////
int main(int argc, char **argv)
{




    int plane_id=4;              //飞机的序号,                                                          ///如何换飞机，即如何分辨实在给第几架飞机生成waypoint
    double hmin = 30;      //飞机最小盘旋高度
    double h = 5,H;          //飞机高度间隔
    double v=15;            //飞机的飞行速度(单位:m/s)
    double paolenth;
    double R,pointlenth,desire_ds,ds;
    double tan_x,tan_y;
    double *add;
    //由四个角点计算中心点的程序
    //输入四个角点的值
    double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
    double j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
    //double wc,jc;
    wc = ( w1+w2+w3+w4 ) / 4;
    jc = ( j1+j2+j3+j4 ) / 4;

    //目前暂时以投弹区的中心点为wei0和jing0
    wei0 =wc;
    jing0=jc;

    cout<<"wc="<<wc<<endl;
    cout<<"jc="<<jc<<endl;

    //    double aaa[2],bbb[2];

    ros::init(argc, argv, "waypoints_node");
    ros::NodeHandle nh;


    //后期给母机发送的自己飞机的状态
    mavros_msgs::sub_plane_msgs sub_plane_state;



    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, get_waypoints);
    
    ros::Subscriber waypointsreach_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, waypoints_reached);

    ros::Subscriber currentgps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, current_gps_callback);

    //#################################接收串口发过来的消息
    ros::Subscriber receive_datasub = nh.subscribe<mavros_msgs::commander_msgs>
            ("commander_msgs", 10,commander_callback);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    //#################################发送给串口的消息
    ros::Publisher Plane_State_data_Pub = nh.advertise<mavros_msgs::sub_plane_msgs>
            ("sub_plane_msgs", 10);



    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
            ("mavros/mission/set_current");
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");
    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        cout<<"THE CONNECTION STATE OF FCU IS "<< current_state.connected <<endl;
        ros::spinOnce();
        rate.sleep();
    }



    //    //不知道是做什么用的
    //    geometry_msgs::PoseStamped pose;
    //    //send a few setpoints before starting
    //    for(int i = 20; ros::ok() && i > 0; --i){
    //        local_pos_pub.publish(pose);
    //        ros::spinOnce();
    //        rate.sleep();
    //    }


    // lat 47.397748, long 8.545596, alt 487.9
    mavros_msgs::WaypointList waypoint_list;


    //将当前的目标航点设为0号航点
    mavros_msgs::WaypointSetCurrent Current_wp;
    Current_wp.request.wp_seq=0;
    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    {
        ROS_INFO("%d", Current_wp.response.success);
        ROS_INFO("Waypoint set to 0 success");
    }


    //设定100个航点
    mavros_msgs::Waypoint waypoint[100];


    //参数表
    //改变速度 command=178  param1 = 未知 ：0/1  param2 = 速度 ：15 param3 = 未知 ：-1 param3 = 未知 ：0
    //改变速度的经纬度似乎可有可无
    //起飞 command=22  param1 = pitch : 45.0
    //飞航点 command=16  param1 = HOLD ：单位 秒
    //盘旋 command=17  param3 = 盘旋半径 ：单位 米
    // !!!!!!! 盘旋半径为正值时为顺时针盘旋,为负值时为逆时针盘旋
    /*waypoint[] 的参数表     类型  mavros_msga::Waypoint
     * .x_lat           x坐标，经度
     * .y_long          y坐标，纬度
     * .z_alt           z坐标，高度
     * .command         设置命令类型  mavros_msgs::CommandCode::NAV_TAKEOFF   mavros_msgs::CommandCode::NAV_LOITER_UNLIM
     * .frame           设置坐标系   mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT
     * .autocontinue    true    第一个点的autocontinue必设置为true()默认为false
     * .is_current      true    （除起飞点，其他的暂设为false）
     *
     * 依据command来定
     * .param1          未知 ：0/1
     * .param2          速度 ：15
     * .param3          未知 ：-1
     * .param4          未知 ：0
     *
     *
     * */





    ///设置起飞点
    double home_lat,home_long,home_x,home_y;
    double takeoff_x,takeoff_y;
    double runway_takeoff_length,runway_takeoff_angular;
    home_lat = 39.9891248;
    home_long = 116.3558232;
    runway_takeoff_length=100;
    runway_takeoff_angular=0;

    add=ll2xy(home_lat,home_long);
    home_x=add[0];
    home_y=add[1];

    takeoff_x = home_x+runway_takeoff_length*cos(runway_takeoff_angular);
    takeoff_y = home_y+runway_takeoff_length*sin(runway_takeoff_angular);
    add=xy2ll(takeoff_x,takeoff_y);

    //设置经纬高
    waypoint[0].x_lat = add[0];
    waypoint[0].y_long = add[1];
    waypoint[0].z_alt = 30;
    //设置命令类型
    waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    //第一个点的autocontinue必设置为true()默认为false
    waypoint[0].autocontinue = true;
    waypoint[0].is_current = true;
    //设置飞机的起飞爬升角(目前还有待研究)
    waypoint[0].param1 = 45.0;


    ///设置盘旋点(由比赛场地决定)



    //
    waypoint[1].x_lat = wc;
    waypoint[1].y_long = jc;

    ///盘旋点的高度 i是飞机的序号,h是飞机飞行高度差
    H = hmin + h * plane_id;
    waypoint[1].z_alt = H;
    waypoint[1].param3 = -30;                                                                 ///说明param3为盘旋半径？？
    //    waypoint[1].param4 = nan;
    waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
    //    waypoint[1].command = 31;//(LOITER_ALTITUTE) 实际最后可能还是会用上面那种
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;
    waypoint[1].is_current = false;


    WaypointPush waypoint_push;
    //cout<<"666!"<<endl;
    size=2;
    cout<<"size="<<size<<endl;
    int i=0;
    for(i = 0; i < size; i++){                                                                     //??????
        //cout<<i<<endl;
        //cout<<"777!"<<endl;
        waypoint_push.request.waypoints.push_back(waypoint[i]);
    }
    if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
    {   cout<<"888!"<<endl;
        ROS_INFO("%d", waypoint_push.response.wp_transfered);
        ROS_INFO("Waypoint push success");
    }


    std::string res;
    system("sh /home/sss/qgc_restart.sh");
    std::cout << res << '\n';


    int mission_receive_flag=1;
    int wait_flag =1;
    int trigger_flag =1;
    int seq_toudan_flag =1;

    while(ros::ok()&&(current_state.mode!="AUTO.MISSION" || !current_state.armed) ){
        ros::spinOnce();
        if(wait_flag){
            cout<<"目前还没有解锁或者到MISSION模式"<<endl<<"请稍等......"<<endl;
            wait_flag = 0;
        }
        continue;
    }


    wait_flag=1;
    if(trigger_flag){
        cout<<"开始起飞!!!"<<endl;
        trigger_flag=0;
    }


    sub_plane_state.sub_plane_id = plane_id;
    sub_plane_state.sub_plane_state = 01;
    Plane_State_data_Pub.publish(sub_plane_state);
    cout<<"已发布起飞状态!!"<<endl;

    cout<<"正在前往盘旋点!!"<<endl;

    double distance_circle;
    double current_lat,current_lon;
    double current_x,current_y;
    double waypoint1_x,waypoint1_y;

    int begin_hover_flag =0 ;


    add=ll2xy( waypoint[1].x_lat, waypoint[1].y_long );
    waypoint1_x=add[0];
    waypoint1_y=add[1];


    //判断是否接近盘旋点
    while (ros::ok()&&(!begin_hover_flag)){
        //cout<<"666!!"<<endl;
        ros::spinOnce();
        current_lat=current_gps.latitude;
        current_lon=current_gps.longitude;
        //cout<<"current_gps.latitude="<<current_gps.latitude<<endl;
        //cout<<"current_gps.longitude="<<current_gps.longitude<<endl;

        add=ll2xy(current_lat,current_lon);
        current_x=add[0];
        current_y=add[1];

        distance_circle=sqrt( (current_x-waypoint1_x)*(current_x-waypoint1_x)+
                           (current_y-waypoint1_y)*(current_y-waypoint1_y) );
        cout<<"distance_circle="<<distance_circle<<endl;
        if (distance_circle<30){

            begin_hover_flag =1 ;

        }
        ros::spinOnce();
        rate.sleep();

    }

    cout<<"已经到达盘旋点!!"<<endl;



//    //前往盘旋点
//    int hover_flag=1;
//    while(ros::ok()&&(reached_waypoints.wp_seq<1)){
//        if(hover_flag){
//            cout<<"正在前往盘旋点!!"<<endl;
//            hover_flag=0;
//        }
//        ros::spinOnce();
//        rate.sleep();
//    }


    sub_plane_state.sub_plane_id = plane_id;
    sub_plane_state.sub_plane_state = 02;
    Plane_State_data_Pub.publish(sub_plane_state);
    cout<<"发布盘旋状态!!"<<endl;


    //接受任务阶段
    //int mission_receive_flag;

    commander.commander_state=0;
    int commander_state_flag=0;


    //while( ( commander.commander_state!=3 ) ){
    while( ( commander_state_flag!=3 ) ){
        ros::spinOnce();
        if(mission_receive_flag){
            cout<<"还没有接受到任务..."<<endl;
            mission_receive_flag=0;

            //JUST FOR TEST
            cout<<"JUST FOR TEST"<<endl;
            cout<<"收到任务请输入3"<<endl;
            //cin>>commander.commander_state;
            cin>>commander_state_flag;
            //cout<<commander.commander_state;
            cout<<commander_state_flag;

        }
    }



    cout<<"开始接受任务!!"<<endl;





    double goal[2] = {0, 0};
    double bbb[2] = {0, 0};
    double tanlenth,aaa;





    goal[0]= commander.commander_latitude;
    //##此处longitude拼写有问题
    goal[1]= commander.commander_longtitude;


    //########################JUST FOR TEST
        int goal_type;
        cout << "请输入目标点的类型"<<endl;
        cin>>goal_type;

        if(goal_type==1){
            //    //目标点在盘旋园外的测试点
            goal[0]= 39.9916445;
            goal[1]= 116.3564010;
        }

        if(goal_type==2){
            //    //目标点在盘旋圆外大圆内的测试点
            goal[0]=  39.9893311;
            goal[1]= 116.3537461;
        }

        if(goal_type==3){
            //        目标点在盘旋园内的测试点
            goal[0]= 39.9889683;
            goal[1]= 116.3536919;
        }
    //    ########################JUST FOR TEST



        cout<<"goal[0]=" << goal[0]<<endl;
        cout<<"goal[1]=" << goal[1]  <<endl;


    //###################以下之后可能要注释掉
    //    while(ros::ok()&&(!mission_received)){
    //        mission_received =


    //        if(mission_receive_flag){
    //            cout<<"是否已经收到了任务?..."<<endl;
    //            mission_receive_flag=0;
    //        }
    //        cin >> mission_received;

    //    }
    //###################以上之后可能要注释掉





    //    cin>>mission_received;

    /**
//####################JUST FOR TEST##################################
//    //接受到消息之后,开始规划航迹从 waypoint[2]开始之后
//    //    cout << "mission received!!!" <<endl;
//    //    while (ros::ok() && mission_received ) {

//    int goal_input_type;
//    cout<<"请输入期望输入目标点的方式"<<endl;
//    cout<<"1 是直接输入目标点的类型 2是手动输入目标点 3是由一号侦察机发送目标点"<<endl;
//    cin >> goal_input_type;

//    if(goal_input_type==1){
//        ///目标点的坐标接收
//        int goal_type;
//        cout << "请输入目标点的类型"<<endl;
//        cin>>goal_type;

//        if(goal_type==1){
//            //    //目标点在盘旋园外的测试点
//            goal[0]= 39.9916445;
//            goal[1]= 116.3564010;
//        }

//        if(goal_type==2){
//            //    //目标点在盘旋圆外大圆内的测试点
//            goal[0]=  39.9893311;
//            goal[1]= 116.3537461;
//        }

//        if(goal_type==3){
//            //        目标点在盘旋园内的测试点
//            goal[0]= 39.9889683;
//            goal[1]= 116.3536919;
//        }
//}

//if(goal_input_type==2){
//    cout<<"请输入目标点的纬度和经度"<<endl;
//    cout<<"纬度= ";
//    cin>>goal[0];
//    cout<<"经度= ";
//    cin>>goal[1];
//}

//if(goal_input_type==3){
//    //待完成!!!!
//}



//    cout<<"goal[0]=" << goal[0]<<endl;
//    cout<<"goal[1]=" << goal[1]  <<endl;


//####################JUST FOR TEST##################################
*/

    //######################################以下是航迹规划######################################


    ///计算飞机飞行高度对应平抛距离
    paolenth = v * sqrt((2 * H) / g);

    ///计算飞机可以直接平抛的大圆半径
    desire_ds = 80;                          //在投掷之前必须先飞多少的直线
    ds = desire_ds + paolenth;
    R = sqrt( r*r + ds*ds);                        ///????
    //把纬经高转化为ENU
    add=ll2xy(wc,jc);
    xc=add[0];
    yc=add[1];
    //把纬经高转化为ENU
    add=ll2xy(goal[0],goal[1]);
    goal_x=add[0];
    goal_y=add[1];

    //至此goal的值都是正确的
    cout<<"goal[0]=" << goal[0]<<endl;
    cout<<"goal[1]=" << goal[1]  <<endl;

    pointlenth=sqrt( ( xc-goal_x )*( xc-goal_x ) +( yc-goal_y )*( yc-goal_y ));

    cout<<"pointlenth="<<pointlenth<<endl;
    double *tan;
    tan = point_tangency(goal);
    tanlenth = sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan_y));

    add=ll2xy(tan[0],tan[1]);
    tan_x=add[0];
    tan_y=add[1];
    cout<<"tan_x  ="<< tan_x <<endl;
    cout<<"tan_y  ="<< tan_y <<endl;

    double n1_x,n1_y;
    double xout,yout;




    /**
    //设置经纬高
    waypoint[0].x_lat = add[0];
    waypoint[0].y_long = add[1];
    waypoint[0].z_alt = 30;
    //设置命令类型
    waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    //第一个点的autocontinue必设置为true()默认为false
    waypoint[0].autocontinue = true;
    waypoint[0].is_current = true;
    //设置飞机的起飞爬升角(目前还有待研究)
    waypoint[0].param1 = 45.0;


    ///设置盘旋点(由比赛场地决定)



    //
    waypoint[1].x_lat = wc;
    waypoint[1].y_long = jc;

    ///盘旋点的高度 i是飞机的序号,h是飞机飞行高度差
    H = hmin + h * i;
    waypoint[1].z_alt = H;
    waypoint[1].param3 = -30;                                                                 ///说明param3为盘旋半径？？
    //    waypoint[1].param4 = nan;
    waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
    //    waypoint[1].command = 31;//(LOITER_ALTITUTE) 实际最后可能还是会用上面那种
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;
    waypoint[1].is_current = false;

*/





    if (pointlenth >= R)     //目标点在圆外且切线长足够平抛
    {   cout << "pointlenth >= R!!!" <<endl;

        /*
        //            cout<<"tan="<< tan <<endl;



        //            tan_x= ( tan[1] - jing0) * 85155.0;
        //            tan_y= ( tan[0] - wei0) * 111177.0;


        //            cout<<"tan="<< tan <<endl;
        //            cout<<"tan="<< *tan <<endl;

        //            cout<<"tan[0]="<< tan[0] <<endl;
        //            cout<<"tan[1]="<< tan[1] <<endl;


        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;
*/

        //            设置切点
        //waypoint[2].x_lat  = tan_x ;                                                                  ///这里赋值意义？？？
        //waypoint[2].y_long = tan_y ;
        //add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        /////////
        add=xy2ll(tan_x,tan_y);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];
        ////////
        //        waypoint[2].x_lat  = tan[0] ;
        //        waypoint[2].y_long = tan[1] ;

        //            cout<<"waypoint[2].x_lat=" << waypoint[2].x_lat <<endl;
        //            cout<<"waypoint[2].y_long="<< waypoint[2].y_long  <<endl;

        waypoint[2].z_alt = H;
        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;

        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;

        for (int i = 3;i <= 39; i++)
        {
            double x = tan_x + (tanlenth- paolenth)/tanlenth*(goal_x - tan_x)*(i-2)/37;
            double y = tan_y + (tanlenth- paolenth)/tanlenth*(goal_y - tan_y)*(i-2)/37;

            if(i==39){
                xout=x;
                yout=y;
            }

            add = xy2ll(x,y);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];


            //if(i<=30){
            //waypoint[i].z_alt =H-(i-2)*((H-hmin)/28) ;

            //}
            //else{
            //waypoint[i].z_alt = hmin;
            //}

            waypoint[i].z_alt = H;

            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;
        //设置投弹点
        //得到投弹点的xy坐标
        /*waypoint[3].x_lat  = goal_x-(goal_x-tan_x)/sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan_y))*paolenth;
        waypoint[3].y_long = goal_y-(goal_y-tan_y)/sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan[1]))*paolenth;

        //转化为经纬度
        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  =  add[0];
        waypoint[3].y_long =  add[1];



        //            cout<<"waypoint[3].x_lat=" << waypoint[3].x_lat <<endl;
        //            cout<<"waypoint[3].y_long="<< waypoint[3].y_long  <<endl;

        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;
        */

        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;


        //            设置目标点
        //            goal[0]= 39.9916445;
        //            goal[1]= 116.3564010;

        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;




        //            用于最后的push
        size=41;




    }

    if (pointlenth < R && pointlenth >= r)     //目标点在圆外然而切线长不够平抛
    {
        cout << "pointlenth < R 且 pointlenth >= r!!!" <<endl;

        //        tan = point_tangency(goal);


        //            point1[0] = -1*tan[0];
        //            point1[1] = -1*tan[1];
        //            aaa = (paolenth - tanlenth)/tanlenth;
        //            point2[0] = point1[0] + aaa*(tan[0] - goal[0]);
        //            point2[1] = point1[1] + aaa*(tan_[1] - goal[1]);
        //            point3[0] = tan[0] + aaa*(tan[0] - goal[0]);
        //            point3[1] = tan[1] + aaa*(tan[1] - goal[1]);


        //ENU系下的操作
        //如果中心不是00怎么办??
        cout<<"tan_x  ="<< tan_x <<endl;
        cout<<"tan_y  ="<< tan_y <<endl;

        //waypoint[2].x_lat  = -1*tan_x;
        //waypoint[2].y_long = -1*tan_y;

        n1_x = xc - tan_x;
        n1_y = yc - tan_y;


        //cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;
        //cout<<"waypoint[2].y_long  ="<< waypoint[2].y_long <<endl;
        for (int i=2;i<= 11;i++)
        {
            double x,y,x1,y1;

            x = -1*r;
            y = (2 - i)*ds/11;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;


            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }

        for (int i=12;i<=27 ;i++)
        {

            double x,y,x1,y1;

            x = cos((i-27)*pi/15)*r;
            y = -1*ds+sin((i-27)*pi/15)*r;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }

        for (int i=28;i<= 39;i++)
        {
            double x,y,x1,y1;

            x = r;
            y = (tanlenth - paolenth + ds) * (i - 27)/12 - ds;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;

            if(i==39){
                xout=x1;
                yout=y1;
            }

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;



        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;



        /*





        aaa = (paolenth - tanlenth)/tanlenth; //什么意思??? 得到了一个系数
        cout<<"aaa  ="<< aaa <<endl;

        waypoint[3].x_lat  = waypoint[2].x_lat  + aaa*(tan_x - goal_x);
        waypoint[3].y_long = waypoint[2].y_long + aaa*(tan_y - goal_y);

        cout<<"waypoint[3].x_lat  ="<< waypoint[3].x_lat <<endl;
        cout<<"waypoint[3].x_lat  ="<< waypoint[3].y_long <<endl;


        waypoint[4].x_lat  = tan_x + aaa*(tan_x - goal_x);
        waypoint[4].y_long = tan_y + aaa*(tan_y - goal_y);



        //转为LLA系
        add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];
        waypoint[2].z_alt = H;

        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;
        //            cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;


        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  = add[0];
        waypoint[3].y_long = add[1];
        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;


        add=xy2ll(waypoint[4].x_lat,waypoint[4].y_long);
        waypoint[4].x_lat  = add[0];
        waypoint[4].y_long = add[1];
        waypoint[4].z_alt = H;
        waypoint[4].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[4].autocontinue = true;
        waypoint[4].is_current = false;

        waypoint[5].x_lat  = goal[0];
        waypoint[5].y_long = goal[1];
        waypoint[5].z_alt = H;
        waypoint[5].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[5].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[5].autocontinue = true;
        waypoint[5].is_current = false;

*/
        size=41;

    }
    //pointlenth为goal到盘旋圆心的距离
    if (pointlenth < r)     //目标点在圆外且切线长足够平抛
    {
        cout << "pointlenth < r!!!" <<endl;

        // 这个判断有什么作用???
        if (pointlenth < 0.0000001)
        {
            goal_x = goal_x + 0.0000001;
            goal_y = goal_y + 0.0000001;
        }

        n1_x = goal_x - xc;
        n1_y = goal_y - yc;

        double a = r + ds + pointlenth;
        for (int i=2;i<=6 ;i++)
        {

            double x,y,x1,y1;

            x = a * (i - 2) / 5;
            y = -1*r;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        for (int i=7;i<=31 ;i++)
        {

            double x,y,x1,y1;

            x = a + cos((i-15)*pi/16)*a;
            y = ds+pointlenth+sin((i-15)*pi/16)*a;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        for (int i=32;i<=39 ;i++)
        {

            double x,y,x1,y1;

            x = 0;
            y = ds+pointlenth - desire_ds * (i - 31)/8;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            if(i==39){
                xout=x1;
                yout=y1;
            }

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;



        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;


        /*
        add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];

        waypoint[2].z_alt = H;
        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;
        //            cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;


        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  = add[0];
        waypoint[3].y_long = add[1];

        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;


        add=xy2ll(waypoint[4].x_lat,waypoint[4].y_long);
        waypoint[4].x_lat  = add[0];
        waypoint[4].y_long = add[1];

        waypoint[4].z_alt = H;
        waypoint[4].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[4].autocontinue = true;
        waypoint[4].is_current = false;



        add=xy2ll(waypoint[6].x_lat,waypoint[6].y_long);
        waypoint[6].x_lat  = add[0];
        waypoint[6].y_long = add[1];

        waypoint[6].z_alt = H;
        waypoint[6].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[6].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[6].autocontinue = true;
        waypoint[6].is_current = false;



        add=xy2ll(waypoint[5].x_lat,waypoint[5].y_long);
        waypoint[5].x_lat  = add[0];
        waypoint[5].y_long = add[1];

        waypoint[5].z_alt = H;
        waypoint[5].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[5].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[5].autocontinue = true;
        waypoint[5].is_current = false;

        //waypoint[5]和waypoint[6]起码还得再有几个点

        waypoint[7].x_lat  = goal[0];
        waypoint[7].y_long = goal[1];
        waypoint[7].z_alt = H;
        waypoint[7].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[7].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[7].autocontinue = true;
        waypoint[7].is_current = false;

*/
        //size=41;
    }
    //        break;
    //    }


    //--------------------------------------


    //############强行降低高度
    for(i=2;i<=41;i++){
        //        if(i<=30){
        //            waypoint[i].z_alt =H-(i-2)*((H-hmin)/28) ;
        //        }
        //        else
        waypoint[i].z_alt = hmin ;

    }




    //#############################
    //单独给降落点赋值
    double land_lat,land_lon;
    double land_x,land_y;

    //############################

    // [555 666];%降落点

    //add=ll2xy(land_lat,land_lon);
    double theta,dland;
    theta=0;//降落方向
    dland=300;//最后的直线长
    double land_length=20;
    land_x = home_x-land_length*cos(theta);
    land_y = home_y-land_length*sin(theta);
    add=xy2ll(land_x,land_y);
    land_lat = add[0];
    land_lon = add[1];
    //着陆点与起飞点不一致
    //    land_lat= 39.9890093;
    //    land_lon= 116.3498190;
    //    add=ll2xy(land_lat,land_lon);
    //    land_x = add[0];
    //    land_y = add[1];




    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    double erout1_x,erout1_y;
    erout1_x=(yout-goal_y)/sqrt((yout-goal_y)*(yout-goal_y)+(goal_x-xout)*(goal_x-xout))*r;
    erout1_y=(goal_x-xout)/sqrt((yout-goal_y)*(yout-goal_y)+(goal_x-xout)*(goal_x-xout))*r;
    double o_out1_x,o_out1_y;
    double o_out2_x,o_out2_y;

    //    o_out1=[goal(1)+erout1(1) goal(2)+erout1(2)];
    //    tanout3=[land(1)-dland*cos(theta) land(2)-dland*sin(theta)];
    //    o_out2=[tanout3(1)-r*sin(theta) tanout3(2)+r*cos(theta)];
    o_out1_x=goal_x+erout1_x;
    o_out1_y=goal_y+erout1_y;
    cout<<"o_out1_x="<<o_out1_x<<endl;
    cout<<"o_out1_y="<<o_out1_y<<endl;


    double tanout3_x,tanout3_y;
    tanout3_x=land_x-dland*cos(theta) ;
    tanout3_y=land_y-dland*sin(theta);

    o_out2_x=tanout3_x-r*sin(theta) ;
    o_out2_y=tanout3_y+r*cos(theta);

    



    
    //加入计算两个圆公切点的函数
    //得到tanout1,tanout2
    //    S=calculate_qiexian(r,o_out1,r,o_out2);
    //    tanout1x=S(1);
    //    tanout1y=S(2);
    //    tanout2x=S(3);
    //    tanout2y=S(4);
    //以上是得到的2个切点;

    double tanout1_x,tanout1_y,tanout2_x,tanout2_y;
    double theta_goal,theta_tan1,theta_tan2,dtheta1,dtheta2,theta_i1,theta_i2,theta_tan3;
    Point A = Point(o_out1_x, o_out1_y), B = Point(o_out2_x, o_out2_y);
    Circle AA = Circle(A, r), BB = Circle(B, r);
    int num = getTan(AA, BB, va, vb);

    tanout1_x=va[1].x;
    tanout1_y=va[1].y;

    tanout2_x=vb[1].x;
    tanout2_y=vb[1].y;


    theta_goal=atan2(goal_y-o_out1_y,goal_x-o_out1_x );
    theta_tan1=atan2(tanout1_y-o_out1_y,tanout1_x-o_out1_x);

    
    
    if (theta_tan1<theta_goal){
        theta_tan1=theta_tan1+2*pi;
    }

    dtheta1=theta_tan1-theta_goal;


    for(i=42;i<=51;i++){

        theta_i1=dtheta1*(i-41)/10+theta_goal;
        x=o_out1_x+r*cos(theta_i1);
        y=o_out1_y+r*sin(theta_i1);

        add = xy2ll(x,y);
        waypoint[i].x_lat = add[0];
        waypoint[i].y_long = add[1];
        //################################
        waypoint[i].z_alt = hmin;
        waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[i].autocontinue = true;
        waypoint[i].is_current = false;
        //################################
    }


    for(i=52;i<=61;i++){
        x=(tanout2_x-tanout1_x)*(i-51)/10+tanout1_x;
        y=(tanout2_y-tanout1_y)*(i-51)/10+tanout1_y;
        add = xy2ll(x,y);
        waypoint[i].x_lat = add[0];
        waypoint[i].y_long = add[1];
        //################################
        waypoint[i].z_alt = hmin;
        waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[i].autocontinue = true;
        waypoint[i].is_current = false;
        //################################
    }
    
    
    
    theta_tan2=atan2(tanout2_y-o_out2_y,tanout2_x-o_out2_x);
    theta_tan3=atan2(tanout3_y-o_out2_y,tanout3_x-o_out2_x);

    if (theta_tan3<theta_tan2){
        theta_tan3=theta_tan3+2*pi;
    }
    dtheta2 = theta_tan3-theta_tan2;


    for(i=62;i<=71;i++){
        theta_i2=dtheta2*(i-61)/10+theta_tan2;
        x=o_out2_x+r*cos(theta_i2);
        y=o_out2_y+r*sin(theta_i2);
        add = xy2ll(x,y);
        waypoint[i].x_lat = add[0];
        waypoint[i].y_long = add[1];
        waypoint[i].z_alt = hmin;

        waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[i].autocontinue = true;
        waypoint[i].is_current = false;
    }
    //    land_lat=39.9890093;
    //    land_lon=116.3498190;

    waypoint[72].x_lat = land_lat;
    waypoint[72].y_long = land_lon;
    waypoint[72].z_alt = 0;
    waypoint[72].command = mavros_msgs::CommandCode::NAV_LAND;
    waypoint[72].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[72].autocontinue = true;
    waypoint[72].is_current = false;

    size=73;




    ros::Time last_request = ros::Time::now();
    // while (ros::Time::now() - last_request < ros::Duration(3.0))
    // {
    //     /* code for True */
    // }



    last_request = ros::Time::now();


    //WaypointPush waypoint_push;
    // for (size_t i = 0; i < waypoint_list.waypoints.size(); i++)
    // {
    //     /* code for loop body */waypoint_push.request.waypoints.push_back()
    // }

    //points.waypoints.size()
    //cout<<"waypoint_list.waypoints.size()"<<waypoint_list.waypoints.size()<<endl;
    cout<<"666!"<<endl;


    //size=50;

    cout<<"size="<<size<<endl;




    //        mavros_msgs::WaypointClear waypoint_clear;
    //        if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
    //        {
    //            ROS_INFO("Waypoint clear success");
    //        }





    //while(ros::ok()){
    //非常重要!!!!!!
    //如果要跟换为新的点的话,那么要定义新的push
    //##################push的作用是讲目前waypointl[i]中的各点追加到waypoint中
    WaypointPush waypoint_push2;
    //ros::spinOnce();

    for(i = 0; i < size; i++){                                                                     //??????
        //cout<<i<<endl;
        //cout<<"777!"<<endl;
        waypoint_push2.request.start_index=0;
        waypoint_push2.request.waypoints.push_back(waypoint[i]);

    }




    if (waypoint_push_client.call(waypoint_push2)&&waypoint_push2.response.success)
    {   cout<<"888!"<<endl;
        ROS_INFO("%d", waypoint_push2.response.wp_transfered);
        ROS_INFO("Waypoint push success");
    }


    //######################################以上是航迹规划######################################

    //std::string res;
    system("sh /home/sss/qgc_restart.sh");
    std::cout << res << '\n';






    //######################################下面开始第二个任务阶段######################################
    //将当前的航点设置为盘旋航点
    WaypointSetCurrent waypoint_setcurrent;
    waypoint_setcurrent.request.wp_seq = 1;
    if (waypoint_setcurrent_client.call(waypoint_setcurrent)&&waypoint_setcurrent.response.success)
    {
        ROS_INFO("Waypoint set to 1 success");
    }



    double distance_tan;
    //double current_lat,current_lon;
    //double current_x,current_y;
    double waypoint2_x,waypoint2_y;
    int begin_task_flag =0 ;


    add=ll2xy( waypoint[2].x_lat, waypoint[2].y_long );
    waypoint2_x=add[0];
    waypoint2_y=add[1];


    //判断是否接近切点
    while (ros::ok()&&(!begin_task_flag)){
        //cout<<"666!!"<<endl;
        ros::spinOnce();
        current_lat=current_gps.latitude;
        current_lon=current_gps.longitude;
        //cout<<"current_gps.latitude="<<current_gps.latitude<<endl;
        //cout<<"current_gps.longitude="<<current_gps.longitude<<endl;

        add=ll2xy(current_lat,current_lon);
        current_x=add[0];
        current_y=add[1];

        distance_tan=sqrt( (current_x-waypoint2_x)*(current_x-waypoint2_x)+
                           (current_y-waypoint2_y)*(current_y-waypoint2_y) );
        cout<<"distance_tan="<<distance_tan<<endl;
        if (distance_tan<3){

            begin_task_flag =1 ;

        }
        ros::spinOnce();
        rate.sleep();

    }


    if(seq_toudan_flag&&begin_task_flag){
        Current_wp.request.wp_seq=5;
        if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
        {
            //ROS_INFO("%d", Current_wp.response.success);
            cout<<"开始执行投弹任务!!"<<endl;
        }
        seq_toudan_flag=0;
    }









    sub_plane_state.sub_plane_id = plane_id;
    sub_plane_state.sub_plane_state = 03;
    Plane_State_data_Pub.publish(sub_plane_state);
    cout<<"已发布任务状态"<<endl;




    int output_flag=1;

    while( ros::ok()&&( reached_waypoints.wp_seq<42 )){
        if(output_flag){
            cout<<"正在执行投弹任务..."<<endl;
            output_flag=0;
        }

        ros::spinOnce();
        rate.sleep();


    }



    output_flag=1;
    //降落阶段
    while( ros::ok()&&( reached_waypoints.wp_seq<72 )){
        if(output_flag){
            cout<<"投弹完成正在降落..."<<endl;
            output_flag=0;
        }
        ros::spinOnce();
        rate.sleep();

    }

output_flag=1;
    while(ros::ok()){

        //一直发布自己已经降落的消息
        sub_plane_state.sub_plane_id = plane_id;
        sub_plane_state.sub_plane_state = 04;
        Plane_State_data_Pub.publish(sub_plane_state);
        if(output_flag){
        cout<<"已发布降落状态"<<endl;
        output_flag=0;
        }
        ros::spinOnce();
        rate.sleep();


    }














    //         WaypointPull waypoint_pull;

    //         if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
    //         {
    //             ROS_INFO("%d", waypoint_pull.response.wp_received);
    //             ROS_INFO("Waypoint pull success");
    //         }






    //ros::spinOnce();
    //rate.sleep();



    //}
    //return 0;

    //    while (ros::Time::now() - last_request < ros::Duration(1.0))
    //    {
    //        /* code for True */
    //    }




    //    WaypointPull waypoint_pull;

    //    if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
    //    {
    //        ROS_INFO("%d", waypoint_pull.response.wp_received);
    //        ROS_INFO("Waypoint pull success");
    //    }

    while (ros::Time::now() - last_request < ros::Duration(1.0))
    {
        /* code for True */
    }


    // WaypointSetCurrent waypoint_setcurrent;
    // waypoint_setcurrent.request.wp_seq = 1;
    // if (waypoint_setcurrent_client.call(waypoint_setcurrent)&&waypoint_setcurrent.response.success)
    // {
    //     ROS_INFO("Waypoint setcurrent success");
    // }

    // while (ros::Time::now() - last_request < ros::Duration(1.0))
    // {
    //     /* code for True */
    // }



    //============================JUST FOR TEST=============================
    //    mavros_msgs::SetMode offb_set_mode;
    //    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    //    mavros_msgs::CommandBool arm_cmd;
    //    arm_cmd.request.value = true;

    //    set_mode_client.call(offb_set_mode);
    //    cout<<offb_set_mode.response.success<<endl<<endl;

    //=====================================================






    ////    int mission_receive_flag=1;
    ////    int wait_flag =1;
    ////    int trigger_flag =1;
    ////    int seq_toudan_flag =1;

    //////    while (ros::ok())
    //////    {
    ////        ros::spinOnce();
    ////        //cout <<"current_state.mode = "<<current_state.mode<<endl;
    ////        //cout <<"current_state.armed = "<<current_state.armed<<endl;

    ////        while(ros::ok()&&(current_state.mode!="AUTO.MISSION" || !current_state.armed) ){
    ////            ros::spinOnce();
    ////            if(wait_flag){
    ////                cout<<"目前还没有解锁或者到MISSION模式"<<endl<<"请稍等......"<<endl;
    ////                wait_flag = 0;
    ////            }
    ////            continue;
    ////        }
    ////        //ros::spinOnce();
    ////        //rate.sleep();

    //////    }

    ////    wait_flag=1;
    ////    if(trigger_flag){
    ////        cout<<"开始起飞!!!"<<endl;
    ////        trigger_flag=0;
    ////    }

    ////    while(!mission_received){
    ////        if(mission_receive_flag){
    ////            cout<<"是否已经收到了任务?..."<<endl;
    ////            mission_receive_flag=0;
    ////        }
    ////        cin >> mission_received;

    ////    }

    //    //加入判断条件,跳出盘旋
    //    //开始进入投弹程序
    //    seq_toudan_flag  = 1 ;

    //    double distance_tan;
    //    double current_lat,current_lon;
    //    double current_x,current_y;
    //    double waypoint2_x,waypoint2_y;
    //    int begin_task_flag =0 ;


    //    add=ll2xy( waypoint[2].x_lat, waypoint[2].y_long );
    //    waypoint2_x=add[0];
    //    waypoint2_y=add[1];


    //    //        //判断是否接近切点
    //    while (ros::ok()&&(!begin_task_flag)){
    //        //cout<<"666!!"<<endl;
    //        ros::spinOnce();
    //        current_lat=current_gps.latitude;
    //        current_lon=current_gps.longitude;
    //        //cout<<"current_gps.latitude="<<current_gps.latitude<<endl;
    //        //cout<<"current_gps.longitude="<<current_gps.longitude<<endl;

    //        add=ll2xy(current_lat,current_lon);
    //        current_x=add[0];
    //        current_y=add[1];

    //        distance_tan=sqrt( (current_x-waypoint2_x)*(current_x-waypoint2_x)+
    //                           (current_y-waypoint2_y)*(current_y-waypoint2_y) );
    //        cout<<"distance_tan="<<distance_tan<<endl;
    //        if (distance_tan<3){

    //            begin_task_flag =1 ;

    //        }
    //        ros::spinOnce();
    //        rate.sleep();

    //    }


    //    if(seq_toudan_flag&&begin_task_flag){
    //        Current_wp.request.wp_seq=5;
    //        if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    //        {
    //            //ROS_INFO("%d", Current_wp.response.success);
    //            cout<<"开始执行投弹任务!!"<<endl;
    //        }
    //        seq_toudan_flag=0;
    //    }



    ////开始进入降落模式
    //    int land_flag=0;
    //while(ros::ok()&&reached_waypoints.wp_seq<72){


    //    cout<<"开始执行投弹任务!!"<<endl;


    //    ros::spinOnce();
    //    rate.sleep();

    //}
    ////下降段航迹规划
    ////cout<<""




    //下降结束传回信息
    //    int land_flag =0
    //            if()



    return 0;
}
