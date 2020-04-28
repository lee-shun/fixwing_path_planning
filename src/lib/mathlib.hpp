/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors  : lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime : 2020-02-14 18:03:53
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _MATHLIB_H_
#define _MATHLIB_H_
#include <math.h>
#include <iostream>

using namespace std;

#define PI 3.1415926535
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define EARTH_RADIUS 6378137
#define CONSTANTS_ONE_G 9.80665
#define rep(i,a,b) for(int i=a;i<b;i++)
#define per(i,a,b) for(int i=b-1;i>=a;i--)

//获取绝对值
float abs_num(float a)
{
    float result;
    if (a < 0)
        result = -a;

    else
        result = a;

    return result;
}

bool ISFINITE(float a)
{
    if ((abs_num(a) > 0.02) && (abs_num(a) < 1000))
    {
        return true;
    }
    else
        return false;
}

float constrain(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}
float max(const float a, const float b)
{
    return (a > b) ? a : b;
}
float min(const float a, const float b)
{
    return (a < b) ? a : b;
}

void quaternion_2_euler(float quat[4], float angle[3])
{
    // 四元数转Euler
    // q0 q1 q2 q3
    // w x y z
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}

void euler_2_quaternion(float angle[3], float quat[4])
{
    // Euler转四元数
    // q0 q1 q2 q3
    // w x y z
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1]) / 2.0);

    double sinTheta_2 = sin(double(angle[1]) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);

    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void matrix_plus_vector_3(float vector_a[3], float rotmax[3][3], float vector_b[3])
{
    vector_a[0] = rotmax[0][0] * vector_b[0] + rotmax[0][1] * vector_b[1] + rotmax[0][2] * vector_b[2];

    vector_a[1] = rotmax[1][0] * vector_b[0] + rotmax[1][1] * vector_b[1] + rotmax[1][2] * vector_b[2];

    vector_a[2] = rotmax[2][0] * vector_b[0] + rotmax[2][1] * vector_b[1] + rotmax[2][2] * vector_b[2];
}

/**
	 * create rotation matrix for the quaternion
	 */
void quat_2_rotmax(float q[4], float R[3][3])
{
    float aSq = q[0] * q[0];
    float bSq = q[1] * q[1];
    float cSq = q[2] * q[2];
    float dSq = q[3] * q[3];
    R[0][0] = aSq + bSq - cSq - dSq;
    R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = aSq - bSq + cSq - dSq;
    R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    R[2][2] = aSq - bSq - cSq + dSq;
}

float rad_2_deg(float rad)
{
    float deg;

    deg = rad * 180 / PI;

    return deg;
}

float deg_2_rad(float deg)
{
    float rad;

    rad = deg * PI / 180;

    return rad;
}

//ref,result---lat,long,alt
void cov_m_2_lat_long_alt(double ref[3], float x, float y, float z, double result[3])
{

    if (x == 0 && y == 0)
    {
        result[0] = ref[0];
        result[1] = ref[1];
    }
    else
    {
        double local_radius = cos(deg_2_rad(ref[0])) * EARTH_RADIUS; //lat是

        result[0] = ref[0] + rad_2_deg(x / EARTH_RADIUS); //得到的是lat，x是北向位置，所以在大圆上

        result[1] = ref[1] + rad_2_deg(y / local_radius); //得到的是long，在维度圆上
    }

    result[2] = ref[2] + z; //高度
}

void cov_lat_long_2_m(double a_pos[2], double b_pos[2], double m[2])
{ //参考点是a点，lat，long，alt
    double lat1 = a_pos[0];
    double lon1 = a_pos[1];

    double lat2 = b_pos[0];
    double lon2 = b_pos[1];

    double n_distance = deg_2_rad(lat2 - lat1) * EARTH_RADIUS; //涉及到ned是向北增加，且纬度向北也增加

    double r_at_ref1 = cos(deg_2_rad(lat1)) * EARTH_RADIUS;

    double e_distance = deg_2_rad(lon2 - lon1) * r_at_ref1; //涉及到ned是向东增加，但是经度向东减少

    m[0] = n_distance;
    m[1] = e_distance;
}
//将ENU(xyz)转换为经纬度(LLA)
//使用方式
//add=ll2xy(x[0],x[1]);
//x[0]=add[0];
//x[1]=add[1];
double *xy2ll(double x, double y)
{
	double LL[2];
	double wei0, jing0;
	wei0 = 39.9890143;
	jing0 = 116.353457;
	double *ret = LL;

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
	double wei0, jing0;
	wei0 = 39.9890143;
	jing0 = 116.353457;
	double *ret = XY;
	//X
	XY[0] = (j - jing0) * 85155.0;
	//Y
	XY[1] = (w - wei0) * 111177.0;

	//    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
	//    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

	return ret;

}
/*
1.精度  要学着用dcmp
2.引用  不能交换两个引用swap（）,引用中间不能变
*/
//三态函数比较;精度问题
int dcmp(double x) {
	if (fabs(x)<eps) return 0;
	return x<0 ? -1 : 1;
}
/*struct Point {
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
};*/




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
/*void printwaypoint(const mavros_msgs::WaypointList points)
{
	cout << "count:" << points.waypoints.size() << endl;
	for (size_t i = 0; i < points.waypoints.size(); i++)
	{

		cout << i << " " << points.waypoints[i].command << " ";
		cout << fixed << setprecision(10) << points.waypoints[i].x_lat << " ";
		cout << fixed << setprecision(10) << points.waypoints[i].y_long << " ";
		cout << fixed << setprecision(10) << points.waypoints[i].z_alt << endl;



	}

}*/
#endif
