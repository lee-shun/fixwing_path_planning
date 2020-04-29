#ifndef _CIRCLE_HPP_
#define _CIRCLE_HPP_
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
#include "mathlib.hpp"
#include "vector.hpp"

struct Circle {
    Point c;
    double r;
    Circle(Point c, double r) : c(c), r(r) {}
    Point getpoint(double a) { return Point(c.x + cos(a) * r, c.y + sin(a) * r); }
};

/*
   求圆的公切线
   */
int getTan(Circle A, Circle B, Point *va, Point *vb) {
    int cnt = 0;
    if (A.r < B.r) {
        swap(A, B);
        swap(va, vb);
    }
    double d = (A.c - B.c).len();
    double rdif = A.r - B.r, rsum = A.r + B.r;
    //内含，没有公切线
    if (dcmp(d - rdif) < 0)
        return 0;
    //内切，有一条公切线
    double base = atan2(B.c.y - A.c.y, B.c.x - A.c.x);
    if (dcmp(d) == 0 && dcmp(A.r - B.r) == 0)
        return -1;
    if (dcmp(d - rdif) == 0) {
        va[cnt] = A.getpoint(base);
        vb[cnt] = B.getpoint(base);
        cnt++;
        return cnt;
    }
    //一定有两条外公切线
    double th = acos((A.r - B.r) / d);
    va[cnt] = A.getpoint(base + th);
    vb[cnt] = B.getpoint(base + th);
    cnt++;
    va[cnt] = A.getpoint(base - th);
    vb[cnt] = B.getpoint(base - th);
    cnt++;
    //可能有一条公切线
    if (dcmp(d - rsum) == 0) {
        va[cnt] = A.getpoint(base);
        vb[cnt] = B.getpoint(base + PI);
        cnt++;
    } else if (dcmp(d - rsum) > 0) {
        double th2 = acos((A.r + B.r) / d);
        va[cnt] = A.getpoint(base + th2);
        vb[cnt] = B.getpoint(base + th2 + PI);
        cnt++;
        va[cnt] = A.getpoint(base - th2);
        vb[cnt] = B.getpoint(base - th2 + PI);
        cnt++;
    }
    return cnt;
}
#endif
