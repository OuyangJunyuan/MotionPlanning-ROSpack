#ifndef MOTIONPAN
#define MOTIONPAN
#include <vector>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;

typedef Matrix<double,1,1> Vector1d;
typedef Matrix<float,1,1> Vector1f;


template<typename T>
class LFPB
{
private:
    double bPos, ePos,dir;
    T tbPos,tePos,tdir;
    double dt_phase1, dt_phase2, dt_phase3, t_total;
    double x, v, a1, a2;
public:
    LFPB<T>(){};
    void Configure(T __bPos,T __ePos,double vel, double acc, double dec,double *t);
    ~LFPB() {};
    T Mapping(double now);
};

template<typename T>
class DoubleS {
private:
    /*
     * p: B样条阶数
     * m: U=[u_0,...,u_m],共m+1个节点
     * n: P=[p_0,...,p_n],共n+1个控制点
     * knot: 节点向量U集合
     * ControlPoint: P控制点集合
     */
    int p, n,m;
    double *DR, *DL, *B,*knot,t;
    vector<T> ControlPoint;
    void gen_BasisFunc_now(double u, int i);
public:
    ~DoubleS();
    DoubleS(){};
    /*
     * cp:  控制点集合
     * c :  c^n连续性，c=1起始点速度连续且0，c=2起始点加速度连续且=0，c=3起始点加加速度连续且=0，...
     */
    void Configure(vector<T> &cp,int c, int p,double t);
    T Mapping(double now);
};

template<typename T>
class BSpline {
private:
    /*
     * p: B样条阶数
     * m: U=[u_0,...,u_m],共m+1个节点
     * n: P=[p_0,...,p_n],共n+1个控制点
     * knot: 节点向量U集合
     * ControlPoint: P控制点集合
     */
    int p, n,m;
    double *DR, *DL, *B,*knot,t;
    vector<T> ControlPoint;
    void gen_BasisFunc_now(double u, int i);
public:
    ~BSpline();
    BSpline(){};
    void Configure(vector<T> &cp,int c, int p,double t);
    T Mapping(double now);
};

#include "MotionPlan.tpp"
#endif