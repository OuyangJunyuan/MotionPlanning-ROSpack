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

//一种online 实现方法：https://zhuanlan.zhihu.com/p/139265234
class DoubleS{
private:
    /*
     * 指令参数:    S, vs, ve, as, ae, Ts, sigma;
     * 限制参数:    Vmin, Vmax, Amin, Amax, Jmin, Jmax;
     * 规划参数:    th, tk, sk, vk, ak, jk, vk1, ak1, jk1,_Amin,_Amax,_Jmin,_Jmax;
     */
    double S, vs, ve, as, ae, Ts, sigma;
    double Vmin, Vmax, Amin, Amax, Jmin, Jmax;
    double th, tk, sk, vk, ak, jk, vk1, ak1, jk1,_Amin,_Amax,_Jmin,_Jmax,Tj2a, Tj2b, Td, hk;
    bool is_InStopPhase=false,is_AccelerationBegin=false;
    Vector3d cmdvector;
public:
    ~DoubleS() {};
    DoubleS() {};
    /*
    *cmd:   S, v0, v1, a0, a1, Ts, sigma;
    *limit: Vmin, Vmax, Amin, Amax, Jmin, Jmax;
    */
    void Configure(double S, double vs, double ve, double as, double ae, double Ts, double precision,
                   double Vmin,double Vmax,double Amin,double Amax,double Jmin,double Jmax);
    double Mapping(double now);
    double Next();
};
#include "MotionPlan.tpp"
#endif