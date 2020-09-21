#include "MotionPlan.h"
#include <math.h>


#pragma region LFPB

/********************************************************************
ABSTRACT:	一维LFPB
INPUTS:		起始点，终止点，速度(绝对值)，加速段加速度(绝对值)，减速段加速度(绝对值)，当前时刻点
***********************************************************************/
template<typename T>
void LFPB<T>::Configure(T _bPos, T _ePos, double vel, double acc, double dec,double *t)
{

	bPos = 0;
	ePos = (_ePos - _bPos).norm();
	dir = (bPos - ePos) < 0 ? 1 : -1;

    tbPos=_bPos;
    tePos=_ePos;
	tdir = (_ePos - _bPos) / ePos;

	x = dir * (ePos - bPos);
	v = vel;
	a1 = dir == 1 ? acc : dec;
	a2 = dir == 1 ? dec : acc;

	dt_phase1 = v / a1, dt_phase3 = v / a2, dt_phase2 = 0;
	double dx_phase1 = 0.5 * a1 * dt_phase1 * dt_phase1, dx_phase3 = 0.5 * a2 * dt_phase3 * dt_phase3, dx_phase2 = 0;
	if (dx_phase1 + dx_phase3 <= x)//有匀速段。
	{
		dx_phase2 = x - dx_phase1 - dx_phase3;
		dt_phase2 = dx_phase2 / v;
	}
	else
	{
		dt_phase1 = sqrt(2 * x / (a1 + a1 * a1 / a2));
		dt_phase3 = a1 * dt_phase1 / a2;
	}
	*t = t_total = dt_phase1 + dt_phase2 + dt_phase3;
}

/*由给定时刻得到对应位移输出*/
template<typename T>
T LFPB<T>::Mapping(double now) {
    double output;
    if (now <= dt_phase1)
        output = bPos + dir * (0.5 * a1 * now * now);
    else if (now <= dt_phase1 + dt_phase2)
        output = bPos + dir * (0.5 * a1 * dt_phase1 * dt_phase1 + v * (now - dt_phase1));
    else if (now <= dt_phase1 + dt_phase2 + dt_phase3)
        output = ePos - dir * (0.5 * a2 * pow(dt_phase1 + dt_phase2 + dt_phase3 - now, 2));
    else if (now < 0)
        output = bPos;
    else
        output = ePos;

    return tbPos + output * tdir;
}
#pragma endregion

#pragma region DoubleS


#pragma endregion

#pragma region Bspline
//二分法求解属于第几个节点段u ∈ [u_i,u_i+1)
int whichSpan(double *U, double u, int m, int p)
{
    int high = m - p, low = p, mid = 0;
    if (u == U[m])
        mid = high;
    else
    {
        mid = (int)((high + low) / 2);
        while (u < U[mid] || u >= U[mid + 1])
        {
            if (u == U[mid + 1])
                mid = mid + 1;
            else
            {
                if (u > U[mid + 1])
                    low = mid;
                else
                    high = mid;
                mid = (high + low) / 2;
            }
        }
    }
    return mid;
}
//默认生成clamped节点
void gen_knot(double *knot,int m,int p)
{
    int temp = m - 2 * p;
    int i=0;
    for (; i <p; i++)                   //p         : 0     ~   p-1
        knot[i]=0;
    for (; i <= m-p; i++)               //m+1-2*p   : p     ~   m-p
        knot[i]=((i-p)*1.0f / temp);
    for (; i < m+1; i++)                //p         : m-p+1 ~   m
        knot[i]=1;


    for (int j = 0; j < m+1; ++j)
    {
        cout<<knot[j]<<",";

    }cout<<endl;
}

template<typename T>
void BSpline<T>::Configure(vector<T> &cp,int c, int _p,double _t)
{
	for (int i = 0; i < cp.size(); i++)
		ControlPoint.push_back(cp[i]);
    ControlPoint.insert(ControlPoint.begin(),c,ControlPoint.front());
    ControlPoint.insert(ControlPoint.end(),c,ControlPoint.back());
    t=_t;
	p = _p;
	n = ControlPoint.size() - 1;
	m = n + p + 1;
	DR = new double[p + 1];
	DL = new double[p + 1];
	B = new double[p + 1];
	knot = new double[m+1];
	gen_knot(knot,m,p);

}
template<typename T>
BSpline<T>::~BSpline()
{
	delete[] DR;
	delete[] B;
	delete[] DL;
	delete[] knot;
}

//也可以直接记录当前时间以免计算下标i。
template<typename T>
void BSpline<T>::gen_BasisFunc_now(double u,int i)
{
	int j, r;
	double temp, acc;
	B[0] = 1;
	for (j = 1; j <= p; j++)
	{
		DL[j] = u - knot[i + 1 - j];
		DR[j] = knot[i + j] - u;
		acc = 0;
		for (r = 0; r <= j - 1; r++)
		{
			temp = B[r] / (DR[r + 1] + DL[j - r]);
			B[r] = acc + DR[r + 1] * temp;
			acc = DL[j - r] * temp;
		}
		B[j] = acc;
	}
}
template<typename T>
T BSpline<T>::Mapping(double now)
{
    now/=t;
	T pout;
    pout.setZero();
	int i = whichSpan(knot, now, m, p);
	gen_BasisFunc_now(now,i);//更新基函数在now处初值
	for (int k = 0; k <= p; k++)
	{
		pout += ControlPoint[i - p + k] * B[k];
	}

	return pout;
}

#pragma endregion



