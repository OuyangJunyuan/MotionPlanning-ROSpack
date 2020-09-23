#include "MotionPlan.h"
#include <math.h>
#include <iostream>


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
        cout<<knot[j]<<","<<endl;


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

#pragma region DoubleS
#define SWAP(_a,_b) do{typeof(_a) _c; _c=_a; _a=_b;_b=_c; }while(0)
void DoubleS::Configure(double so, double se, double vs, double ve, double as, double ae, double Ts, double precision,
                        double Vmin,double Vmax,double Amin,double Amax,double Jmin,double Jmax) {
    //修改后允许行程为负
    dir = so <= se ? 1 : -1;
    if (dir == -1) {
        //如果是反向运行
        Vmin *= dir;
        Vmax *= dir;
        Amin *= dir;
        Amax *= dir;
        Jmin *= dir;
        Jmax *= dir;
        vs *= dir;
        ve *= dir;
        as *= dir;
        ae *= dir;
        SWAP(Vmin, Vmax);
        SWAP(Amin, Amax);
        SWAP(Jmin, Jmax);
    }


    //重新初始化本规划器
    is_InStopPhase = false;
    is_AccelerationBegin = false;

    th = tk = sk = jk = Tj2a = Tj2b = Td = hk = 0;
    vk = vk1 = vs;
    ak = ak1 = as;
    jk1 = jk;

    //S, v0, v1, a0, a1, Ts, sigma;
    this->so = so;
    this->se = se;
    this->S = fabs(se - so);
    this->vs = vs;
    this->ve = ve;
    this->as = as;
    this->ae = ae;
    this->Ts = Ts;
    this->precision = precision;
    this->sigma = precision * Vector3d(S, ve - vs, ae - as).norm();

    //Vmin, Vmax, Amin, Amax, Jmin, Jmax;
    this->Vmin = Vmin;
    this->Vmax = Vmax;
    this->Amin = Amin;
    this->Amax = Amax;
    this->Jmin = Jmin;
    this->Jmax = Jmax;

    //根据指令调整参数
    _Amin = Amin;
    _Amax = Amax;
    _Jmin = Jmin;
    _Jmax = Jmax;
    if (ve >= Vmax) {
        //加速停止的情况交换min max 使式子统一
        SWAP(_Amin,_Amax);
        SWAP(_Jmin,_Jmax);
    }
    if (vs <= Vmax)
        is_AccelerationBegin = true;

    cmdvector = Vector3d(S, ve, ae);
}
void DoubleS::Connect(double se, double ve, double ae)
{
    Configure(so+dir*sk,se,dir*this->vk,ve,dir*this->ak,ae,Ts,precision,Vmin,Vmax,Amin,Amax,Jmin,Jmax);
}

void DoubleS::Stop(double se)
{
    Connect(se,0,0);
}
double DoubleS::Next() {
    if ((cmdvector - Vector3d(sk, vk, ak)).norm() < sigma)
    {
        return se;
    }
    if (!is_InStopPhase) {
        //还未进入停止阶段(匀速段之后) - 式子(4)(5) 与 (8)(9) 只是Jmin和Jmax互换了，这里合并优化
        Tj2a = (_Amin - ak) / _Jmin;
        Tj2b = (ae - _Amin) / _Jmax;
        Td = (ve - vk) / _Amin + Tj2a * (_Amin - ak) / (2 * _Amin) + Tj2b * (_Amin - ae) / (2 * _Amin);
        if (Td - (Tj2a + Tj2b) < 0) {
            double temp = sqrt((_Jmax - _Jmin) * (ak * ak * _Jmax - _Jmin * (ae * ae + 2 * _Jmax * (vk - ve))));
            Tj2a = -ak / _Jmin + temp / (_Jmin * (_Jmin - _Jmax));
            Tj2b = ae / _Jmax + temp / (_Jmax * (_Jmax - _Jmin));
            Td = Tj2a + Tj2b;
        }
        hk = 0.5 * ak * Td * Td +
             (double) 1.0 / 6.0 *
             (_Jmin * Tj2a * (3 * Td * Td - 3 * Td * Tj2a + Tj2a * Tj2a) + _Jmax * Tj2b * Tj2b * Tj2b) +
             Td * vk;
        if (hk >= (S - sk)) {
            is_InStopPhase = true;
            th = tk;//当前时刻就是停止阶段开始时刻
        }
    }
    if (is_InStopPhase) {
        //进入停止阶段
        //如果是减速停止的 - 式子(7)
        double delta = tk - th;
        if (delta >= 0 && delta < Tj2a)
            jk = _Jmin;
        else if (delta < Td - Tj2b)
            jk = 0;
        else
            jk = _Jmax;
    } else {
        //未进入停止阶段
        double temp1 = vk - ak * ak / (2 * Jmax), temp2 = vk - ak * ak / (2 * Jmin);
        if (!is_AccelerationBegin) {
            //起始阶段为减速阶段 - 公式(3)
            if (temp1 > Vmax) {
                if (ak > Amin)
                    jk = Jmin;
                else
                    jk = 0;
            } else {
                if (ak < 0)
                    jk = Jmax;
                else
                    jk = 0;
            }
        } else{
            //起始阶段为加速阶段 - 公式(2)
            if (temp2 < Vmax) {
                if (ak < Amax)
                    jk = Jmax;
                else
                    jk = 0;
            } else {
                if (ak > 0)
                    jk = Jmin;
                else
                    jk = 0;
            }
        }
    }
    tk+=Ts;

    ak = ak1 + Ts/2*(jk1+jk);
    vk = vk1 + Ts/2*(ak1+ak);
    sk += Ts/2*(vk1+vk);

    jk1=jk;
    ak1=ak;
    vk1=vk;

    return so+dir*sk;
}




#pragma endregion

