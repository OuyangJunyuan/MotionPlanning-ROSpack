# MotionPlanning-ROSpack
LFPF/Bspline/DoubleS for motor                                                         																												   ——  ouyjy@HITSZ-WTR

这是一个轨迹规划的库

注：如果参数不合适，则LFPB和DoubleS可能无法物理上实现规划，故可能无解。

---

```
Vector1d/Vector2d/Vector3d/Vector4d
```

### LFPB(梯形规划)

LFPB：当规划量为位移时，速度为梯形故为梯形规划。当然规划量为速度时，其加速度是连续的。

​			故LFPB是规划量一阶连续，二阶冲击的。

本库：只完成两点间LFPB。当规划量是多维的，规划两点间欧式距离为LFPB的(笛卡尔坐标规划)。适合用在一维的电机平滑中，计算快，但多条LFPB衔接处有加速度冲击。

```c++
//多维LFPB
LFPB<Vector2d> lfpb;
//配置：起始点，终止点，速度，最大加速度，运行总时间(运行这段轨迹必须花费的时间，是规划器计算的参数)
lfpb.Configure(Vector2d(0,0),Vector2d(1,1),velocity,acceleration,&total_time);

Vector2d output;
//得到now(0~total_time)时刻对应的输出值
output = lfpb.Mapping(now);

```



### B-Spline(B样条曲线)

https://blog.csdn.net/tuqu/article/details/4749586

B-Spline：可以做到多维度多(控制)点任意阶连续性轨迹规划，必经过起止控制点，ViaPoint视控制点重复次数(比如1,1,1,1则1是4次重复度一维控制点)。

​				   缺点就是计算比较复杂一点。且速度、加速度、加加速度无法进行限制。



本库：默认使用Clamped B-Spline，即起止控制点都经过。且可以设置曲线的连续性。

```c++
    vector<Vector2d> points;
    points.push_back(Vector2d(0,0));
    points.push_back(Vector2d(1,0));
    points.push_back(Vector2d(1,1));
    points.push_back(Vector2d(2,1));
    points.push_back(Vector2d(2,0));

    BSpline<Vector2d> bspline;
	//控制点集合，C3连续性(加加速度连续)，3次B样条曲线，总时间(是用户指定需要多长时间来走完轨迹)。
    bspline.Configure(points,3,3,total_time);
    Vector2d output =bspline.Mapping(now);
```



### DoubleS(双S曲线规划)

https://zhuanlan.zhihu.com/p/139265234

Double S：即规划量是路程的时候，加加速度是梯形，加加加速度冲击。更平滑，无启动冲击。

TODO：现在是单轴(维/自由度)规划，可以增加笛卡尔空间下规划的功能(和LFPB一样规划模长)。

* 在线积分规划算法(本程序所用)：
  * 缺点：每次只能计算下次的输出且只能向前计算，无法给出任意时刻对应的位移。也无法设置和规划设置时候就计算出总时长。且有累计误差
  * 优点：但是可以任意设置 起止边界的位移、速度、加速度值。
* 离线计算法：
  * 缺点：只能设置起始止界位移、速度，加速度是固定为0的。
  * 直接可以得出整个规划的过程，可以给出总时间，提供任意时刻对应的输出的方法。

​				



```c++
//极值设置：最大小速度、加速度、加加加速度
double Vmin=-20,Vmax=20,Amin=-20,Amax=20,Jmin=-30,Jmax=30;
double S=-25, vs=3, ve=-3, as=3  , ae=-3, Ts=0.001, sigma=0.005;

DoubleS ds;
//起始点，终止点，起止速度，起止加速度，积分间隔Ts(一般为0.001)，sigma(误差容许取0.05~0.005),
ds.Configure(0,S,vs,ve,as,ae,Ts,sigma,Vmin,Vmax,Amin,Amax,Jmin,Jmax);

//进行下一次积分累加，并得出下一个Ts间隔后的输出值。
double output=ds.Next()；
//以当前位移、速度、加速度为起始值，x,xx,xxx为终止位移、速度、加速度进行下一个DoubleS规划来进行曲线衔接
ds.Connect(x,xx,xxx);
//以se为停止目标，0为终止速度与加速度进行平滑刹车。
ds.Stop(se);
```

图一：折现为目标点连线图，规划路径为曲线，开头上翘是因为起始速度是3。

图四：这个在线积分算法会有加加速度在匀速期间震荡的现象，影响好像不大。

可以看出，除了加加加速度有冲击之外，加、加加速度都是连续的。

![image-20200923210341562](https://upload-images.jianshu.io/upload_images/15852708-413d678fb3eebd76.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)