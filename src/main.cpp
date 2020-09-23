#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "MotionPlan.h"

int main(int argc, char **argv)
{
    //第三个参数(基本名字即不能带'/')为节点名，除非在launch中的<node ... name="xxx"/>重新指定
    //rosrun pack <exename> 由CMakeList指定
    //rosnode list 中出现的是 <nodename> 由第三个参数
    ros::init(argc, argv, "talker");
    //作为和ROS通信的主要途径,可以有多个Handle：在第一个Handle被完全初始化本节点；在最后销毁一个的Handle来完全关闭节点
    ros::NodeHandle n;
    //发布到的话题名，在本节点名称空间下/nodename/topicname，第二个参数为消息队列大小，还可以指定话题被接受者连接与断开连接回调函数。
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("MP", 1000);


    double Vmin=-20,Vmax=20,Amin=-20,Amax=20,Jmin=-30,Jmax=30;
    double S=-25, vs=3, ve=-3, as=3  , ae=-3, Ts=0.001, sigma=0.005;

    double dt=0.001;
    double last=0,v=vs,v1=vs,a=as,a1=as,j=0;


    DoubleS ds;
    ds.Configure(0,S,vs,ve,as,ae,Ts,sigma,Vmin,Vmax,Amin,Amax,Jmin,Jmax);

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        double output=ds.Next();

        msg.data.push_back(output);

        v=(output-last)/dt;
        if(fabs(v)>fabs(Vmax))
            v=v1;
        msg.data.push_back(v);
        a=(v-v1)/dt;
        if(fabs(a)>fabs(Amax))
            a=a1;
        msg.data.push_back(a);
        j=(a-a1)/dt;
        if(fabs(j)>fabs(Jmax))
            j=Jmax;
        msg.data.push_back(j);

        last=output;
        v1=v;
        a1=a;
        chatter_pub.publish(msg);


        //注意spinOnce()区别:spinOnce只读取当前消息队列第一个元素调用callback后返回;spin直接堵塞，一有消息进队就调用回调，否则原地死循环。
        ros::spinOnce();
        //根据ros::Rate loop_rate(10)指定的频率，自动记录上次sleep到本次sleep之间的时间，然后休眠直到超过对应频率的延时。
        loop_rate.sleep();
    }
    return 0;
}