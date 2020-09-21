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


    int count=0;
    double dt=0.001;
    double last;
    double v;
    double v1;
    double total_t;
//    LFPB<Vector1d> mp1d;
//    mp1d.Configure(Vector1d(-1),Vector1d(1),1,1,1,&total_t);

    BSpline<Vector2d> d;
    vector<Vector2d> point;
    point.push_back(Vector2d(0,0));
    point.push_back(Vector2d(0,1));
    point.push_back(Vector2d(1,1));
    point.push_back(Vector2d(1,0));
    point.push_back(Vector2d(2,0));
    point.push_back(Vector2d(2,1));
    point.push_back(Vector2d(3,1));

    total_t=1;
    d.Configure(point,3,3,total_t);
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(count*dt>=total_t)
        {
            cout<<1<<endl;
            count=0;
        }
        Vector2d out=d.Mapping(count++*dt);
        std_msgs::Float64MultiArray msg;
        msg.data.push_back(out[0]);
        if(fabs((out[0]-last)/dt)>500)
            last=out[0];
//        msg.data.push_back((out[0]-last)/dt);
        msg.data.push_back(out[1]);


        chatter_pub.publish(msg);

        last=out[0];

        //注意spinOnce()区别:spinOnce只读取当前消息队列第一个元素调用callback后返回;spin直接堵塞，一有消息进队就调用回调，否则原地死循环。
        ros::spinOnce();
        //根据ros::Rate loop_rate(10)指定的频率，自动记录上次sleep到本次sleep之间的时间，然后休眠直到超过对应频率的延时。
        loop_rate.sleep();
    }
    return 0;
}