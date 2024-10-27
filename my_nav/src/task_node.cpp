#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ctrl_msgs/command.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define USE_ENABLE    1 //选择代码类型

#define TARGET_Z      1

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;//获取当前坐标

int processflag=0;//进度flag

class task_node
{
private:
    /* data */

    ctrl_msgs::command get_msg[3];//获取消息数组
    ctrl_msgs::command ctrl;//自定义控制消息，发布接受共用 仅仅类中可用
    ros::Publisher task_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    ros::Subscriber nav_task_sub;
    ros::Subscriber send_task_sub;
    ros::Subscriber cv_task_sub;
//一些定义
    bool get_targetheight(float height);
    bool get_targetx(float x);
    bool get_targety(float y);
    
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;  // 更新当前位置信息
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg; // 更新当前控制信息
    }

    void send_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {
    get_msg[0] = *msg;//更新航点信息
    ctrl.Finishsend_flag = get_msg[0].Finishsend_flag;//只置位结束标志，不影响其他ctrl信息
    }
    void cv_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {get_msg[1] = *msg;//更新CV信息
    }
    
    void nav_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {get_msg[2] = *msg;//更新NAV信息
    }
public:
    task_node(ros::NodeHandle& nh);
    ~task_node();

    bool cv_task( int flag);
    bool send_task( int send_num);
    bool nav_land_task(void);
    bool nav_takeoff_task(void);
    bool access(int flag,float deepth);
    bool move_to_relative_position(float in_x, float in_y, float in_z);
    bool move_to_relative_head_position(float relative_x, float relative_y, float relative_z);
    bool rotate_to_yaw(float target_yaw);
    bool rotate_to_yaw_base(float target_yaw);
    bool hover(int time);
    void task_spin(void);
    void clear_flag(void);
    bool turn_true_angle();
    void pub(void)
    {task_pub.publish(ctrl);}
};

task_node::task_node(ros::NodeHandle& nh)
{
    ROS_INFO("create task");

    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &task_node::pose_cb, this);
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &task_node::state_cb, this);
    nav_task_sub = nh.subscribe<ctrl_msgs::command>("task/nav_to_task", 10, &task_node::nav_task_cb, this);
    send_task_sub = nh.subscribe<ctrl_msgs::command>("task/send_task", 10, &task_node::send_task_cb, this);
    cv_task_sub = nh.subscribe<ctrl_msgs::command>("task/cv_task", 10, &task_node::cv_task_cb, this);

	task_pub = nh.advertise<ctrl_msgs::command>("task/task_pub",1);

    ROS_INFO("creat over");

}

task_node::~task_node()
{
    ROS_INFO("end task");  
}


bool task_node::get_targetheight( float height)//高度判断函数
{
    static int cnt = 0;
    if(abs(current_pose.pose.position.z-height)<0.1 &&cnt < 7)//高度到达目标高度附近0.1m以内，且刷新5次皆在
    {cnt ++ ;}
    else{cnt /= 2 ;}
    if(cnt >=7)
    {   cnt = 0;
        return true;}
    else
    {return false;}
}

bool task_node::get_targetx(float x)//目标x判断，x为全局
{
    static int cnt = 0;
    if(abs(current_pose.pose.position.x-x)<0.1 &&cnt < 7)//现在位置距离规划前进位置在0.1m以内 且刷新5次皆在
    {cnt ++ ;}
    else{cnt /= 2;}
    if(cnt >=7)
    {   cnt = 0;
        return true;}
    else
    {return false;}
}

bool task_node::get_targety(float y)//目标y判断，y为全局
{
    static int cnt = 0;
    if(abs(current_pose.pose.position.y-y)<0.1 && cnt < 7)//现在位置距离规划前进位置在0.1m以内 且刷新5次皆在
    {cnt ++ ;}
    else{cnt /= 2;}
    if(cnt >=7)
    {   cnt = 0;
        return true;}
    else
    {return false;}
}

bool task_node::cv_task( int flag)//视觉启动函数
{    
    ctrl.CV_flag = flag;//启动视觉控制
    ctrl.SEND_flag = 0; 
    //VX不进行提供，为前进速度
    ctrl.vy = get_msg[1].vy;
    ctrl.vz = get_msg[1].vz;
    ctrl.yaw = get_msg[1].yaw;//偏航角
    ctrl.Finishcv_flag = get_msg[1].Finishcv_flag;

    task_pub.publish(ctrl);//发布ctrl消息
    ROS_INFO("pub:CV_task");
    if(ctrl.Finishcv_flag == 0){//判断结束标志
    return false;
    }
    else{
    ROS_INFO("FINISH:CV_task");
    return true;
    }

}

bool task_node::send_task(int send_num)//多航点启动函数
{

    ctrl.CV_flag = 0;
    ctrl.SEND_flag = send_num;//启动航点控制 send_num表示在第几个航点
    task_pub.publish(ctrl);

    if(ctrl.Finishcv_flag == 0){//判断结束标志
    return false;
    }
    else{
    ROS_INFO("FINISH:SEND_task");
    return true;
    }
}

bool task_node::nav_land_task(void)//降落函数，仅仅第一次有效
{
    static bool first_info = true;
    if(first_info)//仅仅输出一次
    {
        ROS_INFO("-------pub:land-------");
        first_info = false;
    }
    clear_flag();
    ctrl.Land_flag = 1;//降落指令
    task_pub.publish(ctrl);
    return true;
}

bool task_node::nav_takeoff_task(void)//起飞函数，仅仅第一次有效
{
    static bool first_info = true;
    if(first_info)//仅仅输出一次
    {
        ROS_INFO("-------pub:takeoff-------");
        first_info = false;
    }
    clear_flag();
    ctrl.Takeoff_flag = 1;//起飞指令
    task_pub.publish(ctrl);

    return get_targetheight(1.);

}

bool task_node::access(int flag, float deepth) //穿越圆环，不提供 vz, vy 版本，flag:视觉标志 deepth 穿越深度
{
    static float last_x = 0; // 设置静态变量，记录位置信息
    static float last_y = 0; // 设置静态变量，记录位置信息
    static float last_yaw = 0; // 设置静态变量，记录位置信息
    static bool first_execution = true; // 设置静态变量，标记是否是第一次执行
    clear_flag();

    ctrl.CV_flag = flag; // 启动视觉控制，自给，flagcv 的模式
    ctrl.SEND_flag = 0;

    // 第一次执行时记录当前位置
    if (first_execution)
    {
        last_x = current_pose.pose.position.x; // 记录当前位置
        last_y = current_pose.pose.position.y; // 记录当前位置
        last_yaw = tf::getYaw(current_pose.pose.orientation);
        first_execution = false; // 设置为非第一次执行
    }

    // 获取相对机头的速度
    float relative_vy = get_msg[1].vy; // 相对机头的 y 速度
    float relative_vz = get_msg[1].vz; // 相对机头的 z 速度

    // 获取当前航向角（弧度）
    float current_yaw = tf::getYaw(current_pose.pose.orientation);

    // 计算地面坐标系下的速度
    float desired_speed = 0.4; // 设置穿越速度为 0.4
    ctrl.vx = desired_speed * cos(current_yaw) - relative_vy * sin(current_yaw); // 根据航向计算 绝对 x 方向速度
    ctrl.vy = relative_vy * cos(current_yaw) + desired_speed * sin(current_yaw); // 根据航向计算 绝对 y 方向速度
    ctrl.vz = relative_vz;

    if (ctrl.Finishcv_flag == 3)
    {
        ctrl.vy = 0;
        ctrl.vz = 0;
    }

    // 计算新的目标位置
    float target_x = last_x + deepth * cos(current_yaw);
    float target_y = last_y + deepth * sin(current_yaw);

    // 检查目标位置是否达到
    if (get_targetx(target_x) && get_targety(target_y))
    {   
        first_execution = true; 
        return true;
    }
    else
    {
        task_pub.publish(ctrl); // 发布控制信息
        ROS_INFO("pub:access %f", deepth);
        return false;
    }
}

bool task_node::move_to_relative_position(float in_x, float in_y, float in_z)//绝对坐标系下,以初始方向为正方向 相对位移，z为绝对位移
{
    static float last_x = 0; 
    static float last_y = 0; 
    static float last_z = 0; 
    static bool first_execution = true; 

    //clear_flag();
    ctrl.CV_flag = 3; // 添加vx控制
    ctrl.SEND_flag = 0;

    // 第一次执行时记录当前位置
    if (first_execution)
    {
        last_x = current_pose.pose.position.x;
        last_y = current_pose.pose.position.y;
        last_z = current_pose.pose.position.z;
        first_execution = false;
    }

    // 计算当前位置与目标位置的距离
    float distance = sqrt(pow(in_x + last_x - current_pose.pose.position.x, 2) +
                          pow(in_y + last_y - current_pose.pose.position.y, 2) +
                          pow(in_z - current_pose.pose.position.z, 2));

    const float threshold = 0.12; // 到达阈值

    if (distance < threshold)
    {
        // 到达目标位置
        first_execution = true; // 重置函数
        last_x = 0; 
        last_y = 0; 
        last_z = 0;
        return true; // 到达目标位置
    }
    else
    {
        // 控制机器人朝向目标位置
        ctrl.vx = (in_x + last_x - current_pose.pose.position.x) / distance * 0.6;
        ctrl.vy = (in_y + last_y - current_pose.pose.position.y) / distance * 0.6;
        ctrl.vz = (in_z + last_z - current_pose.pose.position.z) / distance * 0.6;

        task_pub.publish(ctrl);
        ROS_INFO("Moving to position: target(%f, %f, %f) current(%f, %f, %f)", 
                 in_x, in_y, in_z, 
                 current_pose.pose.position.x, 
                 current_pose.pose.position.y, 
                 current_pose.pose.position.z);
        return false; // 还未到达目标位置
    }
}

bool task_node::move_to_relative_head_position(float relative_x, float relative_y, float relative_z) {//相对坐标系下
    // 获取当前航向角（单位为弧度）
    float current_yaw = tf::getYaw(current_pose.pose.orientation);

    // 计算目标位置的绝对坐标
    float target_x = (relative_x * cos(current_yaw)) - (relative_y * sin(current_yaw));
    float target_y = (relative_x * sin(current_yaw)) + (relative_y * cos(current_yaw));
    float target_z = relative_z; // 假设相对高度变化直接加上

    // 调用之前的函数，移动到目标位置
    return move_to_relative_position(target_x, target_y, target_z);
}

bool task_node::rotate_to_yaw(float target_yaw)//正为逆时针//绝对坐标系下
{
    //static bool first_yawtarget = true; 
    //clear_flag();

    float yaw_error = target_yaw - tf::getYaw(current_pose.pose.orientation);

    // 处理 yaw 的范围 -π 到 π
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    // 设置旋转速度
    ctrl.yaw = yaw_error * 0.3; // 旋转速度比例因子

    if(ctrl.yaw > 0.4)
    ctrl.yaw = 0.4;

    if(ctrl.yaw < -0.4)
    ctrl.yaw = -0.4;

    // 判断是否到达目标偏航
    const float yaw_threshold = M_PI / 22; // 偏航到达阈值  8

    if (abs(yaw_error) < yaw_threshold)
    {
        ctrl.yaw = 0; // 到达目标偏航，停止旋转
        task_pub.publish(ctrl);
        ROS_INFO("Reached target yaw: %f", target_yaw);
        return true; // 到达目标偏航
    }
    else
    {
        task_pub.publish(ctrl); // 发布控制信息
        ROS_INFO("Rotating: target_yaw(%f), current_yaw(%f)", 
                 target_yaw, tf::getYaw(current_pose.pose.orientation));
        return false; // 还未到达目标偏航
    }
}

bool task_node::rotate_to_yaw_base(float target_yaw)//正为逆时针//相对坐标系下
{
    static bool if_first_flag = true;
    static float last_yaw = 0;
    if(if_first_flag == true)
    {
        last_yaw = tf::getYaw(current_pose.pose.orientation);
        if_first_flag = false;
    }

    if(task_node::rotate_to_yaw(target_yaw + last_yaw))
    {
        if_first_flag = true;
        return true;
    }
    else
    {
        return false;
    }

}

bool task_node::hover(int time)//s为单位
{    
    static float last_x = 0; 
    static float last_y = 0; 
    static float last_z = 0; 
    static bool  if_first_flag = true;

    ctrl.CV_flag = 3; // 添加vx控制
    ctrl.SEND_flag = 0;

    if(if_first_flag)
        {
            ROS_INFO("HOVER--------------");
            if_first_flag = false;
            last_x = current_pose.pose.position.x;
            last_y = current_pose.pose.position.y;
            last_z = current_pose.pose.position.z;
        }

    // 计算当前位置与目标位置的距离
    float distance = sqrt(pow(last_x - current_pose.pose.position.x, 2) +
                          pow(last_y - current_pose.pose.position.y, 2) +
                          pow(last_z - current_pose.pose.position.z, 2));

    ros::Time start_time = ros::Time::now();

    if(ros::ok() && (ros::Time::now() - start_time).toSec() < time){
    // 控制机器人朝向目标位置
    ctrl.vx = (last_x - current_pose.pose.position.x) / distance * 0.4;
    ctrl.vy = (last_y - current_pose.pose.position.y) / distance * 0.4;
    ctrl.vz = (last_z - current_pose.pose.position.z) / distance * 0.4;

    task_pub.publish(ctrl);
    return false; // 还未到达目标位置
    }
    else
    {
        return true;
    }
}

void task_node::task_spin(void)
{
        ros::spinOnce();

}

void task_node::clear_flag(void)//清楚发送，慎重用
{
    ctrl.Land_flag = 0;
    ctrl.Takeoff_flag = 0; 
    ctrl.Process_flag = 0;

    ctrl.CV_flag = 0;
    ctrl.Finishcv_flag = 0;  
    ctrl.SEND_flag = 0;  
    ctrl.Finishsend_flag = 0;

    ctrl.vx= 0.0;
    ctrl.vy= 0.0;
    ctrl.vz= 0.0;
    ctrl.yaw = 0.0;
    ROS_INFO("task node clear flag--------------");
}

bool task_node::turn_true_angle(void)//当finishcv_falg 为3时候，需要逆时针转动，为4时，需要顺时针转动。为5时，停止对准（此时同时已经对准圆形）
{
    static int yaw_read = 0;
    static int cnt = 0;
    bool trusttime_flag = false;
    int mode_flag = 0;
    ctrl.Finishcv_flag = get_msg[1].Finishcv_flag;
    cnt ++;
    cnt /= 12;
    if(cnt  == 9 || cnt  == 8)
    {
        trusttime_flag = true;
        ROS_INFO("turn_true_angle--------------");
    }

    if(ctrl.Finishcv_flag >=3 || ctrl.Finishcv_flag <= 5)
    {
        switch(ctrl.Finishcv_flag) 
        {
            case 3:
            {
                if(rotate_to_yaw(yaw_read) && trusttime_flag == true)
                {
                    yaw_read += M_PI / 72;
                }
            }break;
            case 4:
            {
                if(rotate_to_yaw(yaw_read) && trusttime_flag == true)
                {
                    yaw_read -= M_PI / 72;
                }
            }break;
            default:
            {
                return true;
            }break;
        }
    }
    task_pub.publish(ctrl);
}

bool out_time_control(int time , int *processflag)
{
    static int cnt = 0;
    static int last_processflag = 0;
    if(last_processflag != *processflag)
    {
        last_processflag = *processflag;
        cnt = 0;
    }
    cnt ++ ;

    if(cnt > time * 20)
    {
        //*processflag ++;//超时进入下一操作
        *processflag = 99;//超时停止
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;
    task_node task(nh);//初始化整个类
    ros::Rate rate(20.0);

    //the setpoint publishing rate MUST be faster than 2Hz

    ROS_INFO("to connect px4");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){//等待px4回应，确保通信
    task.task_spin();
    rate.sleep();
    }

    ROS_INFO("connected px4");

//指令速写
//task.nav_takeoff_task()
//task.nav_land_task()
//task.access(1,deepth)
//task.send_task(1)
//task.cv_task(1)
//task.move_to_relative_position(float in_x, float in_y, float in_z)
//task.rotate_to_yaw(M_PI / 4)//逆时针旋转
//task.move_to_relative_head_position(float relative_x, float relative_y, float relative_z) 
//out_time_control(int time , int *processflag)超时控制函数

#if USE_ENABLE == 0     //本定义用作正式代码

while(ros::ok()){
    switch (processflag)//processflag 决定任务进程s
        {
        case 0:
            if(task.nav_takeoff_task() && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;//进入下一个线程
            }break;
        case 1:
            if(task.access(1,1) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;
            }break;
        case 2:
            if(task.rotate_to_yaw(M_PI / 2) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;    
            }break;
        case 3:
            if(task.move_to_relative_head_position(1, 0, 0) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;
            }break;            
        default:
                task.clear_flag();
                task.nav_land_task();
            break;
        }
        task.task_spin();
        rate.sleep();
    }
#endif

#if USE_ENABLE == 1   //本定义用作测试

while(ros::ok()){
    switch (processflag)//processflag 决定任务进程s
        {
        case 0:
            if(task.nav_takeoff_task() && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;//进入下一个线程
            }break;
        case 1:
            if(task.move_to_relative_head_position(1, 0, 0) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;
            }break;
        case 2:
            if(task.cv_task(1) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;    
            }break;            
        case 3:
            if(task.access(1,2.2) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;    
            }break;
        case 4:
            if(task.move_to_relative_head_position(0.2, 0.5, 0) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;
            }break;
        case 5:
            if(task.rotate_to_yaw_base(- M_PI / 6) && out_time_control(15 , &processflag))//顺时针旋转30度
            {
                task.clear_flag();
                processflag++;
            }break; 
        case 6:
            if(task.cv_task(1) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;    
            }break;            
        case 7:
            if(task.access(1,2.5) && out_time_control(15 , &processflag))
            {
                task.clear_flag();
                processflag++;    
            }break;                         
        default:
                task.clear_flag();
                task.nav_land_task();
            break;
        }
        task.task_spin();
        rate.sleep();
    }

#endif
    return 0;
}