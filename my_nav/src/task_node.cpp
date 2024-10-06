#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ctrl_msgs/command.h>


#define USE_ENABLE    1 //选择代码类型

#define TARGET_Z      1

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;//获取当前坐标

int processflag=0;//进度flag



class task_node
{
private:
    /* data */

    ctrl_msgs::command get_msg[3];//获取消息
    ctrl_msgs::command ctrl;//自定义控制消息
    ros::Publisher task_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    ros::Subscriber nav_task_sub;
    ros::Subscriber send_task_sub;
    ros::Subscriber cv_task_sub;

    bool get_targetheight(float height);
    bool get_targetx(float x);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;  // 更新当前位置信息
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg; // 更新当前控制信息
    }
    void nav_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {get_msg[2] = *msg;//更新NAV信息
    }
    void send_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {
    get_msg[0] = *msg;//更新航点信息
    ctrl.Finishsend_flag = get_msg[0].Finishsend_flag;
    }
    void cv_task_cb(const ctrl_msgs::command::ConstPtr& msg)
    {get_msg[1] = *msg;//更新CV信息
    ctrl.vx = get_msg[1].vx;
    ctrl.vy = get_msg[1].vy;
    ctrl.vz = get_msg[1].vz;
    ctrl.yaw = get_msg[1].yaw;
    ctrl.Finishcv_flag = get_msg[1].Finishcv_flag;}
    
public:
    task_node(ros::NodeHandle& nh);
    ~task_node();

    bool cv_task( int flag);
    bool send_task( int send_num);
    int nav_land_task(void);
    bool nav_takeoff_task(void);
    bool access(int flag,int deepth);
    void task_spin(void);
    void clear_flag(void);
    void pub(void)
{
    task_node::task_pub.publish(ctrl);
}
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


bool task_node::get_targetheight( float height)
{
    static int cnt = 0;
    if(abs(current_pose.pose.position.z-height)<0.08 &&cnt < 7)
    {
    cnt ++ ;
    }
    if(cnt >=7)
    {
        cnt = 0;
        return true;
    }
    else
    {
        return false;
    }
}

bool task_node::get_targetx(float x)
{
    static int cnt = 0;
    if(abs(current_pose.pose.position.x-x)<0.15 &&cnt < 5)
    {
    cnt ++ ;
    }
    if(cnt >=5)
    {
        cnt = 0;
        return true;
    }
    else
    {
        return false;
    }
}

bool task_node::cv_task( int flag)
{    
    ctrl.CV_flag = flag;//启动视觉控制
    ctrl.SEND_flag = 0; 
    //cv识别到的已经默认发送
    task_pub.publish(ctrl);
    ROS_INFO("pub:CV_task");
    if(ctrl.Finishcv_flag == 0){
    return false;
    }
    else{
    ROS_INFO("FINISH:CV_task");
    return true;
    }

}

bool task_node::send_task(int send_num)
{
    ctrl.CV_flag = 0;
    ctrl.SEND_flag = send_num;//启动航点控制
    task_pub.publish(ctrl);
    ROS_INFO("pub:SEND_task");
    if(ctrl.Finishcv_flag == 0){
    return false;
    }
    else{
    ROS_INFO("FINISH:SEND_task");
    return true;
    }
}

int task_node::nav_land_task(void)
{
    clear_flag();
    ctrl.Land_flag = 1;//降落指令
    task_pub.publish(ctrl);
    ROS_INFO("pub:land");
    return 1;
}

bool task_node::nav_takeoff_task(void)
{
    clear_flag();
    ctrl.Takeoff_flag = 1;//起飞指令
    task_pub.publish(ctrl);
    ROS_INFO("pub:takeoff");


    return get_targetheight(1.);

}

bool task_node::access(int flag,int deepth)
{
    static float last_x =  0;
    static int flag_x = 0;
    clear_flag();

    ctrl.CV_flag = flag;//启动视觉控制   自给
    ctrl.SEND_flag = 0;

    if(flag)
    {
        flag_x = 1;
    } 

    if(flag_x)
    {
        flag_x = 0;
        last_x = current_pose.pose.position.x;
    }
    ctrl.vz = 0;
    ctrl.vy = 0;
    ctrl.vx = 0.6;

    ROS_INFO("pub:access");
    if(get_targetx(last_x + deepth))
    return true;
    else
    return false;

}
void task_node::task_spin(void)
{
        ros::spinOnce();

}

void task_node::clear_flag(void)
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
    ROS_INFO("clear flag");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;
    task_node task(nh);
    ros::Rate rate(20.0);

    //the setpoint publishing rate MUST be faster than 2Hz

    ROS_INFO("to connect px4");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
    task.task_spin();
    rate.sleep();
    }

    ROS_INFO("connected px4");
/** 
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    ros::Time last_request = ros::Time::now();
*/

#if USE_ENABLE == 0

while(ros::ok()){
    switch (processflag)
        {
        case 0:
            if(task.nav_takeoff_task();)
            {
                task.clear_flag();
                processflag++;
            }
            break;
        case 1:
            if(task.send_task(1))
            {
                task.clear_flag();
                processflag++;
            }
            break;
        case 2:
            if(task.access(1,1))
            {
                task.clear_flag();
                processflag++;
            }
        default:
                task.clear_flag();
                task.nav_land_task();
            break;
        }
        task.task_spin();
        rate.sleep();
    }
#endif

#if USE_ENABLE == 1

int cnt = 0;
task.nav_takeoff_task();
int skip_flag = 0;

while (ros::ok())
{
    while(!task.nav_takeoff_task() && ros::ok())
    {
        task.task_spin();
        rate.sleep();
    }
    
    task.nav_land_task();
    task.task_spin();
    rate.sleep();

}
#endif
    return 0;
}