#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ctrl_msgs/command.h>


#define USE_ENABLE    0 //选择代码类型

#define TARGET_Z      1

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;//获取当前坐标
ros::Publisher task_pub;

int processflag=0;//进度flag

ctrl_msgs::command get_msg[3];//获取消息
ctrl_msgs::command ctrl;//自定义控制消息

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;  // 更新当前位置信息
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg; // 更新当前控制信息
}


void send_task_cb(const ctrl_msgs::command::ConstPtr& msg)
{
    get_msg[0] = *msg;//更新航点信息
    ctrl.Finishsend_flag = get_msg[0].Finishsend_flag;
}

void cv_task_cb(const ctrl_msgs::command::ConstPtr& msg)
{
    get_msg[1] = *msg;//更新CV信息

    ctrl.vx = get_msg[1].vx;
    ctrl.vy = get_msg[1].vy;
    ctrl.vz = get_msg[1].vz;
    ctrl.yaw = get_msg[1].yaw;
    ctrl.Finishcv_flag = get_msg[1].Finishcv_flag;  
}

void nav_task_cb(const ctrl_msgs::command::ConstPtr& msg)
{
    get_msg[2] = *msg;//更新NAV信息
}

void clear_flag(void)
{
    ctrl.Land_flag = 0;
    ctrl.Takeoff_flag = 0; 
    ctrl.Process_flag = 0;

    ctrl.CV_flag = 0;
    ctrl.Finishcv_flag = 0;  
    ctrl.SEND_flag = 0;  
    ctrl.Finishsend_flag = 0;

    ctrl.x= 0.0;
    ctrl.y= 0.0;
    ctrl.z= 0.0;
    ctrl.yaw = 0.0;

}



bool get_targetheight( float height)
{
    static int cnt =1;
    if(abs(current_pose.pose.position.z-height)<0.05 &&cnt < 20)
    {

        cnt ++ ;

    }
    if(cnt >=20)
    {
        return true;
    }
    else 
    {
        return false;
    }

}

bool cv_task( int flag)
{    
    ctrl.CV_flag = flag;//启动视觉控制
    ctrl.SEND_flag = 0; 


    //cv识别到的已经默认发送
    ROS_INFO("pub:CV_task");
    task_pub.publish(ctrl);
    if(ctrl.Finishcv_flag == 0){
    return false;
    }
    else{
    ROS_INFO("FINISH:CV_task");
    return true;
    }

}

int send_task( int send_num)
{
    ctrl.CV_flag = 0;
    ctrl.SEND_flag = send_num;//启动航点控制
    ROS_INFO("pub:SEND_task");
    task_pub.publish(ctrl);
    if(ctrl.Finishcv_flag == 0){
    return 0;
    }
    else{
    ROS_INFO("FINISH:SEND_task");
    return 1;
    }


}

int nav_land_task(void)
{
    clear_flag();
    ctrl.Land_flag = 1;//降落指令
    ROS_INFO("pub:land");
    task_pub.publish(ctrl);

    return 1;
}

bool nav_takeoff_task(void)
{
    clear_flag();
    ctrl.Takeoff_flag = 1;//起飞指令
    ROS_INFO("pub:takeoff");
    task_pub.publish(ctrl);

    return get_targetheight(1);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);



    ros::Subscriber send_task_sub = nh.subscribe<ctrl_msgs::command>("task/send_to_task",10,send_task_cb);
    ros::Subscriber cv_task_sub = nh.subscribe<ctrl_msgs::command>("task/cv_to_task",10,cv_task_cb);
    ros::Subscriber nav_task_sub = nh.subscribe<ctrl_msgs::command>("task/nav_to_task",10,nav_task_cb);


	ros::Publisher task_pub = nh.advertise<ctrl_msgs::command>("task/task_pub",1);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
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
            if(nav_takeoff_task())
            {
                clear_flag();
                processflag++;
            }
            break;
        case 1:
            if(cv_task(1))
            {
                clear_flag();
                processflag++;
            }
            break;
        case 2:
            if(send_task(1))
            {
                clear_flag();
                processflag++;
            }
            break;

        case 3:
            if(cv_task(1))
            {
                clear_flag();
                processflag++;
            }
            break;
        default:
            clear_flag();
            nav_land_task();
            break;
        }

        task_pub.publish(ctrl);
        ros::spinOnce();
        rate.sleep();
    }
#endif


#if USE_ENABLE == 1

int cnt = 0;
nav_takeoff_task();
int skip_flag = 0;

while (ros::ok())
{
    while(nav_takeoff_task() && ros::ok())
    {
    ros::spinOnce();
    rate.sleep();
    }
    

    cv_task(1);
    task_pub.publish(ctrl);
    ros::spinOnce();
    rate.sleep();

}
#endif


    return 0;
}