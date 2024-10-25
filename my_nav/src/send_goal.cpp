/*
 * send_goal.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: unicorn
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ctrl_msgs/command.h>

/*move_base_msgs::MoveBaseAction
 move_base在world中的目标
*/ 

geometry_msgs::PoseStamped current_pose;

ros::Publisher send_task_pub;

const float work_waypoints_table[2][12]={
{0.6 ,0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0},//横坐标X
{0,-0.5,-0.5, 0, 0, 0, 0, 0, 0, 0, 0,  0} //纵坐标Y
};

int processflag = 0;

ctrl_msgs::command get_ctrl_flag;
ctrl_msgs::command send_ctrl_flag;

void clear_flag(void)
{
    send_ctrl_flag.Land_flag = 0;
    send_ctrl_flag.Takeoff_flag = 0; 
    send_ctrl_flag.Process_flag = 0;

    send_ctrl_flag.CV_flag = 0;
    send_ctrl_flag.Finishcv_flag = 0;  
    send_ctrl_flag.SEND_flag = 0;  
    send_ctrl_flag.Finishsend_flag = 0;

    send_ctrl_flag.vx= 0.0;
    send_ctrl_flag.vy= 0.0;
    send_ctrl_flag.vz= 0.0;
    send_ctrl_flag.yaw = 0.0;
	ROS_INFO("sendgoal clear---------------------------");	
}

void task_cb(const ctrl_msgs::command::ConstPtr& msg)
{
	get_ctrl_flag = *msg;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;




bool nav_points_task(MoveBaseClient& ac,int num,move_base_msgs::MoveBaseGoal goal)//ac,goal为传入的局部变量，num为航点在数组中的位置
{
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = work_waypoints_table[0][num];
		goal.target_pose.pose.position.y = work_waypoints_table[1][num];
		goal.target_pose.pose.position.z = 1;
		goal.target_pose.pose.orientation.w = -1;
		ROS_INFO("sending---------------------------");	
        ac.sendGoal(goal);//发送多航点到nav进行控制
		ROS_INFO("overing///////////////////////////");		
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("You have reached the goal %d", num);
			clear_flag();
			send_ctrl_flag.Finishsend_flag = 1;
			send_task_pub.publish(send_ctrl_flag);//发送结束标志
			send_task_pub.publish(send_ctrl_flag);
			return true;

        } else {
            ROS_INFO("The base failed to reach goal %d", num);
			return false ;
        }
}



int main(int argc, char** argv) 
{

	int cnt = 0 ;
	ros::init(argc, argv, "send_goals_node");
	ros::NodeHandle nh;

	ros::Subscriber task_sub = nh.subscribe<ctrl_msgs::command>("task/task_pub",10,task_cb);

	ros::Publisher send_task_pub = nh.advertise<ctrl_msgs::command>("task/send_to_task",1);

	ros::Rate rate(20.0);

	MoveBaseClient ac("move_base", true);
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(40));//设定航点到达超时时间
	ROS_INFO("Connected to move base server");
	// Send a goal to move_base
	//目标的属性设置

	move_base_msgs::MoveBaseGoal goal;//定义发布速度
	
	ROS_INFO("");
	while(ros::ok())
{
	switch(processflag)
	{
	case 0:
		//默认空循环
	case 1/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,0,goal);//见形参
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		processflag = 0;
		}
		break;
	case 2/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,1,goal);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		processflag = 0;
		}
		break;
	case 3/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,2,goal);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		processflag = 0;
		}
		break;
	case 4/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,3,goal);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		processflag = 0;
		}
		break;
	case 5/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,4,goal);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		processflag = 0;
		}
		break;

	default:

		break;
	}
	ros::spinOnce();//调用回调函数

	if(get_ctrl_flag.SEND_flag != 0)//通过订阅task_node发布的信息决定飞行的哪一个航点，当不飞行航点时置0，不允许（运行空）
	{
	processflag = get_ctrl_flag.SEND_flag;
	}
}
	
return 0;

}


