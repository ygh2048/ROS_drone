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
{0.5 ,0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0,  0},//横坐标X
{0,-0.5,-0.5, 0, 0, 0, 0, 0, 0, 0, 0,  0} //纵坐标Y
};

int process_flag[3]={0,0,0};

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

}

void task_cb(const ctrl_msgs::command::ConstPtr& msg)
{
	get_ctrl_flag = *msg;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;

move_base_msgs::MoveBaseGoal goal;


bool nav_points_task(MoveBaseClient& ac,int first,int last)
{
		for (int i = first; i < last; i++) {
        goal.target_pose.pose.position.x = work_waypoints_table[0][i];
        goal.target_pose.pose.position.y = work_waypoints_table[1][i];
		goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal %d", i);

        ac.sendGoal(goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("You have reached the goal %d", i);
            process_flag[i] = 1; // 标记为成功
			clear_flag();
			send_ctrl_flag.Finishsend_flag = 1;
			send_task_pub.publish(send_ctrl_flag);
			return true;

        } else {
            ROS_INFO("The base failed to reach goal %d", i);
            process_flag[i] = 2; // 标记为失败
			return false ;
        }
    }

}



int main(int argc, char** argv) 
{
	ros::init(argc, argv, "send_goals_node");
	ros::NodeHandle nh;

	ros::Subscriber task_sub = nh.subscribe<ctrl_msgs::command>("task/task_pub",10,task_cb);


	ros::Publisher send_task_pub = nh.advertise<ctrl_msgs::command>("task/send_to_task",1);

	ros::Rate rate(20.0);

	MoveBaseClient ac("move_base", true);
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	ROS_INFO("Connected to move base server");
	// Send a goal to move_base
	//目标的属性设置


	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	//goal.target_pose.pose.position.z = 1;
	goal.target_pose.pose.orientation.w = -1;
	ROS_INFO("");
	while(ros::ok())
{
	switch (processflag)
	{
	case 0:
		break;
	case 1/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,0,3);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		}
		break;
	case 2/* constant-expression */:
		/* code */
		if(get_ctrl_flag.SEND_flag)
		{
		ros::spinOnce();//调用回调函数
		nav_points_task(ac,0,3);
		send_task_pub.publish(send_ctrl_flag);
		clear_flag();
		}
		break;	
	default:
		break;
	}
	ros::spinOnce();//调用回调函数
	rate.sleep();


	if(get_ctrl_flag.SEND_flag != 0)
	{
	processflag = get_ctrl_flag.SEND_flag;
	}

}

	
return 0;

}