		#include <ros/ros.h>
		#include <geometry_msgs/PoseStamped.h>
		#include <geometry_msgs/Twist.h>
		#include <sensor_msgs/Joy.h>
		#include <mavros_msgs/CommandBool.h>
		#include <mavros_msgs/CommandLong.h>
		#include <mavros_msgs/SetMode.h>
		#include <mavros_msgs/State.h>
		#include <mavros_msgs/PositionTarget.h>
		#include <mavros_msgs/RCIn.h>
		#include <ctrl_msgs/command.h>
	
		/*
		uint16 IGNORE_PX=1
		uint16 IGNORE_PY=2
		uint16 IGNORE_PZ=4
		uint16 IGNORE_VX=8
		uint16 IGNORE_VY=16
		uint16 IGNORE_VZ=32
		uint16 IGNORE_AFX=64
		uint16 IGNORE_AFY=128
		uint16 IGNORE_AFZ=256
		uint16 FORCE=512
		uint16 IGNORE_YAW=1024
		uint16 IGNORE_YAW_RATE=2048
		*/



	// 声明全局发布器
ros::Publisher local_pos_pub;

#define VELOCITY2D_CONTROL 0b011111000011 //选择需要的功能掩码，需要的给0,不需要的给1,从右往左对应PX PY PZ VX VY VZ AFX AFY AFZ FORCE YAW YAW_RATE
unsigned short velocity_mask = VELOCITY2D_CONTROL;

mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
// 创建 PositionTarget 消息并设置目标位置
mavros_msgs::PositionTarget landing_target;
int rc_value;

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)//遥控器通道值回调函数
	{
		rc=*msg;
		rc_value=rc.channels[4]; //订阅第五通道
	}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
	{
		current_state = *msg;
	}

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
	{
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择本地坐标
		current_goal.type_mask = velocity_mask;//速度控制的掩码
		current_goal.velocity.x = msg->linear.x;
		current_goal.velocity.y = msg->linear.y;
		current_goal.position.z = 0.6;
		current_goal.yaw_rate = msg->angular.z;
		ROS_INFO("开始导航规划");
	}

void land_cb(const ctrl_msgs::command::ConstPtr& msg )
{
    if (msg->Land_flag == 1) 
	{
        ROS_INFO("Received landing command.");
        landing_target.header.stamp = ros::Time::now();
        landing_target.header.frame_id = "map"; // 根据实际需要设置坐标系
        landing_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        landing_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                   mavros_msgs::PositionTarget::IGNORE_VY |
                                   mavros_msgs::PositionTarget::IGNORE_VZ |
                                   mavros_msgs::PositionTarget::IGNORE_AFX |
                                   mavros_msgs::PositionTarget::IGNORE_AFY |
                                   mavros_msgs::PositionTarget::IGNORE_AFZ |
                                   mavros_msgs::PositionTarget::IGNORE_YAW |
                                   mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		landing_target.type_mask = velocity_mask;//速度控制的掩码
        // 使用预设的或获取的当前坐标
        double current_x = 0; // 这里使用示例值，实际情况中应从传感器或状态获取
        double current_y = 0; // 这里使用示例值，实际情况中应从传感器或状态获取
        double landing_z = 0.0; // Z轴高度设为0，表示降落到地面

        landing_target.position.x = current_x;
        landing_target.position.y = current_y;
        landing_target.position.z = landing_z;

        ROS_INFO("Landing command published. Target position: [%.2f, %.2f, %.2f]",
                 landing_target.position.x, landing_target.position.y, landing_target.position.z);	
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;
	setlocale(LC_ALL,"");//设置中文
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	("mavros/state", 10, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	("mavros/setpoint_raw/local", 1);
	ros::Publisher vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
	("mavros/vision_pose/pose", 1);
	ros::Subscriber rc_sub=nh.subscribe<mavros_msgs::RCIn>
	("mavros/rc/in",10,rc_cb);//订阅遥控器通道pwm
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	("mavros/cmd/arming");
	ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
	("mavros/cmd/command");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	("mavros/set_mode");
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>
	("/cmd_vel", 1, twist_cb); //订阅MOVEBASE规划出来的速度
	ros::Subscriber sub = nh.subscribe<ctrl_msgs::command>
	("command_position", 1, land_cb); //订阅降落


			//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

			// wait for FCU connection
	while(ros::ok() && !current_state.connected)
		{
			ros::spinOnce();
			rate.sleep();
		}
			//------------------------------------------这一段用来起飞的，高度0.6米，过后rviz打点后直接订阅前面的速度控制回调函数
current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
current_goal.type_mask = velocity_mask;
current_goal.position.z = 0.6;
			//-------------------------------------------

mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";

mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

		/*---------------------------------------------这一段是用来强制上锁的代码，目前用不上
			mavros_msgs::CommandLong disarm_cmd;
			disarm_cmd.request.broadcast = false;
			disarm_cmd.request.command = 400;
			disarm_cmd.request.param2 = 21196; // Kill no check landed
		---------------------------------------------*/

ros::Time last_request = ros::Time::now();
			//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i)
		{
			local_pos_pub.publish(current_goal);
			ros::spinOnce();
			rate.sleep();
		}
	while(ros::ok() && rc_value>900 && rc_value<1150) //需要把遥控器通道置于900-1150之间才可以进入导航规划控制，数值的来源可以从/mavros/rc/in话题查看
		{
			if( current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
				{
				if( set_mode_client.call(offb_set_mode) &&
					offb_set_mode.response.mode_sent)
					{
					ROS_INFO("Offboard 启用");
					}
					last_request = ros::Time::now();
				} else 
				{
					if( !current_state.armed &&(ros::Time::now() - last_request > ros::Duration(5.0)))
					{
						if( arming_client.call(arm_cmd) &&
							arm_cmd.response.success)
							{
							ROS_INFO("解锁起飞");
							}
							last_request = ros::Time::now();
					}
				}
			local_pos_pub.publish(current_goal);
			//ROS_INFO("vel.x = %0.2f   vel.y = %0.2f  pos.z = %0.2f\n",current_goal.velocity.x,current_goal.velocity.y,current_goal.position.z);
			ros::spinOnce();
			rate.sleep();
		}
	return 0;
}