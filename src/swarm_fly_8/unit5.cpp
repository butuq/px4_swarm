#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

float x,y,z,theta=0;
int stage = 1;
int id = 4;
int agents = 8;
int flag = 1;
bool is_armed = 0;


void stage_1()
{
	if(!is_armed)
	{
		z = 3;
		theta = 0;
	}
	//Timer
	theta = theta + 0.08;
}

void stage_2()
{
	static bool init[2] = {0,0};
	if(theta<=3.14 && init[0] == 0)
	{
		
		init[0] = 1;
	}
	else if(theta > 3.14 && init[1] == 0)
	{
		
		init[1] = 1;
	}
	//Timer
	theta = theta + 0.08;
}

void stage_3()
{

	static bool init[8] = {0,0,0,0,0,0,0,0};
	static bool is_done = 0;
	if(is_done)
	{
		for(int i=0;i<=7;i++)
		{
			init[i] = 0;
		}
		is_done = 0;
	}
	if(theta > ((0+id)%(agents+1))*6.28/8 && theta<=((1+id)%(agents+1))*6.28/8 && init[0] == 0)
	{
		x=x+1;
		init[0] = 1;
	}
	else if(theta > ((1+id)%(agents+1))*6.28/8 && theta <= ((2+id)%(agents+1))*6.28/8 && init[1] == 0)
	{
		x=x+1;
		z=z-1;
		init[1] = 1;
	}
	else if(theta > ((2+id)%(agents+1))*6.28/8 && theta <= ((3+id)%(agents+1))*6.28/8 && init[2] == 0)
	{
		z=z-1;
		init[2] = 1;
	}
	else if(theta > ((3+id)%(agents+1))*6.28/8 && theta <= ((4+id)%(agents+1))*6.28/8 && init[3] == 0)
	{
		x=x-1;
		z=z-1;
		init[3] = 1;
	}
	else if(theta > ((4+id)%(agents+1))*6.28/8 && theta <= ((5+id)%(agents+1))*6.28/8 && init[4] == 0)
	{
		x=x-1;
		init[4] = 1;
	}
	else if(theta > ((5+id)%(agents+1))*6.28/8 && theta <= ((6+id)%(agents+1))*6.28/8 && init[5] == 0)
	{
		x=x-1;
		z++;
		init[5] = 1;
	}
	else if(theta > ((6+id)%(agents+1))*6.28/8 && theta <= ((7+id)%(agents+1))*6.28/8 && init[6] == 0)
	{
		z++;
		init[6] = 1;
	}
	else if(theta > ((7+id)%(agents+1))*6.28/8 && theta <= ((8+id)%(agents+1))*6.28/8 && init[7] == 0)
	{
		x++;
		z++;
		init[7] = 1;
	}
	//Timer
	theta = theta + 0.01;
	if(theta > 6.24)
	{
		is_done = 1;
	}
}

void stage_4()
{

}

void stage_5()
{

}

void stage_6()
{

}

mavros_msgs::State current_state;
tf2::Quaternion myQuaternion;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav4/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav4/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav4/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav4/mavros/set_mode");

    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
		    is_armed = 1;
                }
                last_request = ros::Time::now();
            }
        }
	//Stage selector
	switch(flag)
	{
		case(1):
			stage_1();
			break;
		case(2):
			stage_2();
			break;
		case(3):
			//stage_3();
			break;
		case(4):
			//stage_3();
			break;
		default:
			//stage_3();
			break;

	}


	if (theta>6.28)
	{
		theta = 0;
		flag++;

	}

	myQuaternion.setRPY( 0, 0, theta);
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	//pose.pose.orientation.x = myQuaternion[0];
	//pose.pose.orientation.y = myQuaternion[1];
	//pose.pose.orientation.z = myQuaternion[2];
	//pose.pose.orientation.w = myQuaternion[3];


	local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
