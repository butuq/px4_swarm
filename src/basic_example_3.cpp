#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

float x,y, theta=2.512;
bool flag = 1;


void first_mov()
{
	//x = 16*sin(theta)*sin(theta)*sin(theta);
        //y = 13*cos(theta)-5*cos(2*theta)-2*cos(3*theta)-cos(4*theta);
	x = 2.5*cos(theta)*cos(theta)*cos(theta)-2;
	y = 2.5*sin(theta)*sin(theta)*sin(theta);
}

void second_mov()
{
	x = 10 + 4 * cos(-theta);
	y = 20 + 4 * sin(-theta);
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
            ("/uav2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

    

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
                }
                last_request = ros::Time::now();
            }
        }

	if(flag)
	{
		first_mov();
	}
	else
	{
		first_mov();
	}

	if (theta>6.28)
	{
		theta = 0;
		flag = !flag;
	}
	theta = theta + 0.01;
	myQuaternion.setRPY( 0, 0, theta);
	pose.pose.position.x = x;
	pose.pose.position.y = y;
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
