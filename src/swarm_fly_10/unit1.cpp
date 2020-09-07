#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#define ID 		0

//Strings
#define STRING1 "/uav0/mavros/state"
#define STRING2 "/uav0/mavros/setpoint_position/local"
#define STRING3 "/uav0/mavros/cmd/arming"
#define STRING4 "/uav0/mavros/set_mode"

//Stage Parameters
#define INITIAL_ALTITUDE	5
#define DELTA_TIME			3

float x,y,z,theta=0;
int stage = 1;
int id = ID;
int agents = 8;
int flag = 1;
double delta_time = DELTA_TIME;

// //Prepare for circle
// void stage_4()
// {
// 	switch(id)
// 	{
// 		case(0):
// 			break;
// 		case(1):
// 			break;
// 		case(2):
// 			break;
// 		case(3):
// 			break;
// 		case(4):
// 			break;
// 		case(5):
// 			break;
// 		case(6):
// 			break;
// 		case(7):
// 			break;
//		case(8):
// 			break;
// 		case(9):
// 			break;
// 	}
// }


//From the ground to initial altitude
void stage_1()
{
	delta_time = DELTA_TIME*2;
	switch(id)
	{
		case(0):
			z=z+INITIAL_ALTITUDE;
			//z=z+4;
			break;
		case(1):
			z=z+INITIAL_ALTITUDE;
			break;
		case(2):
			z=z+INITIAL_ALTITUDE;
			break;
		case(3):
			z=z+INITIAL_ALTITUDE;
			break;
		case(4):
			z=z+INITIAL_ALTITUDE;
			break;
		case(5):
			z=z+INITIAL_ALTITUDE;
			break;
		case(6):
			z=z+INITIAL_ALTITUDE;
			break;
		case(7):
			z=z+INITIAL_ALTITUDE;
			break;
		case(8):
			z=z+INITIAL_ALTITUDE;
			break;
		case(9):
			z=z+INITIAL_ALTITUDE;
			break;

	}
}

//Bow & arrow
void stage_2()
{
	delta_time = DELTA_TIME*1.07;
	switch(id)
	{
		case(0):
			break;
		case(1):
			z=z+2;
			break;
		case(2):
			z=z+1;
			break;
		case(3):
			break;
		case(4):
			z=z-2;
			x=x-3;
			break;
		case(5):
			z=z-1;
			x=x-3;
			break;
		case(6):
			x=x-2;
			break;
		case(7):
			x=x-1;
			break;
		case(8):
			z=z+1;
			x=x-3;
			break;
		case(9):
			z=z-1;
			x=x-4;
			break;
	}
}


//Bow prepares to shoot
void stage_3()
{
	switch(id)
	{
		case(0):
			x=x+3;
			break;
		case(1):			
			break;
		case(2):
			break;
		case(3):
			x=x+1;		
			break;
		case(4):			
			break;
		case(5):
			break;
		case(6):
			x=x+1;
			break;
		case(7):
			x=x+1;
			break;
		case(8):
			x=x+1;
			break;
		case(9):
			x=x+1;
			break;
	}
}

//Bow shoot
void stage_4()
{
	switch(id)
	{
		case(0):
			x=x+2;
			break;
		case(1):			
			break;
		case(2):
			break;
		case(3):
			x=x+2;		
			break;
		case(4):			
			break;
		case(5):
			break;
		case(6):
			x=x+2;
			break;
		case(7):
			x=x+2;
			break;
		case(8):
			x=x+2;
			break;
		case(9):
			x=x+2;
			break;
	}
}

//Bow to wall 1
void stage_5()
{
	delta_time = DELTA_TIME*1.5;
	switch(id)
	{
		case(0):
			x=x+1;
			break;
		case(1):	
			z=z+1;
			x=x+13;		
			break;
		case(2):
			z=z+1;
			x=x+12;	
			break;
		case(3):
			x=x+1;		
			break;
		case(4):
			z=z-1;
			x=x+13;			
			break;
		case(5):
			z=z-1;
			x=x+12;	
			break;
		case(6):
			x=x+1;
			break;
		case(7):
			x=x+1;
			break;
		case(8):
			x=x+1;
			break;
		case(9):
			x=x+1;
			break;
	}
}

//Bow to wall 2
void stage_6()
{
	delta_time = DELTA_TIME*1.5;
	switch(id)
	{
		case(0):
			x=x+1;
			break;
		case(1):	
			z=z-1;	
			break;
		case(2):
			z=z-1;	
			break;
		case(3):
			x=x+1;		
			break;
		case(4):
			z=z+2;			
			break;
		case(5):
			z=z+2;	
			break;
		case(6):
			x=x+1;
			break;
		case(7):
			x=x+1;
			break;
		case(8):
			x=x+1;
			break;
		case(9):
			x=x+1;
			break;
	}
}

//Prepare for impact
void stage_7()
{
	delta_time = DELTA_TIME*1.07;
	switch(id)
	{
		case(0):
			x=x+2;
			break;
		case(1):			
			break;
		case(2):
			break;
		case(3):
			x=x+2;		
			break;
		case(4):			
			break;
		case(5):
			break;
		case(6):
			x=x+2;
			break;
		case(7):
			x=x+2;
			break;
		case(8):
			x=x+2;
			break;
		case(9):
			x=x+2;
			break;
	}
}

//Impact
void stage_8()
{
	//delta_time = DELTA_TIME*1.07;
	switch(id)
	{
		case(0):
			x=x+2;
			break;
		case(1):
			y=y-2;
			z=z+1;
			x=x-12;
			break;
		case(2):
			y=y-2;
			z=z+2;
			x=x-6;
			break;
		case(3):
			x=x+2;		
			break;
		case(4):
			y=y-2;
			z=z-3;
			x=x-12;			
			break;
		case(5):
			y=y-2;
			z=z-2;
			x=x-6;	
			break;
		case(6):
			x=x+2;
			break;
		case(7):
			y=y+2;
			z=z+4;
			x=x-8;
			break;
		case(8):
			x=x+2;
			break;
		case(9):
			x=x+2;
			break;
	}
}

//Impact 2
void stage_9()
{
	//delta_time = DELTA_TIME*1.07;
	switch(id)
	{
		case(0):
			x=x+2;
			break;
		case(1):
			y=y+2;
			break;
		case(2):
			y=y+2;
			break;
		case(3):
			x=x+2;		
			break;
		case(4):
			y=y+2;		
			break;
		case(5):
			y=y+2;	
			break;
		case(6):
			x=x+2;
			break;
		case(7):
			y=y-2;
			x=x-2;
			break;
		case(8):
			x=x-8;
			break;
		case(9):
			x=x-10;
			break;
	}
}

//Impact 3
void stage_10()
{
	//delta_time = DELTA_TIME*1.07;
	switch(id)
	{
		case(0):
			x=x+2;
			break;
		case(1):
			break;
		case(2):
			break;
		case(3):
			x=x+2;		
			break;
		case(4):		
			break;
		case(5):	
			break;
		case(6):
			x=x+2;
			break;
		case(7):
			break;
		case(8):
			break;
		case(9):
			break;
	}
}



//Prepare for circle
// void stage_4()
// {
// 	switch(id)
// 	{
// 		case(0):
// 			break;
// 		case(1):
// 			break;
// 		case(2):
// 			break;
// 		case(3):
// 			break;
// 		case(4):
// 			break;
// 		case(5):
// 			break;
// 		case(6):
// 			break;
// 		case(7):
// 			break;
//		case(8):
// 			break;
// 		case(9):
// 			break;
// 	}
// }

mavros_msgs::State current_state;
tf2::Quaternion myQuaternion;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
	char buffer[64];
	char b[32];
	std::string a;
    ros::init(argc, argv, "offb_node");
	a=ros::this_node::getName();
	sprintf(b,"%s",a.c_str());
    char c; 
    int i,digit,number=0; 
    for(i=0;i<strlen(b);i++) 
    { 
    	c = b[i]; 
    	if(c>='0' && c<='9') //to confirm it's a digit 
    	{ 
    		digit = c - '0'; 
    		number = number*10 + digit; 
    	} 
    } 
	number--;
	id=number;

    ros::NodeHandle nh;

	sprintf(buffer,"%s",STRING1);
	buffer[4]=id+'0';
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (buffer, 10, state_cb);
	sprintf(buffer,"%s",STRING2);
	buffer[4]=id+'0';
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (buffer, 10);
	sprintf(buffer,"%s",STRING3);
	buffer[4]=id+'0';
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (buffer);
	sprintf(buffer,"%s",STRING4);
	buffer[4]=id+'0';
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (buffer);

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
	{
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


    while(ros::ok())
	{
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
		else 
		{
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
                    ROS_INFO("Vehicle armed");
					ROS_INFO("%d",id);
                }
                last_request = ros::Time::now();
            }
        }
	//Stage selector
	if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(delta_time)))
	{
		switch(flag)
		{
			case(1):
				stage_1();
				ROS_INFO("Stage1");
				break;	
			case(2):
				stage_2();
				ROS_INFO("Stage2");
				break;
			case(3):
				stage_3();
				ROS_INFO("Stage3");
				break;
			case(4):
				stage_4();
				ROS_INFO("Stage4");
				break;
			case(5):
				stage_5();
				ROS_INFO("Stage5");
				break;
			case(6):
				stage_6();
				ROS_INFO("Stage6");
				break;
			case(7):
				stage_7();
				ROS_INFO("Stage7");
				break;
			case(8):
				//stage_8();
				ROS_INFO("Stage5");
				break;
			case(9):
				//stage_9();
				ROS_INFO("Stage3");
				break;
			case(10):
				//stage_10();
				ROS_INFO("Stage4");
				break;
			case(11):
				//stage_11();
				ROS_INFO("Stage5");
				break;
			default:
				//stage_3();
				ROS_INFO("def");
				break;

		}
		flag++;

	last_request = ros::Time::now();
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
