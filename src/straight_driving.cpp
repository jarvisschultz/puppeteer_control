// PuppeteerControlMain.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011


//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>
#include <command_msgs/speed_command.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <time.h>
#include <assert.h>
#include "kbhit.h"

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Class Definitions
//---------------------------------------------------------------------------

class StraightController {

private:
    typedef struct
    {
	int RobotMY;
	float DT;
    } Control;

    int i;
    int robots_state_req;
    bool exit_flag;
    bool stop_flag;
    bool start_flag;
    Control *robot;
    ros::NodeHandle n_;
    ros::Timer timer;
    ros::ServiceClient client;
    command_msgs::speed_command srv;

 
public:
    StraightController(){
	// note: the string at the end of this command matters
	client = n_.serviceClient<command_msgs::speed_command>("send_serial_data");

	robot = GetParameters();
	    
	// create callback timer that will run with a period of DT from robot struct
	timer = n_.createTimer(ros::Duration(robot->DT), &StraightController::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& e)
	{
	    float speed = 0.0;
	    static int start_counter = 0;
	    char InputString[20];
	    // On the first timerCallback, let's send the start string:
	    if (start_counter == 0)
	    {
		ROS_INFO("Beginning movement execution\n");
		srv.request.robot_index = robot->RobotMY;
		srv.request.type = 'm';
		srv.request.Vleft = 0.0;
		srv.request.Vright = 0.0;
		srv.request.Vtop = 0.0;
		srv.request.div = 0;
		start_counter++;
	    }

	    // If we have detected a keyboard hit, let's read the hit, and send the speed out
	    // to the robot
	    if (kbhit())
	    {
		fflush(stdout);
		if ( fgets(InputString, sizeof InputString, stdin) != NULL )
		{
		    char *newline = strchr(InputString, '\n'); /* search for newline character */
		    if ( newline != NULL )
		    {
			*newline = '\0'; /* overwrite trailing newline */
		    }
		}
		
		if (InputString[0] == 's')
		{
		    ROS_INFO("Re-sending start command\n");
		    srv.request.robot_index = robot->RobotMY;
		    srv.request.type = 'm';
		    srv.request.Vleft = 0.0;
		    srv.request.Vright = 0.0;
		    srv.request.Vtop = 0.0;
		    srv.request.div = 0;
		    start_counter++;
		}
		else
		{		
		    sscanf(InputString, "%f", &speed);
		    ROS_INFO("Sending Speed String!");
		    srv.request.robot_index = robot->RobotMY;
		    srv.request.type = 'h';
		    srv.request.Vleft = speed;
		    srv.request.Vright = speed;
		    srv.request.Vtop = 0.0;
		    srv.request.div = 3;
		}
	    }
	    // send request to service
	    if(client.call(srv))
	    {
		if(srv.response.confirm_sent == 1)
		{
		    ROS_DEBUG("Send Successful: speed_command\n");
		}
		else if(srv.response.confirm_sent == 0)
		{
		    ROS_DEBUG("Send Request Denied: speed_command\n");
		    static bool request_denied_notify = true;
		    if(request_denied_notify)
		    {
			ROS_INFO("Send Requests Denied: speed_command\n");
			request_denied_notify = false;
		    }
		}
	    }
	    else 
	    {
		ROS_ERROR("Failed to call service: speed_command\n");
	    }
	    
	}

    Control *GetParameters(void)
	{
	    // Create a Control object called robot
	    Control *robot;
	    int robot_id;
	    char InputString[20];

	    size_t alloc;
	    alloc = sizeof(*robot);
	    robot = (Control*) malloc(alloc);

	    // Get keyboard input to set the robot MY:
	    ROS_INFO("Which robot are you controlling?");
	    fflush(stdout);
	    if ( fgets(InputString, sizeof InputString, stdin) != NULL )
	    {
		char *newline = strchr(InputString, '\n'); /* search for newline character */
		if ( newline != NULL )
		{
		    *newline = '\0'; /* overwrite trailing newline */
		}
		ROS_INFO("You are controlling robot %s\n", InputString);
	    }
	    sscanf(InputString, "%d", &robot_id);
	    robot->RobotMY = robot_id;

	    // Set the frequency that we want to poll the keyboard:
	    robot->DT = 0.2;
	    return(robot);
	}
};


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // startup node
    ros::init(argc, argv, "puppeteer_control");
    ros::NodeHandle n;
  
    StraightController controller1;

    // infinite loop
    ros::spin();

    return 0;
}
