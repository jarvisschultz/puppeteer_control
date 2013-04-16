// kalman_kinematic_control.cpp
// Jarvis Schultz
// April 17, 2012


//----------------------------------------------------------------------------
//  Notes
//  ---------------------------------------------------------------------------
//  This is a new version of the wiimote control node that allows the
//  user to drive the robots with the wiimote.  Instead of relying on
//  the wiiuse.h package, this version relies on cwiid, and the ROS
//  joystick stack.
//  ---------------------------------------------------------------------------
//  Includes
//  ---------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>
#include <wiimote/State.h>
#include <puppeteer_msgs/RobotCommands.h>

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
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "kbhit.h"


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define KTRANS (15.0)
#define KROT (5.0)
#define MULT (2.0)
#define KHEIGHT (15.0)
#define TIMEOUT (0.5) // seconds

//---------------------------------------------------------------------------
// Class Definitions
//---------------------------------------------------------------------------

class WiiControl{

private:
    int operating_condition;
    ros::NodeHandle n_;
    ros::Publisher serial_pub;
    ros::Subscriber sub;
    ros::Timer timer;
    puppeteer_msgs::RobotCommands cmd;
    wiimote::State wii_state;
    int robot_index;
    ros::Time wiitime;
    bool wii_control;
    

public:
    WiiControl() {
	ROS_DEBUG("Instantiating WiiControl Class");
	// get operating condition:
	if(ros::param::has("/operating_condition"))
	    ros::param::set("/operating_condition", 0);
	else
	{
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ros::param::set("/operating_condition", 0);
	}
	// get robot index
	if(ros::param::has("robot_index"))
	    ros::param::get("robot_index", robot_index);
	else
	{
	    ROS_WARN("Cannot Find Parameter: robot_index");
	    ros::param::set("robot_index", 0);
	}

	// define a publisher for the serial commands
	serial_pub = n_.advertise<puppeteer_msgs::RobotCommands>
	    ("serial_commands", 1);
	
	// set default values:
	cmd.robot_index = robot_index;
	cmd.div = 3;
	wii_control = true;
	wiitime = ros::Time::now();

	// Define subscriber:
	sub = n_.subscribe("/wiimote/state", 1, &WiiControl::subscriber_cb
			   , this);
	ros::Duration(0.5).sleep();
	
	// Define a timer and callback for checking system state:
	timer = n_.createTimer(ros::Duration(0.033),
			       &WiiControl::timercb, this);
    }

    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("Timer callback triggered");
	    int operating_condition = 0;
	    ros::param::getCached("/operating_condition", operating_condition);
	    ros::param::getCached("robot_index", robot_index);
	    cmd.robot_index = robot_index;
	    
	    if (kbhit()) keyboard_interpreter();

	    // check delay on wiimote:
	    ros::Duration dt = ros::Time::now()-wiitime;
	    if (dt.toSec() > TIMEOUT)
	    {
		ROS_WARN_THROTTLE(1,"wiimote timeout detected!");
		ros::param::set("/operating_condition", 4);
		cmd.type = (uint8_t) 'q';
	    }
		    

	    // If running, send command
	    if (wii_control)
	    {
		serial_pub.publish(cmd);
	    }
	    return;
	}

    // This gets called every time the estimator publishes a new robot
    // pose
    void subscriber_cb(const wiimote::State &s)
	{
	    wiitime = s.header.stamp;
	    ROS_DEBUG("Subscriber callback triggered");
	    // set_leds();

	    // if A is pressed, we are driving:
	    if(s.buttons[4])
	    {
		ROS_DEBUG("A pressed");
		float xval = s.nunchuk_joystick_zeroed[0];
		float yval = s.nunchuk_joystick_zeroed[1];
		if(fabs(yval) < 0.1) yval = 0.0;
		if(fabs(xval) < 0.1) xval = 0.0;
		float rspeed = yval*KTRANS+xval*KROT;
		float lspeed = yval*KTRANS-xval*KROT;
		if (s.buttons[5])
		{
		    rspeed *= MULT;
		    lspeed *= MULT;
		}
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'h';
		cmd.v_left = lspeed;
		cmd.v_right = rspeed;
		cmd.v_top = 0.0;
		cmd.div = 3;
		
	    }
	    // is down pressed?
	    else if (s.buttons[7])
	    {
		ROS_DEBUG("DOWN Pressed");
		float yval = s.nunchuk_joystick_zeroed[1];
		float tspeed = yval*KHEIGHT;
		if (s.buttons[5])
		    tspeed *= 2.0;
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'h';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = tspeed;
		cmd.div = 3;
		
	    }
	    // is left pressed?
	    else if (s.buttons[8])
	    {
		ROS_DEBUG("LEFT Pressed");
		float yval = s.nunchuk_joystick_zeroed[1];
		float tspeed = yval*KHEIGHT;
		if (s.buttons[5])
		    tspeed *= 2.0;
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'n';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = 0;
		cmd.v_top_left = tspeed;
		cmd.v_top_right = 0;
		cmd.div = 3;
		
	    }
	    // is right pressed?
	    else if (s.buttons[9])
	    {
		ROS_DEBUG("RIGHT Pressed");
		float yval = s.nunchuk_joystick_zeroed[1];
		float tspeed = yval*KHEIGHT;
		if (s.buttons[5])
		    tspeed *= 2.0;

		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'n';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = 0;
		cmd.v_top_left = 0;
		cmd.v_top_right = tspeed;
		cmd.div = 3;

	    }
	    // any middle buttons, set operating condition to zero
	    else if (s.buttons[2] || s.buttons[3] || s.buttons[10])
	    {
		ROS_DEBUG("IDLING");
		ros::param::set("/operating_condition", 0);
		wii_control = false;
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'h';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = 0;
		cmd.v_top_left = 0;
		cmd.v_top_right = 0;
		cmd.div = 3;
		serial_pub.publish(cmd);
	    }
	    else if (s.buttons[0] && s.buttons[1])
	    {
		ROS_DEBUG("RESUMING");
		ros::param::set("/operating_condition", 2);
		wii_control = true;
	    }
	    else if (s.nunchuk_buttons[0] && s.nunchuk_buttons[1])
	    {
		ROS_DEBUG("sending start string");
		send_start_flag();
	    }
	    // are we pushing up?
	    else if (s.buttons[6])
	    {
		ROS_DEBUG("resetting robot's pose");
		ros::param::set("/operating_condition", 0);
		wii_control = false;
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'r';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = 0;
		cmd.v_top_left = 0;
		cmd.v_top_right = 0;
		cmd.div = 3;
		serial_pub.publish(cmd);
	    }		     
	    else
	    {
		cmd.robot_index = robot_index;
		cmd.type = (uint8_t) 'h';
		cmd.v_left = 0;
		cmd.v_right = 0;
		cmd.v_top = 0;
		cmd.v_top_left = 0;
		cmd.v_top_right = 0;
		cmd.div = 3;
	    }
	    
	}

    void keyboard_interpreter(void)
	{
	    fflush(stdout);
	    char c = fgetc(stdin);
	    ROS_DEBUG("Read Character %c",c);

	    if (c == 'S')
	    {
		ROS_INFO("Sending start flag");
		send_start_flag();
	    }
	    else if (isdigit(c))
	    {
		robot_index = atoi(&c);
		ros::param::set("robot_index", robot_index);
		ROS_INFO("Robot index is now %d",robot_index);
	    }
	    else if (c == 'W')
	    {
		if (wii_control)
		    ROS_INFO("Disabling wii control!");
		else
		    ROS_INFO("Enabling wii control!");
		wii_control = !wii_control;		
	    }
	    else if (c == 'R')
	    {
		ROS_INFO("Resetting Robot!");
		// let's send the command to the robot to reset it's state
		cmd.type = (uint8_t) 'r';
		ros::param::set("/operating_condition", 0);
	    }
	    else
		ROS_WARN("Unrecognized character");

	    return;
	}

    void send_start_flag(void)
	{
	    ROS_DEBUG("Sending start flag");
	    // First set the parameters for the service call
	    cmd.robot_index = robot_index;
	    cmd.type = (uint8_t) 'm';
	    cmd.v_left = 0;
	    cmd.v_right = 0;
	    cmd.v_top = 0;
	    cmd.v_top_left = 0;
	    cmd.v_top_right = 0;
	    cmd.div = 0;

	    serial_pub.publish(cmd);
	}
};

//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ROSCONSOLE_AUTOINIT;

    // startup node
    ros::init(argc, argv, "wiimote_controller");

    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;
    ROS_INFO("Starting wiimote control node");
  
    WiiControl controller1;

    // infinite loop
    ros::spin();

    return 0;
}
