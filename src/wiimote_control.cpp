
/******INCLUDE FILES:********************************/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <puppeteer_msgs/speed_command.h>

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
#include "kbhit.h"
#include "wiiuse.h"



/******DEFINED CONSTANTS:********************************/
#define	    BAUDRATE		B115200
#define	    MODEMDEVICE		"/dev/ttyUSB0"
#define     _POSIX_SOURCE	1 /* POSIX compliant source */
#define     MAX_WIIMOTES	4
#define     PACKET_SIZE		12

/*******GLOBAL VARIABLES********************************/

class WiimoteNode{
   
private:
    int robot_index;
    int BroadcastFlag;
    ros::NodeHandle n_;
    ros::ServiceClient client;
    ros::Timer poll_timer, send_timer;
    puppeteer_msgs::speed_command srv;
    wiimote** wiimotes;
    int found, connected;

public:
    WiimoteNode() {
	robot_index = 0;
	ROS_INFO("Starting Wiimote Node...");
	// Create a client for calling the speed command service:
	client = n_.serviceClient<puppeteer_msgs::speed_command>("speed_command");
	poll_timer = n_.createTimer(ros::Duration(0.01), &WiimoteNode::polling_callback, this);
	send_timer = n_.createTimer(ros::Duration(0.05), &WiimoteNode::sending_callback, this);
	char InputString[20];

	// Let's check for the operating_condition parameter:
	if (!ros::param::has("operating_condition"))
	{
	    ROS_INFO("Setting operating_condition to IDLE");
	    ros::param::set("/operating_condition", 0);
	}
	
	if (!ros::param::has("robot_index"))
	{
	    // OK, so now we need to get the user's input as to which robot we are trying to control:
	    fputs("Which Robot Are You Controlling?: ", stdout);
	    fflush(stdout);
	    if ( fgets(InputString, sizeof InputString, stdin) != NULL )
	    {
		char *newline = strchr(InputString, '\n'); /* search for newline character */
		if ( newline != NULL )
		{
		    *newline = '\0'; /* overwrite trailing newline */
		}
		printf("You are controlling robot %s\n", InputString);
	    }

	    // Now we need to convert the user's input into an integer
	    sscanf(InputString, "%d", &robot_index);
	    // Now, let's set the global robot_index:
	    ros::param::set("/robot_index", robot_index);
	}
	else
	{
	    ros::param::get("/robot_index", robot_index);
	}
	
	wiimotes =  wiiuse_init(MAX_WIIMOTES);
	found = wiiuse_find(wiimotes, MAX_WIIMOTES, 5);
	if (!found)
	{
	    printf ("No wiimotes found.");
	    return;
	}

	connected = wiiuse_connect(wiimotes, MAX_WIIMOTES);
	if (connected)
	    printf("Connected to %i wiimotes (of %i found).\n", connected, found);
	else {
	    printf("Failed to connect to any wiimote.\n");
	    return;
	}
	wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_1);
	wiiuse_set_leds(wiimotes[1], WIIMOTE_LED_2);
	wiiuse_set_leds(wiimotes[2], WIIMOTE_LED_3);
	wiiuse_set_leds(wiimotes[3], WIIMOTE_LED_4);
	wiiuse_rumble(wiimotes[0], 1);
	wiiuse_rumble(wiimotes[1], 1);
	wiiuse_rumble(wiimotes[2], 1);
	wiiuse_rumble(wiimotes[3], 1);

	wiiuse_rumble(wiimotes[0], 0);
	wiiuse_rumble(wiimotes[1], 0);
	wiiuse_rumble(wiimotes[2], 0);
	wiiuse_rumble(wiimotes[3], 0);
    }
    
    void handle_event(struct wiimote_t* wm)
	{
	    static int operating_condition = 4;
	    ros::param::get("/operating_condition", operating_condition);
	    if (operating_condition != 2) return;	    
	    
	    ROS_INFO("\n--- EVENT [id %i] ---\n", wm->unid);

	    /* Let's provide some feedback to the user about what buttons have been pressed: */
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))		printf("A pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_B))		printf("B pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_UP))		printf("UP pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_DOWN))            printf("DOWN pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_LEFT))            printf("LEFT pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_RIGHT))           printf("RIGHT pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_MINUS))           printf("MINUS pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_PLUS))            printf("PLUS pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE))		printf("ONE pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_TWO))		printf("TWO pressed\n");
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_HOME))            printf("HOME pressed\n");
    
	    if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE) && IS_PRESSED(wm, WIIMOTE_BUTTON_TWO))
	    {
		printf("Sending Start String\n\r");
		srv.request.robot_index = robot_index;
		srv.request.type = (uint8_t) 'm';
		srv.request.Vleft = 0.0;
		srv.request.Vright = 0.0;
		srv.request.Vtop = 0.0;
		srv.request.div = 0;
	    }
	    else if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))
	    {
		// If we are pressing A, let's go ahead and collect the Analog input data
		// and print it to the screen
		if (wm->exp.type == EXP_NUNCHUK)
		{
		    /* nunchuk */
		    struct nunchuk_t* nc = (nunchuk_t*)&wm->exp.nunchuk;

		    if (IS_PRESSED(nc, NUNCHUK_BUTTON_C))	printf("Nunchuk: C pressed\n");
		    if (IS_PRESSED(nc, NUNCHUK_BUTTON_Z))	printf("Nunchuk: Z pressed\n");


		    printf("nunchuk joystick angle:     %f\n", nc->js.ang);
		    printf("nunchuk joystick magnitude: %f\n", nc->js.mag);

		    float Angle = nc->js.ang;
		    float Mag = nc->js.mag;

		    if(isnan(Angle) != 0 )
			return;

		    // Now, let's slow down the refresh rate, because we don't need to be
		    // sending so much data to the mobile robot:
		    static unsigned int counter = 0;
		    counter++;
		    if (counter%1 == 0)
		    {
			// Now, let's calculate the instructions that we want to send to
			// the robot, and then convert them to strings that we can
			// transfer out to the robot

			// First, let's calculate the "cartesian coordinates of the joystick"
			float k_trans = 15.0;
			float k_rot = 5.0;
			float yval, xval, rspeed, lspeed, tspeed;

			yval = cos((360.0-Angle)*M_PI/180.0)*Mag;
			xval = -sin((360.0-Angle)*M_PI/180.0)*Mag;

			if(fabs(yval) < 0.1) yval = 0.0;
			if(fabs(xval) < 0.1) xval = 0.0;

			printf("XValue = %f\n",xval);
			printf("YValue = %f\n",yval);

			// Now, we can convert these into angular velocities of the wheels
			rspeed = yval*k_trans-xval*k_rot;
			lspeed = yval*k_trans+xval*k_rot;

			// Turbo Mode:
			if (IS_PRESSED(wm, WIIMOTE_BUTTON_B))
			{
			    rspeed = 2.0*rspeed;
			    lspeed = 2.0*lspeed;
			}
		
			tspeed = 0.0;
		
			srv.request.robot_index = robot_index;
			srv.request.type = (uint8_t) 'h';
			srv.request.Vleft = lspeed;
			srv.request.Vright = rspeed;
			srv.request.Vtop = tspeed;
			srv.request.div = 3;
		    }
		}

	    }
	    else if(IS_PRESSED(wm, WIIMOTE_BUTTON_DOWN))
	    {
		if (wm->exp.type == EXP_NUNCHUK)
		{
		    /* nunchuk */
		    struct nunchuk_t* nc = (nunchuk_t*)&wm->exp.nunchuk;

		    if (IS_PRESSED(nc, NUNCHUK_BUTTON_C))	printf("Nunchuk: C pressed\n");
		    if (IS_PRESSED(nc, NUNCHUK_BUTTON_Z))	printf("Nunchuk: Z pressed\n");


		    printf("nunchuk joystick angle:     %f\n", nc->js.ang);
		    printf("nunchuk joystick magnitude: %f\n", nc->js.mag);

		    float Angle = nc->js.ang;
		    float Mag = nc->js.mag;

		    // Now, let's slow down the refresh rate, because we don't need to be
		    // sending so much data to the mobile robot:
		    static unsigned int counter = 0;
		    counter++;
		    if (counter%1 == 0)
		    {
			// Now, let's calculate the instructions that we want to send to
			// the robot, and then convert them to strings that we can
			// transfer out to the robot

			// First, let's calculate the "cartesian coordinates of the joystick"
			float k_trans = 15.0;
			float yval, tspeed;

			yval = cos((360.0-Angle)*M_PI/180.0)*Mag;

			if(fabs(yval) < 0.1) yval = 0.0;

			printf("YValue = %f\n",yval);

			// Now, we can convert these into angular velocities of the wheels
			tspeed = yval*k_trans;

			// Turbo Mode:
			if (IS_PRESSED(wm, WIIMOTE_BUTTON_B))
			{
			    tspeed = 2.0*tspeed;
			}

			// Now we can assemble the data strings to send to the PIC32
			srv.request.robot_index = robot_index;
			srv.request.type = (uint8_t) 'h';
			srv.request.Vleft = 0.0;
			srv.request.Vright = 0.0;
			srv.request.Vtop = tspeed;
			srv.request.div = 3;
		    }
		}
	    }
	    else
	    {
		srv.request.robot_index = robot_index;
		srv.request.type = (uint8_t) 'h';
		srv.request.Vleft = 0.0;
		srv.request.Vright = 0.0;
		srv.request.Vtop = 0.0;
		srv.request.div = 3;
	    }
	}

    void handle_read(struct wiimote_t* wm, byte* data, unsigned short len)
	{
	    int i = 0;

	    printf("\n\n--- DATA READ [wiimote id %i] ---\n", wm->unid);
	    printf("finished read of size %i\n", len);
	    for (; i < len; ++i)
	    {
		if (!(i%16))
		    printf("\n");
		printf("%x ", data[i]);
	    }
	    printf("\n\n");
	}

    void handle_ctrl_status(struct wiimote_t* wm)
	{
	    printf("\n\n--- CONTROLLER STATUS [wiimote id %i] ---\n", wm->unid);

	    printf("attachment:      %i\n", wm->exp.type);
	    printf("speaker:         %i\n", WIIUSE_USING_SPEAKER(wm));
	    printf("ir:              %i\n", WIIUSE_USING_IR(wm));
	    printf("leds:            %i %i %i %i\n", WIIUSE_IS_LED_SET(wm, 1), WIIUSE_IS_LED_SET(wm, 2), WIIUSE_IS_LED_SET(wm, 3), WIIUSE_IS_LED_SET(wm, 4));
	    printf("battery:         %f %%\n", wm->battery_level);
	}

    void handle_disconnect(wiimote* wm)
	{
	    printf("\n\n--- DISCONNECTED [wiimote id %i] ---\n", wm->unid);
	}

    void KeyboardInterpreter(void)
	{
	    char InputString[20];
	    // OK, so now we need to get the user's input as to which robot we are trying to control:
	    fflush(stdout);
	    if ( fgets(InputString, sizeof InputString, stdin) != NULL )
	    {
		char *newline = strchr(InputString, '\n'); /* search for newline character */
		if ( newline != NULL )
		{
		    *newline = '\0'; /* overwrite trailing newline */
		}
	    }

	    if(strncmp(InputString,"1",1)==0 || strncmp(InputString,"2",1)==0 || strncmp(InputString,"3",1)==0)
	    {
		// So here, we are just switching which robot we are controlling:
		printf("You are controlling robot %s\n", InputString);
		sscanf(InputString, "%d", &robot_index);
		BroadcastFlag = 0;
		ros::param::set("/robot_index", robot_index);
	    }
	    else if(strncmp(InputString,"a",1)==0)
	    {
		printf("You are contolling all of the robots:");
		// Set the xbee address to broadcast mode:
		BroadcastFlag = 1;
		robot_index = 9;
		ros::param::set("/robot_index", robot_index);
	    }
	}

    void sending_callback(const ros::TimerEvent& e)
	{
	    static int operating_condition;
	    ros::param::get("/operating_condition", operating_condition);
	    if (operating_condition != 2) return;	    
	    
	    // Call service to send data to the robot:
	    if(client.call(srv))
	    {
		if(srv.response.error == false)
		{
		    ROS_DEBUG("Send Successful: speed_command\n");
		}
		else
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

    void polling_callback(const ros::TimerEvent& e)
	{
	    // Check for keyboard inputs:
	    if (kbhit()) KeyboardInterpreter();
	    if (wiiuse_poll(wiimotes, MAX_WIIMOTES))
	    {
		/*
		 *	This happens if something happened on any wiimote.
		 *	So go through each one and check if anything happened.
		 */
		int i = 0;
		for (; i < MAX_WIIMOTES; ++i) {
		    switch (wiimotes[i]->event) {
		    case WIIUSE_EVENT:
			/* a generic event occured */
			handle_event(wiimotes[i]);
			break;

		    case WIIUSE_STATUS:
			/* a status event occured */
			handle_ctrl_status(wiimotes[i]);
			break;

		    case WIIUSE_DISCONNECT:
		    case WIIUSE_UNEXPECTED_DISCONNECT:
			/* the wiimote disconnected */
			handle_disconnect(wiimotes[i]);
			break;

		    case WIIUSE_READ_DATA:
			/*
			 *	Data we requested to read was returned.
			 *	Take a look at wiimotes[i]->read_req
			 *	for the data.
			 */
			break;

		    case WIIUSE_NUNCHUK_INSERTED:
			/*
			 *	a nunchuk was inserted
			 *	This is a good place to set any nunchuk specific
			 *	threshold values.  By default they are the same
			 *	as the wiimote.
			 */
			//wiiuse_set_nunchuk_orient_threshold((struct nunchuk_t*)&wiimotes[i]->exp.nunchuk, 90.0f);
			//wiiuse_set_nunchuk_accel_threshold((struct nunchuk_t*)&wiimotes[i]->exp.nunchuk, 100);
			printf("Nunchuk inserted.\n");
			break;

		    case WIIUSE_CLASSIC_CTRL_INSERTED:
			printf("Classic controller inserted.\n");
			break;

		    case WIIUSE_GUITAR_HERO_3_CTRL_INSERTED:
			/* some expansion was inserted */
			handle_ctrl_status(wiimotes[i]);
			printf("Guitar Hero 3 controller inserted.\n");
			break;

		    case WIIUSE_NUNCHUK_REMOVED:
		    case WIIUSE_CLASSIC_CTRL_REMOVED:
		    case WIIUSE_GUITAR_HERO_3_CTRL_REMOVED:
			/* some expansion was removed */
			handle_ctrl_status(wiimotes[i]);
			printf("An expansion was removed.\n");
			break;

		    default:
			break;
		       
		    }
		}
	    }

	    /*
	     *	Disconnect the wiimotes
	     */
	    // wiiuse_up(wiimotes, MAX_WIIMOTES);
	}
};


    int main(int argc, char** argv)
	{
	    // Create a ROS Node:
	    ros::init(argc, argv, "wiimote_node");

	    WiimoteNode wiimote;
    
	    // ROS Infinite Loop:
	    ros::spin();

	    return 0;
	}
	
