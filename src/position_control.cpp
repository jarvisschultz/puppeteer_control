// kinematic_controller.cpp
// Jarvis Schultz
// July 11, 2011


//----------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This program is for 3D trajectory following for a single puppeteer
// using a kinematic controller.  It subscribes to a version of the
// estimator node that is driven by a timer.  It then determines the
// correct controls to send to the robot via a service call to the
// serial node.

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>

#include <puppeteer_msgs/speed_command.h>
#include <puppeteer_msgs/RobotPose.h>

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


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define DWHEEL	(0.07619999999999)
#define DPULLEY	(0.034924999999999998)
#define WIDTH	(0.1323340)
#define MAX_ANG_VEL  (60.0)
std::string filename;

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}       

//---------------------------------------------------------------------------
// Class Definitions
//---------------------------------------------------------------------------

class KinematicControl{

private:
    typedef struct
    {
	int RobotMY;
	float DT;
	unsigned int num;
	float vals[][5]; // unknown length; x,y,z,wd,thd 
    } Trajectory;       

    int operating_condition;
    Trajectory *traj;
    ros::NodeHandle n_;
    ros::ServiceClient client;
    // ros::Subscriber sub;
    ros::Timer timer;
    puppeteer_msgs::speed_command srv;
    puppeteer_msgs::RobotPose pose;
    bool start_flag;
    float desired_x, desired_y, desired_th;
    unsigned int num;
    
public:
    KinematicControl() {
	// Initialize necessary variables:
	if(ros::param::has("operating_condition"))
	    // set operating_condition to idle so the robot doesn't drive
	    ros::param::set("operating_condition", 0);
	else
	{
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ros::param::set("operating_condition", 0);
	}
	
	// Read in the trajectory:
	traj = ReadControls(filename);

	// Define service client:
	client = n_.serviceClient<puppeteer_msgs::speed_command>("speed_command");
	// Define subscriber:
	// sub = n_.subscribe("/robot_pose", 1, &KinematicControl::subscriber_cb, this);
	// Define timer:
	timer = n_.createTimer(ros::Duration(0.033), &KinematicControl::subscriber_cb, this);

	// Send a start flag:
	send_start_flag();
	
    }

    // This gets called every time the estimator publishes a new robot
    // pose
    void subscriber_cb(const ros::TimerEvent& e)
	{
	    static double running_time = 0.0;
	    static ros::Time base_time;
	    ros::param::get("/operating_condition", operating_condition);

	    ROS_DEBUG("Pose callback triggered");
	    
	    if (operating_condition == 0 || operating_condition == 1 ||
		operating_condition == 3)
	    {
		ROS_DEBUG("A non-runnning condition");
		start_flag = true;
		return;
	    }
	    else if (operating_condition == 2)
	    {
		ROS_DEBUG("Running Condition");
		if (start_flag == true)
		{
		    ROS_INFO("Beginning movement execution");

		    // set parameters for sending initial pose
		    srv.request.robot_index = traj->RobotMY;
		    srv.request.type = 'l';
		    srv.request.Vleft = traj->vals[0][1];
		    srv.request.Vright = traj->vals[0][2];
		    srv.request.Vtop = atan2(traj->vals[1][2]-traj->vals[0][2],
					     traj->vals[1][1]-traj->vals[0][1]);
		    srv.request.div = 4;

		    start_flag = false;
		    base_time = ros::Time::now();
		    
		}
		else
		{
		    ROS_DEBUG("Getting values to send");
		    // we will run the regular control loop
		    // let's first get the expected pose at the given time:
		    running_time = ((ros::Time::now()).toSec()-
				    base_time.toSec());
		    // check that running_time is less than the final time:
		    if (running_time <= traj->vals[num-1][0])
		    {
			ROS_DEBUG("Getting desired pose and control values");
			get_desired_pose(running_time);
		    }
		    else
		    {
			// stop robot!
			ROS_INFO("Trajectory Finished!");
			srv.request.robot_index = traj->RobotMY;
			srv.request.type = 'h';
			srv.request.Vleft = 0.0;
			srv.request.Vright = 0.0;
			srv.request.Vtop = 0.0;
			srv.request.div = 3;
			// set operating_condition to stop
			ros::param::set("operating_condition", 3);
			start_flag = true;
		    }
		}
	    }
	    else if (operating_condition == 4)
	    {
		ROS_WARN_THROTTLE(5,"Emergency Stop Detected!");
		srv.request.robot_index = traj->RobotMY;
		srv.request.type = 'h';
		srv.request.Vleft = 0.0;
		srv.request.Vright = 0.0;
		srv.request.Vtop = 0.0;
		srv.request.div = 3;
		start_flag = true;
	    }

	    ROS_DEBUG("Calling service");
	    // send request to service
	    if(client.call(srv))
	    {
		if(srv.response.error == false)
		    ROS_DEBUG("Send Successful: speed_command\n");
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
		ROS_ERROR("Failed to call service: speed_command\n");
	}

    void get_desired_pose(float time)
	{
	    // This function reads through the trajectory array and
	    // interpolates the desired pose of the robot at the
	    // current operating time

	    // first we iterate through the array to find the right
	    // time entry
	    unsigned int index;
	    float mult;
	    for (index=0; index<num; index++)
	    {
		if (traj->vals[index][0] > time)
		    break;
	    }
	    mult = (time-traj->vals[index-1][0])/traj->DT;
	    desired_x = (traj->vals[index-1][1]) +
		mult*(traj->vals[index][1]-traj->vals[index-1][1]);
	    desired_y = (traj->vals[index-1][2]) +
		mult*(traj->vals[index][2]-traj->vals[index-1][2]);
	    // now, let's estimate the desired orientation to do this
	    // we just draw a straight line from the current point to
	    // the next point
	    desired_th = atan2(traj->vals[index][2]-desired_y,
			       traj->vals[index][1]-desired_x);
	    while (desired_th <= -M_PI)
		desired_th += 2.0*M_PI;
	    while (desired_th > M_PI)
		desired_th -= 2.0*M_PI;

	    ROS_INFO("Time = %f", time);
	    ROS_INFO("Xd = %f\tYd = %f\tTd = %f\t",desired_x, desired_y, desired_th);

	    // Set service parameters:
	    srv.request.robot_index = traj->RobotMY;
	    srv.request.type = 'k';
	    srv.request.Vleft = time;
	    srv.request.Vright = desired_x;
	    srv.request.Vtop = desired_y;
	    srv.request.div = 4;
	    
	    return;
	}

    void send_start_flag(void)
	{
	    // First set the parameters for the service call
	    srv.request.robot_index = traj->RobotMY;
	    srv.request.type = 'm';
	    srv.request.Vleft = 0.0;
	    srv.request.Vright = 0.0;
	    srv.request.Vtop = 0.0;
	    srv.request.div = 0;

	    // send request to service
	    if(client.call(srv))
	    {
		if(srv.response.error == false)
		    ROS_DEBUG("Send Successful: speed_command\n");
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
		ROS_ERROR("Failed to call service: speed_command\n");
	}


    Trajectory *ReadControls(std::string filename)
	{
	    unsigned int i,j;
	    float temp_float, xd, xdd, yd, ydd, xdp, ydp;
	    std::string line, temp;
	    Trajectory *traj;
	    std::ifstream file;
	    file.open(filename.c_str(), std::fstream::in);
	    // Read line telling us the number of data points:
	    getline(file, line);
	    std::stringstream ss(line);
	    ss >> temp >> num;
	    ROS_DEBUG("Number of time points = %d",num);

	    // Now we can initialize the trajectory struct:
	    size_t alloc;
	    alloc = sizeof(*traj) + sizeof(traj->vals[0])*num;
	    traj = (Trajectory*) malloc(alloc);
	    
	    // Now, we can start reading in the important file stuff:
	    for (i=0; i<num; i++)
	    {
		for (j=0; j<3; j++)
		{		
		    getline(file, line, ',');
		    std::stringstream ss(line);
		    ss >> temp_float;
		    traj->vals[i][j] = temp_float;
		}
		getline(file, line);
	    }
	    file.close();
	    
	    // Now we can set DT and the robot_index
	    ros::param::get("/robot_index", traj->RobotMY);
	    traj->DT = traj->vals[1][0]-traj->vals[0][0];
	    traj->num = num;

	    // Now let's set the feedforward terms in the vals array:
	    for (i=0; i<num-2; i++)
	    {
		xd = (traj->vals[i+1][1]-traj->vals[i][1])/
		    (traj->vals[i+1][0]-traj->vals[i][0]);
		yd = (traj->vals[i+1][2]-traj->vals[i][2])/
		    (traj->vals[i+1][0]-traj->vals[i][0]);
		xdp = (traj->vals[i+2][1]-traj->vals[i+1][1])/
		    (traj->vals[i+2][0]-traj->vals[i+1][0]);
		ydp = (traj->vals[i+2][2]-traj->vals[i+1][2])/
		    (traj->vals[i+2][0]-traj->vals[i+1][0]);
		xdd = (xdp-xd)/(traj->vals[i+1][0]-traj->vals[i][0]);
		ydd = (ydp-yd)/(traj->vals[i+1][0]-traj->vals[i][0]);
		// Now we can calculate the angular and translational
		// velocities of the robot:
		traj->vals[i][3] = sqrt(pow(xd,2)+pow(yd,2));
		traj->vals[i][4] = (ydd*xd-xdd*yd)/(pow(xd,2)+pow(yd,2));

	    }
	    // Now, let's fill out the last few entries:
	    traj->vals[num-2][3] = traj->vals[num-3][3];
	    traj->vals[num-2][4] = traj->vals[num-3][4];
	    traj->vals[num-1][3] = traj->vals[num-3][3];
	    traj->vals[num-1][4] = traj->vals[num-3][4];

	    return traj;
	}
};

// command_line parsing:
void command_line_parser(int argc, char** argv)
{
    std::string working_dir, file;
   
    // First set the global working directory to the location of the
    // binary:
    working_dir = argv[0];

    int fflag = 0, pflag = 0, rflag = 0;
    int robot_index = 0;
    int index;
    int c;
     
    opterr = 0;
     
    while ((c = getopt (argc, argv, "f:p:r:")) != -1)
    {
	switch (c)
	{
	case 'f':
	    fflag = 1;
	    file = optarg;
	    break;
	case 'p':
	    pflag = 1;
	    working_dir = optarg;
	    break;
	case 'r':
	    rflag = 1;
	    robot_index = atoi(optarg);
	    break;
	case ':':
	    fprintf(stderr,
		    "No argument given for command line option %c \n\r", c);
	    break;
	default:
	    fprintf(stderr, "Usage: %s [-f filename] [-p path-to-file]\n",
		    argv[0]);
	    exit(EXIT_FAILURE);
	}
    }
     
    for (index = optind; index < argc; index++)
	printf ("Non-option argument %s\n", argv[index]);
    if (pflag != 1)
    {
	// Then we just use the default path:
	std::size_t found = working_dir.find("bin");
	std::string tmp_dir = working_dir.substr(0, found);
	working_dir = tmp_dir+"data/";
    }
 
    if (fflag == 0)
    {
	// No file was given:
	file = "default.txt";
    }

    if (rflag != 1)
	robot_index = 1;

    ROS_INFO("Setting robot_index to %d",robot_index);
    ros::param::set("robot_index", robot_index);
  
    // Get filenames:
    filename = working_dir + file;
    ROS_INFO("Filename: %s",filename.c_str());
    return;
}


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // startup node
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle n;

    command_line_parser(argc, argv);
  
    KinematicControl controller1;

    // infinite loop
    ros::spin();

    return 0;
}
