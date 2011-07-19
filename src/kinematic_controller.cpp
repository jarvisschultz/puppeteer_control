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
#define WIDTH	(0.132334/2) 
std::string filename;

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
	float vals[][3];
    } Trajectory;       

    int operating_condition;
    Trajectory *traj;
    ros::NodeHandle n_;
    ros::ServiceClient client;
    ros::Subscriber sub;
    puppeteer_msgs::speed_command srv;
    puppeteer_msgs::RobotPose pose;
    bool start_flag;
    float desired_x, desired_y, desired_th, actual_x, actual_y, actual_th;
    float offset_x, offset_y, offset_th;
    unsigned int num;
    // Controller gains
    float krho, kbeta, kalpha;

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
	sub = n_.subscribe("/robot_pose", 1, &KinematicControl::subscriber_cb, this);

	// set control gain values:
	krho = 5;
	kbeta = -1.5;
	kalpha = 20;
	if (krho < 0 || kbeta > 0 || kalpha-krho < 0)
	    ROS_WARN("Controller gains are not stable!!!");
    }

    // This gets called every time the estimator publishes a new robot
    // pose
    void subscriber_cb(const puppeteer_msgs::RobotPose &pose)
	{
	    static double running_time = 0.0;
	    static ros::Time base_time;
	    ros::param::get("/operating_condition", operating_condition);
	    
	    if (operating_condition != 2)
	    {
		start_flag = true;
		return;
	    }
	    else
	    {
		if (start_flag == true)
		{
		    start_flag = false;
		    base_time = ros::Time::now();
		    
		    ROS_INFO("Beginning movement execution");
		    // get offset values:
		    offset_x = traj->vals[0][0];
		    offset_y = traj->vals[0][1];
		    offset_th = traj->vals[0][2];
		    // set parameters for start command
		    srv.request.robot_index = traj->RobotMY;
		    srv.request.type = 'm';
		    srv.request.Vleft = 0.0;
		    srv.request.Vright = 0.0;
		    srv.request.Vtop = 0.0;
		    srv.request.div = 0;
		}
		else
		{
		    // we will run the regular control loop
		    // let's first get the expected pose at the given time:
		    running_time = ((ros::Time::now()).toSec()-
				    base_time.toSec());
		    // check that running_time is less than the final time:
		    if (running_time <= traj->vals[num-1][0])
		    {
			get_desired_pose(running_time);
			get_control_values();
		    }
		    else
		    {
			// stop robot!
			srv.request.robot_index = traj->RobotMY;
			srv.request.type = 'h';
			srv.request.Vleft = 0.0;
			srv.request.Vright = 0.0;
			srv.request.Vtop = 0.0;
			srv.request.div = 3;
			// set operating_condition to stop
			ros::param::set("operating_condition", 3);
		    }
		}
		
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
	    while (desired_th < -M_PI)
		desired_th += 2.0*M_PI;
	    while (desired_th > M_PI)
		desired_th -= 2.0*M_PI;
	    return;
	}

    void get_control_values(void)
	{
	    float alpha, beta, rho, v, omega, vleft, vright;
	    // This function takes no arguments, it just calculates
	    // the wheel velocities we should send as dictated by the
	    // closed loop controller.  It also sets those parameters
	    // in the serial service

	    // now, we can find the error terms:
	    actual_x = pose.x_robot+offset_x;
	    actual_y = pose.y_robot+offset_y;
	    actual_th = pose.theta+offset_th;
	    rho = sqrt(pow((actual_x-desired_x), 2.0)
		       +pow((actual_y-desired_y),2.0));
	    alpha = -actual_th+atan2(desired_y-actual_y,
				     desired_x-actual_x);
	    while (alpha < -M_PI)
		alpha += 2.0*M_PI;
	    while (alpha > M_PI)
		alpha -= 2.0*M_PI;
	    beta = -(actual_th)-alpha;
	    // Now we can convert those to translational and
	    // angular body velocities:
	    v = krho*rho;
	    omega = kalpha*alpha+kbeta*beta;
	    // Now we can convert those to angular wheel velocities:
	    vright = (v+omega*WIDTH)/DWHEEL/2.0;
	    vleft = (v-omega*WIDTH)/DWHEEL/2.0;
	    // Set service parameters:
	    srv.request.robot_index = traj->RobotMY;
	    srv.request.type = 'h';
	    srv.request.Vleft = vleft;
	    srv.request.Vright = vright;
	    srv.request.Vtop = 0.0;
	    srv.request.div = 3;

	    return;
	}

    Trajectory *ReadControls(std::string filename)
	{
	    unsigned int i,j;
	    float temp_float;
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
  ros::init(argc, argv, "kinematic_controller");
  ros::NodeHandle n;

  command_line_parser(argc, argv);
  
  KinematicControl controller1;

  // infinite loop
  ros::spin();

  return 0;
}
