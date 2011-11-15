// kalman_kinematic_control.cpp
// Jarvis Schultz
// July 11, 2011


//----------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This version of the kinematic controller is designed to subscribe
// to data published by the robot_pose_ekf ROS package, and run the
// kinematic controller based on the results of that filter.  
//  
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

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
#define MAX_TRANS_VEL  (2.25)
#define MAX_ANG_VEL (30.0)
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
	float vals[][5]; // unknown length;
			 // t,x,y,Vd,Wd
    } Trajectory;       

    int operating_condition;
    Trajectory *traj;
    ros::NodeHandle n_;
    ros::ServiceClient client;
    ros::Subscriber sub;
    ros::Timer timer;
    puppeteer_msgs::speed_command srv;
    puppeteer_msgs::RobotPose pose;
    bool start_flag, cal_start_flag;
    float desired_x, desired_y, desired_th, actual_x, actual_y, actual_th;
    float vd, wd, rdotd;
    unsigned int num;
    // Controller gains
    float k1, k2, k3;
    float zeta, b;

public:
    KinematicControl() {
	ROS_DEBUG("Instantiating KinematicControl Class");
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
	sub = n_.subscribe("/pose_ekf", 1, &KinematicControl::subscriber_cb, this);
	// Define a timer and callback for checking system state:
	timer = n_.createTimer(ros::Duration(0.1), &KinematicControl::timercb, this);

	// Send a start flag:
	send_start_flag();
	
	// set control gain values:
	zeta = 0.7;
	b = 10;

	// set flags:
	cal_start_flag = true;
	start_flag = true;
    }

    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("Timer callback triggered");
	    int operating_condition = 0;
	    ros::param::get("/operating_condition", operating_condition);

	    if(operating_condition == 3 || operating_condition == 4)
	    {
		start_flag = true;
		cal_start_flag = true;
	    }
	    return;
	}

    // This gets called every time the estimator publishes a new robot
    // pose
    // void subscriber_cb(const puppeteer_msgs::RobotPose &pose)
    void subscriber_cb(const nav_msgs::Odometry &pose)
	{
	    ROS_DEBUG("Subscriber callback triggered");
	    static double running_time = 0.0;
	    static ros::Time base_time;
	    ros::param::get("/operating_condition", operating_condition);
	    
	    if (operating_condition == 0 || operating_condition == 3)
	    {
		start_flag = true;
		cal_start_flag = true;
		return;
	    }
	    else if (operating_condition == 1)
	    {
		if (cal_start_flag == true)
		{
		    ROS_INFO("Sending initial pose.");

		    // set parameters for sending initial pose
		    srv.request.robot_index = traj->RobotMY;
		    srv.request.type = 'l';
		    srv.request.Vleft = traj->vals[0][1];
		    srv.request.Vright = traj->vals[0][2];
		    srv.request.Vtop = atan2(traj->vals[1][2]-traj->vals[0][2],
					     traj->vals[1][1]-traj->vals[0][1]);
		    srv.request.div = 4;

		    cal_start_flag = false;
		}
		ROS_INFO_THROTTLE(5, "Calibrating...");
	    }
	    else if (operating_condition == 2)
	    {
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
		    ROS_DEBUG("Setting Base Time to %f",base_time.toSec());
		    
		}
		else
		{
		    // we will run the regular control loop
		    // let's first get the expected pose at the given time:
		    running_time = ((ros::Time::now()).toSec()-
				    base_time.toSec());
		    ROS_DEBUG("Running time is %f", running_time);
		    ROS_DEBUG("Final time is %f", traj->vals[num-1][0]);
		    // check that running_time is less than the final time:
		    if (running_time <= traj->vals[num-1][0])
		    {
			get_desired_pose(running_time);
			get_control_values(pose);
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
			cal_start_flag = true;
		    }
		}
	    }
	    else if (operating_condition == 4)
	    {
		ROS_WARN("Emergency Stop Detected!");
		srv.request.robot_index = traj->RobotMY;
		srv.request.type = 'h';
		srv.request.Vleft = 0.0;
		srv.request.Vright = 0.0;
		srv.request.Vtop = 0.0;
		srv.request.div = 3;
		start_flag = true;
		cal_start_flag = true;
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

    void get_desired_pose(float time)
	{
	    ROS_DEBUG("Interpolating desired pose");
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
	    desired_th = clamp_angle(desired_th);
	    // while (desired_th <= -M_PI)
	    // 	desired_th += 2.0*M_PI;
	    // while (desired_th > M_PI)
	    // 	desired_th -= 2.0*M_PI;

	    // Now, we can interpolate the feedforward terms:
	    vd = (traj->vals[index-1][3])+
		mult*(traj->vals[index][3]-traj->vals[index-1][3]);
	    wd = (traj->vals[index-1][4])+
		mult*(traj->vals[index][4]-traj->vals[index-1][4]);
	    // rdotd = (traj->vals[index-1][5])+
	    // 	mult*(traj->vals[index][5]-traj->vals[index-1][5]);

	    rdotd = 0.0;
	    ROS_DEBUG("Desired values at time t = %f", time);
	    ROS_DEBUG("Xd = %f\tYd = %f\tTd = %f\t",desired_x, desired_y, desired_th);
	    ROS_DEBUG("vd = %f\twd = %f\trdotd = %f\t",vd, wd, rdotd);
	    return;
	}

    // void get_control_values(const puppeteer_msgs::RobotPose &pose)
    void get_control_values(const nav_msgs::Odometry &p)
	{
	    ROS_DEBUG("Calculating the control values");
	    float v, omega, dtheta;
	    float comps[3];

	    // Fill out the robot's pose by transforming the published
	    // odometry message into the robot's own reference frame
	    actual_x = p.pose.pose.position.x;
	    actual_y = -p.pose.pose.position.y;

	    actual_th = tf::getYaw(p.pose.pose.orientation);
	    actual_th = clamp_angle(-actual_th);
	    

	    
	    // actual_x = pose.x_robot;
	    // actual_y = pose.y_robot;
	    // actual_th = pose.theta;
	    ROS_DEBUG("Xa = %f\tYa = %f\tTa = %f\t",actual_x, actual_y, actual_th);

	    // Now calculate the gain values:
	    k1 = 2*zeta*sqrt(pow(wd,2)+b*pow(vd,2));
	    k2 = b*fabs(vd);
	    k3 = k1;

	    // calc control values:
	    v = vd*cos(desired_th-actual_th) +
		k1*(cos(actual_th)*(desired_x-actual_x)+
		    sin(actual_th)*(desired_y-actual_y));
	    comps[0] = desired_th-actual_th-2.0*M_PI;
	    comps[1] = desired_th-actual_th+2.0*M_PI;
	    comps[2] = desired_th-actual_th;
	    if (abs(comps[0]) < abs(comps[1]))
	    {
		if (abs(comps[0]) < abs(comps[2]))
		    dtheta = comps[0];
		else
		    dtheta = comps[2];
	    }
	    else
	    {
		if (abs(comps[1]) < abs(comps[2]))
		    dtheta = comps[1];
		else
		    dtheta = comps[2];
	    }
	    omega = wd + k2*((float) sgn(vd))*
		(cos(actual_th)*(desired_y-actual_y)-
		 sin(actual_th)*(desired_x-actual_x))
		+ k3*dtheta;

	    while (v > MAX_TRANS_VEL || omega > MAX_ANG_VEL)
	    {
		v *= 0.9;
	        omega *= 0.9; 
	    }
	    ROS_DEBUG("Commands: v = %f\tomega = %f",v,omega);

	    // Set service parameters:
	    srv.request.robot_index = traj->RobotMY;
	    srv.request.type = 'd';
	    srv.request.Vleft = v;
	    srv.request.Vright = omega;
	    srv.request.Vtop = 0.0;  //  Disable winches
	    // srv.request.Vtop = rdotd;
	    srv.request.div = 4;

	    return;
	}

    void send_start_flag(void)
	{
	    ROS_DEBUG("Sending start flag");
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

	    // let's set some parameters for the initial pose of the robot:
	    ros::param::set("/robot_x0", traj->vals[0][1]);
	    ros::param::set("/robot_z0", traj->vals[0][2]);
	    ros::param::set("/robot_y0", 2.0);

	    double th = atan2(traj->vals[1][1]-traj->vals[0][1],
			      traj->vals[1][2]-traj->vals[0][2]);

	    th = clamp_angle(th);
	    // while (th <= -M_PI)
	    // 	th += 2.0*M_PI;
	    // while (th > M_PI)
	    // 	th -= 2.0*M_PI;
	    ros::param::set("/robot_th0", th);

	    return traj;
	}

    double clamp_angle(const double theta)
	{
	    double th = theta;
	    while(th > M_PI)
		th -= 2.0*M_PI;
	    while(th <= -M_PI)
		th += 2.0*M_PI;
	    return th;
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
