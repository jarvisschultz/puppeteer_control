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
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

#include "puppeteer_msgs/speed_command.h"
#include "puppeteer_msgs/RobotPose.h"
#include "puppeteer_msgs/RobotCommands.h"


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
#define MIN_COMM_FREQUENCY (10.0)
std::string filename;
char ch;

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}       

template<typename T>
    T fromString(const std::string& s)
{
     std::istringstream stream (s);
     T t;
     stream >> t;
     return t;
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
	float vals[][7]; // unknown length;
			 // t,x,y,Vd,Wd,rdotl,rdotr
    } Trajectory;       

    int operating_condition;
    Trajectory *traj;
    ros::NodeHandle n_;
    ros::Publisher serial_pub;
    ros::Subscriber sub;
    ros::Timer timer,pathtimer;
    // ros::Time service_time;
    ros::Publisher ref_pub, rpath_pub;
    puppeteer_msgs::RobotCommands command;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry ref_pose;
    nav_msgs::Path path_m, path_r;
    bool start_flag, cal_start_flag, winch;
    float desired_x, desired_y, desired_th, actual_x, actual_y, actual_th;
    float vd, wd, rdotleft_d, rdotright_d;
    unsigned int num;
    // Controller gains
    float k1, k2, k3;
    float zeta, b;
    // Key times
    ros::Time base_time;
    ros::Time current_time;
    ros::Time tstamp;


public:
    KinematicControl() {
	ROS_DEBUG("Instantiating KinematicControl Class");
	// Initialize necessary variables:
	if(ros::param::has("/operating_condition"))
	    ros::param::set("/operating_condition", 0);
	else
	{
	    ROS_WARN("Cannot Find Parameter: /operating_condition");
	    ros::param::set("/operating_condition", 0);
	}

	// check if we are running the winches... if not, let's set
	// the parameter to say so
	if(!ros::param::has("winch_bool"))
	{
	    ROS_WARN("No winch parameter... setting to false");
	    ros::param::set("winch_bool",false);
	}
	
	// Define publisher for the serial commands:
	serial_pub = n_.advertise<puppeteer_msgs::RobotCommands>
	    ("serial_commands", 1);
	
	// Define subscriber:
	sub = n_.subscribe("pose_ekf", 10, &KinematicControl::subscriber_cb
			   , this);
	// Define a timer and callback for checking system state:
	timer = n_.createTimer(ros::Duration(0.033),
			       &KinematicControl::timercb, this);
	// Define a publisher for publishing the robot's reference pose
	ref_pub = n_.advertise<nav_msgs::Odometry> ("reference_pose", 100);

	// define a timer and publisher for publishing the path of the robot
	pathtimer = n_.createTimer(ros::Duration(5),
				   &KinematicControl::pathpubcb, this);
	rpath_pub = n_.advertise<nav_msgs::Path> ("desired_path_robot", 1);
	
	// Read in the trajectory:
	traj = ReadControls(filename);
	// publish the robot results:
	set_robot_path();
		
	// Send a start flag:
	ros::Duration(0.5).sleep();
	send_start_flag();
	
	// set control gain values:
	zeta = 0.7;
	b = 10;

	// set flags:
	cal_start_flag = true;
	start_flag = true;

    } // END OF KinematicControl()



    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("Control timer callback triggered");
	    ros::param::getCached("/operating_condition", operating_condition);

	    if( operating_condition == 0 ||
		operating_condition == 3 ||
		operating_condition == 4 )
	    {
		start_flag = true;
		cal_start_flag = true;
	    }
	    if (operating_condition == 3 || operating_condition == 4)
	    {
		// then let's keep sending the stop command just so we
		// make sure the robots stop:
		command.robot_index = traj->RobotMY;
		command.header.stamp = tstamp;
		command.type = 'h';
		command.v_left = 0.0;
		command.v_right = 0.0;
		command.v_top = 0.0;
		command.div = 3;
		service_request();
	    }		
		
	    return;
	} // END OF timercb()



    // This gets called every time the estimator publishes a new robot
    // pose
    void subscriber_cb(const nav_msgs::Odometry &pose)
	{
	    ROS_DEBUG("Control pose subscriber triggered");
	    static double running_time = 0.0;
	    ros::param::getCached("/operating_condition", operating_condition);
	    tstamp = pose.header.stamp;
	    
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
		    ROS_DEBUG("Sending initial pose.");

		    // set parameters for sending initial pose
		    command.robot_index = traj->RobotMY;
		    command.header.stamp = tstamp;
		    command.type = 'l';
		    command.x = traj->vals[0][1];
		    command.y = traj->vals[0][2];
		    command.th = atan2(
			traj->vals[1][2]-traj->vals[0][2],
			traj->vals[1][1]-traj->vals[0][1]);
		    command.div = 4;

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
		    command.robot_index = traj->RobotMY;
		    command.header.stamp = tstamp;
		    command.type = 'l';
		    command.x = traj->vals[0][1];
		    command.y = traj->vals[0][2];
		    command.th = atan2(
			traj->vals[1][2]-traj->vals[0][2],
			traj->vals[1][1]-traj->vals[0][1]);
		    command.div = 4;

		    start_flag = false;
		    base_time = command.header.stamp;
		    ROS_DEBUG("Setting Base Time to %f",base_time.toSec());

		    // check if we are running the winch:
		    check_winch();
		}
		else
		{
		    // we will run the regular control loop
		    // let's first get the expected pose at the given time:
		    current_time = tstamp;
		    running_time = (current_time-base_time).toSec();
		    ROS_DEBUG("Running time is %f", running_time);
		    ROS_DEBUG("Final time is %f", traj->vals[num-1][0]);
		    // check that running_time is less than the final time:
		    if (running_time <= traj->vals[num-1][0])
		    {
			get_desired_pose(running_time, pose);
			get_control_values(pose);
		    }
		    else
		    {
			// stop robot!
			ROS_INFO("Trajectory Finished!");
			command.robot_index = traj->RobotMY;
			command.header.stamp = tstamp;
			command.type = 'h';
			command.v_left = 0.0;
			command.v_right = 0.0;
			command.v_top = 0.0;
			command.div = 3;
			// set operating_condition to stop
			ros::param::set("/operating_condition", 3);
			start_flag = true;
			cal_start_flag = true;
		    }
		}
	    }
	    else if (operating_condition == 4)
	    {
		ROS_WARN_THROTTLE(1, "Emergency Stop Detected!");
		command.robot_index = traj->RobotMY;
		command.header.stamp = ros::Time::now();
		command.type = 'h';
		command.v_left = 0.0;
		command.v_right = 0.0;
		command.v_top = 0.0;
		command.div = 3;
		start_flag = true;
		cal_start_flag = true;
	    }

	    service_request();
	    return;
	} // END OF subscribercb()


    
    void service_request(void)
	{
	    // set local time for current service call:
	    // service_time = ros::Time::now();
	    serial_pub.publish(command);
	    return;
	} // END OF service_request()


    
    void get_desired_pose(float time, const nav_msgs::Odometry &p)
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
	    if (isnan(desired_th) == 0)
		desired_th = angles::normalize_angle(desired_th);

	    // Now, we can interpolate the feedforward terms:
	    vd = (traj->vals[index-1][3])+
		mult*(traj->vals[index][3]-traj->vals[index-1][3]);
	    wd = (traj->vals[index-1][4])+
		mult*(traj->vals[index][4]-traj->vals[index-1][4]);
	    rdotleft_d = (traj->vals[index-1][5])+
	    	mult*(traj->vals[index][5]-traj->vals[index-1][5]);
	    rdotright_d = (traj->vals[index-1][6])+
	    	mult*(traj->vals[index][6]-traj->vals[index-1][6]);

	    ROS_DEBUG("Desired values at time t = %f", time);
	    ROS_DEBUG("Xd = %f\tYd = %f\tTd = %f\t",
		      desired_x, desired_y, desired_th);
	    ROS_DEBUG("vd = %f\twd = %f\trdotl = %f\trdotr = %f\t",
		      vd, wd, rdotleft_d, rdotright_d);

	    // now we can convert the desired pose into a
	    // nav_msgs::Odometry and publish it
	    ref_pose.header.stamp = p.header.stamp;
	    ref_pose.header.frame_id = "/robot_odom_pov";
	    std::stringstream ss;
	    ss << "robot_" << ch << "_base_footprint_ref";
	    ref_pose.child_frame_id = ss.str();
	    ref_pose.pose.pose.position.x = desired_x;
	    ref_pose.pose.pose.position.y = desired_y;
	    ref_pose.pose.pose.position.z = 0;
	    geometry_msgs::Quaternion quat =
		tf::createQuaternionMsgFromYaw(desired_th);
	    ref_pose.pose.pose.orientation = quat;
	    ref_pub.publish(ref_pose);

	    // now, let's publish the transform that goes along with it
	    geometry_msgs::TransformStamped ref_trans;
	    ref_trans.header.stamp = ref_pose.header.stamp;
	    ref_trans.header.frame_id = ref_pose.header.frame_id;
	    ref_trans.child_frame_id = ref_pose.child_frame_id;
	    ref_trans.transform.translation.x = ref_pose.pose.pose.position.x;
	    ref_trans.transform.translation.y = ref_pose.pose.pose.position.y;
	    ref_trans.transform.translation.z = ref_pose.pose.pose.position.z;
	    ref_trans.transform.rotation = quat;

	    br.sendTransform(ref_trans);	    
	    	    
	    return;
	}  // END OF get_desired_pose()



    // void get_control_values(const puppeteer_msgs::RobotPose &pose)
    void get_control_values(const nav_msgs::Odometry &p)
	{
	    ROS_DEBUG("Calculating the control values");
	    float v, omega;// , dtheta;

	    // Fill out the robot's pose by transforming the published
	    // odometry message into the robot's own reference frame
	    actual_x = p.pose.pose.position.x;
	    actual_y = -p.pose.pose.position.y;

	    actual_th = tf::getYaw(p.pose.pose.orientation);
	    actual_th = angles::normalize_angle(-actual_th);
	    
	    
	    ROS_DEBUG("Xa = %f\tYa = %f\tTa = %f\t",
		      actual_x, actual_y, actual_th);

	    // Now calculate the gain values:
	    k1 = 2*zeta*sqrt(pow(wd,2)+b*pow(vd,2));
	    k2 = b*fabs(vd);
	    k3 = k1;

	    ROS_DEBUG("Gains: %f  %f  %f",k1,k2,k3);
	    
	    // calc control values:
	    v = vd*cos(desired_th-actual_th) +
		k1*(cos(actual_th)*(desired_x-actual_x)+
		    sin(actual_th)*(desired_y-actual_y));
	    omega = wd + k2*((float) sgn(vd))*
	    	(cos(actual_th)*(desired_y-actual_y)-
	    	 sin(actual_th)*(desired_x-actual_x))
		+ k3*angles::shortest_angular_distance(actual_th, desired_th);

	    ROS_DEBUG("Intermediate value of control values: v = %f\tw = %f"
		      ,v,omega);

	    // check to make sure no errors occurred
	    if (isnan(v) != 0)
		v = 0.0;
	    if (isnan(omega) != 0)
		omega = 0.0;

	    // prevent out-of-control speeds
	    while (v > MAX_TRANS_VEL || omega > MAX_ANG_VEL)
	    {
		v *= 0.9;
	        omega *= 0.9; 
	    }

	    ROS_DEBUG("Sending control values: v = %f\tw = %f",v,omega);

	    // Set service parameters:
	    command.robot_index = traj->RobotMY;
	    command.header.stamp = current_time;
	    command.v_robot = v;
	    command.w_robot = omega;

	    if (winch)
	    {
		command.type = 'i';
		command.rdot_left = rdotleft_d;
		command.rdot_right = rdotright_d;
	    }
	    else
	    {
		command.type = 'd';
		command.rdot = 0.0;  //  Disable winches
	    }
	    command.div = 4;

	    return;
	} // END OF get_control_values()



    void send_start_flag(void)
	{
	    ROS_DEBUG("Sending start flag");
	    // First set the parameters for the service call
	    command.robot_index = traj->RobotMY;
	    command.header.stamp = ros::Time::now();
	    command.type = 'm';
	    command.div = 0;
	    serial_pub.publish(command);
	    return;
	} // END OF send_start_flag()


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
		    // fill out t,x,y
		    getline(file, line, ',');
		    std::stringstream ss(line);
		    ss >> temp_float;
		    traj->vals[i][j] = temp_float;
		}
		// fill out r dots
		getline(file, line, ',');
		std::stringstream ss(line);
		ss >> temp_float;
		traj->vals[i][5] = temp_float;

		getline(file, line);
		std::stringstream ss2(line);
		ss2 >> temp_float;
		traj->vals[i][6] = temp_float;
	    }
	    file.close();
	    
	    // Now we can set DT and the robot_index
	    ros::param::get("robot_index", traj->RobotMY);
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
	    ros::param::set("robot_x0", traj->vals[0][1]);
	    ros::param::set("robot_z0", traj->vals[0][2]);
	    ros::param::set("robot_y0", 1.0); // this value is arbitrary!
	    ros::param::set("robot_r0", traj->vals[0][5]);

	    double th = atan2(traj->vals[1][2]-traj->vals[0][2],
			      traj->vals[1][1]-traj->vals[0][1]);
	    
	    if (isnan(th) == 0)
	    {
		th = angles::normalize_angle(th);
		ros::param::set("robot_th0", th);
	    }
	    else
		ROS_ERROR("Initial angle returned NaN!");

	    return traj;
	} // END OF ReadControls()

    
    double clamp_angle(const double theta)
	{
	    double th = theta;
	    while(th > M_PI)
		th -= 2.0*M_PI;
	    while(th <= -M_PI)
		th += 2.0*M_PI;
	    return th;
	}
    
    double angle_correction(double desired, double actual)
	{
	    double tmp = actual;
	    while ((desired-tmp) > M_PI) tmp += 2.0*M_PI;
	    while ((desired-tmp) < -M_PI) tmp -= 2.0*M_PI;
	    return(desired-tmp);
	}

    void check_winch(void)
	{
	    // check if we are running the winches... if not, let's set
	    // the parameter to say so
	    if(!ros::param::has("winch_bool"))
		ros::param::set("winch_bool",false);
	    ROS_DEBUG("Checking winch bool");
	    ros::param::getCached("winch_bool", winch);
	    ROS_DEBUG("winch_bool = %d", winch);	    
	    return;
	}

    void set_robot_path(void)
	{
	    path_r.poses.resize(traj->num);
	    path_r.header.frame_id = "robot_odom_pov";
	    for (unsigned int i=0; i<(traj->num); i++)
	    {
		path_r.poses[i].header.frame_id = "robot_odom_pov";
		path_r.poses[i].pose.position.x = traj->vals[i][1];
		path_r.poses[i].pose.position.y = traj->vals[i][2];
		path_r.poses[i].pose.position.z = 0;
	    }
	    return;
	}

    void pathpubcb(const ros::TimerEvent& e)
	{
	    // publish paths
	    rpath_pub.publish(path_r);
	    return;
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

    // did we spec rflag on command line?
    if (rflag != 1)
    {
	// let's see if there is a parameter already
	if(!ros::param::has("robot_index"))
	{
	    // use a default:
	    robot_index = 1;
	    ROS_INFO("Setting robot_index to default (%d)",robot_index);
	    ros::param::set("robot_index", robot_index);
	}
	else
	    ros::param::get("robot_index", robot_index);
    }
    else
    {
	// since we did spec it, let's set it
	ros::param::set("robot_index", robot_index);
    }
  
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
    ROSCONSOLE_AUTOINIT;
    
    // startup node
    ros::init(argc, argv, "kinematic_controller");
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);
    ros::NodeHandle n;

    command_line_parser(argc, argv);

    std::string name = ros::this_node::getNamespace();
    if (!name.empty())
	ch = *name.rbegin();
    else
	ch = '0';

    KinematicControl controller1;

    // infinite loop
    ros::spin();

    return 0;
}
