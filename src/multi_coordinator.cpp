// Jarvis Schultz

// April 2011

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This is the coordinator node for controlling mulitple robots.


// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/Robots.h>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_ROBOTS (9)
#define NUM_CALIBRATES (30)
#define ROBOT_CIRCUMFERENCE (57.5) // centimeters
#define DEFAULT_RADIUS (ROBOT_CIRCUMFERENCE/M_PI/2.0/100.) // meters
#define NUM_EKF_INITS (3)

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class Coordinator
{

private:
    ros::NodeHandle n_;
    ros::Subscriber robots_sub;
    ros::Publisher robots_pub[MAX_ROBOTS];
    ros::Timer timer;
    int nr;
    bool calibrated_flag;
    unsigned int calibrate_count;
    ros::Time tstamp;
    std::vector<int> order;
    int operating_condition;
    double robot_radius;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry kin_pose[MAX_ROBOTS];
    puppeteer_msgs::Robots current_bots, prev_bots;


public:
    Coordinator() {
	ROS_DEBUG("Creating publishers and subscribers");
	timer = n_.
	    createTimer(ros::Duration(0.033), &Coordinator::timercb, this);
	robots_sub = n_.subscribe("/robot_positions", 1,
				  &Coordinator::datacb, this);
	// get the number of robots
	if (ros::param::has("/number_robots"))
	    ros::param::get("/number_robots",nr);
	else
	{
	    ros::param::set("/number_robots", 1);
	    nr = 1;
	}
	for (int j=0; j<nr; j++)
	{
	    // create publishers
	    std::stringstream ss;
	    ss << "/robot_" << j+1 << "/vo";
	    robots_pub[j] = n_.advertise<nav_msgs::Odometry>(ss.str(), 1);
	}

	// get operating condition
	if (ros::param::has("/operating_condition"))
	    ros::param::get("/operating_condition", operating_condition);
	else
	    ros::param::set("/operating_condition", 0);

	// get the size of the robot:
	if(ros::param::has("/robot_radius"))
	    ros::param::get("/robot_radius", robot_radius);
	else
	{
	    robot_radius = DEFAULT_RADIUS;
	    ros::param::set("/robot_radius", robot_radius);
	}

	// setup default values:
	calibrated_flag = false;
	tstamp = ros::Time::now();
	    
	// set covariance for the pose messages
	double kin_cov_dist = 0.01;	// in meters^2
	double kin_cov_ori = pow(M_PI,2.0);	// radians^2
	boost::array<double,36ul> kincov = {{kin_cov_dist, 0, 0, 0, 0, 0,
					     0, kin_cov_dist, 0, 0, 0, 0,
					     0, 0,        99999, 0, 0, 0,
					     0, 0, 0,        99999, 0, 0,
					     0, 0, 0, 0,        99999, 0,
					     0, 0, 0, 0, 0,  kin_cov_ori}};
	for (int i=0; i<nr; i++)
	    kin_pose[i].pose.covariance = kincov;


	ROS_INFO("Starting Coordinator Node");

    }


    void datacb(const puppeteer_msgs::Robots &bots)
	{
	    ROS_DEBUG("datacb triggered");
	    static bool first_flag = true;
	    puppeteer_msgs::Robots b;
	    b = bots;

	    // correct the points in bots
	    b = adjust_for_robot_size(b);
	    // store the values that we received:
	    if (first_flag) {
		current_bots = b;
		prev_bots = b;
		first_flag = false;
		return;
	    }
	    prev_bots = current_bots;
	    current_bots = b;
	    // do we need to calibrate?
	    if ( !calibrated_flag ) {
		calibrate_routine();
		return;
	    }

	    // send all relevant transforms
	    send_frames();

	    // transform the Robots to /optimization_frame
	    transform_robots();
	    
	    return;
	}
	
    

    void timercb(const ros::TimerEvent& e)
	{
	    static bool gen_flag = true;
	    static unsigned int init_ekf_count = 0;
	    
	    if (gen_flag)
	    {
		// Generate the robot ordering vector
		gen_flag = generate_order();
		return;
	    }

	    // get operating condition
	    ros::param::get("/operating_condition", operating_condition);
	    	    
	    // check to see if we are in run state
	    if(operating_condition == 1 || operating_condition == 2)
	    {
		if(calibrated_flag == true)
		{
		    if(init_ekf_count <= NUM_EKF_INITS)
			get_kinect_estimate(1);
		    else
			get_kinect_estimate(operating_condition);
		    init_ekf_count++;
		}
		return;
	    }
	    
	    // are we in idle or stop condition?
	    else if(operating_condition == 0 || operating_condition == 3)
		ROS_DEBUG("Coordinator node is idle due to operating condition");

	    // are we in emergency stop condition?
	    else if(operating_condition == 4)
		ROS_WARN_ONCE("Emergency Stop Requested");

	    // otherwise something terrible has happened
	    else
		ROS_ERROR("Invalid value for operating_condition");

	    calibrated_flag = false;
	    calibrate_count = 0;
	    init_ekf_count = 0;
	    return;
	}

    
    bool generate_order(void)
	{
	    // this function looks at the starting positions of each
	    // of the robots, it then determines what order they will
	    // be seen by the Kinect.  They are ordered from the
	    // highest x-value (in /oriented_optimization_frame) to
	    // the lowest
	    std::vector<double> pos;
	    double tmp;
	    for (int j=0; j<nr; j++)
	    {
		std::stringstream ss;
		ss << "/robot_" << j+1 << "/robot_x0";
		if (ros::param::has(ss.str()))
		    ros::param::get(ss.str(), tmp);
		else {
		    ROS_WARN_THROTTLE(1, "Cannot determine ordering!");
		    return true;
		}

		pos.push_back(tmp);		    
		order.push_back(j+1);
	    }

	    // now we can sort the stuff
	    int keyi=0;
	    double key=0;
	    int j=0;
	    for (unsigned int i=1; i<pos.size(); ++i) 
	    {
		key= pos[i];
		keyi = order[i];
		j = i-1;
		while((j >= 0) && (pos[j] < key))
		{
		    pos[j+1] = pos[j];
		    order[j+1] = order[j];
		    j -= 1;
		}
		pos[j+1]=key;
		order[j+1]=keyi;
	    }

	    return false;
	}


    
        void get_kinect_estimate(int op)
	{
	    // // Let's first get the transform from /optimization_frame
	    // // to /map
	    // static tf::StampedTransform transform;
	    // geometry_msgs::PointStamped tmp;
		    
	    // try{
	    // 	tf.lookupTransform(
	    // 	    "map", "optimization_frame",
	    // 	    tstamp, transform);
	    // 	tf.transformPoint("map", transformed_robot, tmp);

	    // }
	    // catch(tf::TransformException& ex){
	    // 	ROS_ERROR(
	    // 	    "Error trying to lookupTransform from /map "
	    // 	    "to /optimization_frame: %s", ex.what());
	    // 	return;
	    // }
	    
	    // // Now we can publish the Kinect's estimate of the robot's
	    // // pose
	    // kin_pose.header.stamp = tstamp;
	    // kin_pose.header.frame_id = "map";
	    // kin_pose.child_frame_id = "base_footprint_kinect";
	    // tmp.point.z = 0.0;
	    // kin_pose.pose.pose.position = tmp.point;
	    // double theta = 0.0;
	    // if (op == 2)
	    // {
	    // 	theta = atan2(transformed_robot.point.x-
	    // 			     transformed_robot_last.point.x,
	    // 			     transformed_robot.point.z-
	    // 			     transformed_robot_last.point.z);
	    // 	theta = clamp_angle(theta-M_PI/2.0);
	    // }
	    // else
	    // {
	    // 	theta = robot_start_ori;
	    // 	theta = clamp_angle(-theta); 
	    // }
	    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	    // kin_pose.pose.pose.orientation = quat;				 
	    
	    // // Now let's publish the estimated pose as a
	    // // nav_msgs/Odometry message on a topic called /vo
	    // vo_pub.publish(kin_pose);

	    // // now, let's publish the transform that goes along with it
	    // geometry_msgs::TransformStamped kin_trans;
	    // tf::Quaternion q1, q2;
	    // q1 = tf::createQuaternionFromYaw(theta);
	    // q2 = tf::Quaternion(1.0,0,0,0);
	    // q1 = q1*q2;
	    // tf::quaternionTFToMsg(q1, quat);

	    // kin_trans.header.stamp = tstamp;
	    // kin_trans.header.frame_id = kin_pose.header.frame_id;
	    // kin_trans.child_frame_id = kin_pose.child_frame_id;
	    // kin_trans.transform.translation.x = kin_pose.pose.pose.position.x;
	    // kin_trans.transform.translation.y = kin_pose.pose.pose.position.y;
	    // kin_trans.transform.translation.z = kin_pose.pose.pose.position.z;
	    // kin_trans.transform.rotation = quat;

	    // ROS_DEBUG("Sending transform for output of estimator node");
	    // br.sendTransform(kin_trans);
	    
	    return;
	}

    double clamp_angle(double theta)
	{
	    double th = theta;
	    while(th > M_PI)
		th -= 2.0*M_PI;
	    while(th <= M_PI)
		th += 2.0*M_PI;
	    return th;
	}

    // this function accounts for the size of the robot:
    puppeteer_msgs::Robots adjust_for_robot_size(puppeteer_msgs::Robots &p)
	{
	    ROS_DEBUG("correct_vals called");
	    puppeteer_msgs::Robots point;
	    Eigen::Vector3d ur;
	    point = p;

	    for (unsigned int j=0; j<p.number; j++)
	    {
		// let's create a unit vector from the kinect frame to the
		// robot's location
		ur << p.robots[j].point.x, p.robots[j].point.y, p.robots[j].point.z;
		// now turn it into a unit vector:
		ur = ur/ur.norm();
		// now we can correct the values of point
		ur = ur*robot_radius;
	    
		point.robots[j].point.x = point.robots[j].point.x+ur(0);
		point.robots[j].point.y = point.robots[j].point.y+ur(1);
		point.robots[j].point.z = point.robots[j].point.z+ur(2);
	    }
	    
	    return(point);	    	    
	}


    
}; // end Coordinator Class



//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_coordinator");

    // // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting Coordinator Node...\n");
    Coordinator coord;
  
    ros::spin();
  
    return 0;
}
