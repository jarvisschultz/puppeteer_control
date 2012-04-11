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
#include <Eigen/Dense>


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
    std::vector<int> ref_ord;
    int operating_condition;
    std::vector<double> robot_radius;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry kin_pose[MAX_ROBOTS];
    puppeteer_msgs::Robots current_bots, prev_bots, start_bots, cal_bots;
    Eigen::Vector3d cal_pos;


public:
    Coordinator() {
	ROS_DEBUG("Creating publishers and subscribers");
	timer = n_.
	    createTimer(ros::Duration(0.033), &Coordinator::timercb, this);
	robots_sub = n_.subscribe("robot_positions", 1,
				  &Coordinator::datacb, this);
	// get the number of robots
	if (ros::param::has("/number_robots"))
	    ros::param::get("/number_robots",nr);
	else
	{
	    ROS_WARN("Number of robots not set...");
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

	// set operating condition to idle
	ros::param::set("/operating_condition", 0);

	// get the size of the robot:
	for (int j=0; j<nr; j++)
	{
	    std::stringstream ss;
	    double tmp = 0;
	    ss << "/robot_" << j+1 << "/robot_radius";
	    if(ros::param::has(ss.str()))
	    {
		ros::param::get(ss.str(), tmp);
		robot_radius.push_back(tmp);
	    }
	    else
		robot_radius.push_back( DEFAULT_RADIUS );	    
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

	return;
    }


    void datacb(const puppeteer_msgs::Robots &bots)
    // void datacb(boost::shared_ptr<puppeteer_msgs::Robots const> bots)
	{
	    ROS_DEBUG("datacb triggered");
	    static bool first_flag = true;
	    puppeteer_msgs::Robots b;
	    b = bots;

	    tstamp = ros::Time::now();
	    
	    // correct the points in bots
	    b = adjust_for_robot_size(b);
	    // store the values that we received:
	    if (first_flag) {
		ROS_DEBUG("First call!");
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

	    // // transform the Robots to /optimization_frame
	    // transform_robots();

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
		ROS_DEBUG_THROTTLE(1,"Coordinator node is idle due to operating condition");

	    // are we in emergency stop condition?
	    else if(operating_condition == 4)
		ROS_WARN_THROTTLE(1,"Emergency Stop Requested");

	    // otherwise something terrible has happened
	    else
		ROS_ERROR("Invalid value for operating_condition");

	    calibrated_flag = false;
	    calibrate_count = 0;
	    init_ekf_count = 0;
	    return;
	}
    

    void calibrate_routine(void)
	{
	    ROS_DEBUG("calibration_routine triggered");
	    static Eigen::MatrixXd cal_eig(nr,3);
			
	    // if this is the first call to the function, let's get
	    // the starting pose for each of the robots.
	    if(calibrate_count == 0)
	    {
		ROS_DEBUG_THROTTLE(1,"Calibrating...");
		puppeteer_msgs::Robots r;
		double tmp;
		r.robots.resize(nr);
		for (int j=0; j<nr; j++)
		{
		    std::stringstream ss;
		    ss << "/robot_" << j+1 << "/robot_x0";
		    ros::param::get(ss.str(), tmp);
		    ss.str(""); ss.clear();
		    r.robots[j].point.x = tmp;
		    ss << "/robot_" << j+1 << "/robot_y0";
		    ros::param::get(ss.str(), tmp);
		    ss.str(""); ss.clear();
		    r.robots[j].point.y = tmp;
		    ss << "/robot_" << j+1 << "/robot_z0";
		    ros::param::get(ss.str(), tmp);
		    ss.str(""); ss.clear();
		    r.robots[j].point.z = tmp;
		}

		start_bots = r;
		
		// increment counter, and initialize transform values
		calibrate_count++;
		cal_pos << 0, 0, 0;
		for(int i=0; i<cal_eig.rows(); i++) {
		    cal_eig(i,0) = 0; cal_eig(i,1) = 0; cal_eig(i,2) = 0; }
		return;
	    }
	    // we are in the process of calibrating:
	    else if (calibrate_count <= NUM_CALIBRATES)
	    {
		ROS_DEBUG("summing the data");
		puppeteer_msgs::Robots sorted_bots;
		sorted_bots = sort_bots_with_order(&current_bots);
		Eigen::Matrix<double, Eigen::Dynamic, 3> sorted_eig;
		bots_to_eigen(&sorted_eig, &sorted_bots);
		// now we have the sorted matrix... let's keep on
		// adding the values:
		cal_eig += sorted_eig;
		calibrate_count++;
	    }
	    // we are ready to find the transformation:
	    else
	    {
		ROS_DEBUG("Getting transforms");
		std::cout << "cal_eig: " << std::endl;
		std::cout << cal_eig;
		std::cout << std::endl;
		cal_eig /= (double) NUM_CALIBRATES; // average all vectors
		
 		std::cout << "cal_eig/30: " << std::endl;
		std::cout << cal_eig;
		std::cout << std::endl;
		
		// get transform for each robot:
		Eigen::Matrix<double, Eigen::Dynamic, 3> temp_eig;
		bots_to_eigen(&temp_eig, &start_bots);

		std::cout << "temp_eig: " << std::endl;
		std::cout << temp_eig;
		std::cout << std::endl;
		
		cal_eig = temp_eig-cal_eig;

		std::cout << "cal_eig_final: " << std::endl;
		std::cout << cal_eig;
		std::cout << std::endl;
		
		// Now find the mean of the transforms:
		for (int i=0; i<nr; i++)
		    cal_pos += cal_eig.block<1,3>(i,0);
		cal_pos /= nr;
		cal_pos = -1.0*cal_pos;

		std::cout << "cal_pos: " << std::endl;
		std::cout << cal_pos;
		std::cout << std::endl;
		
		ROS_DEBUG("calibration pose: %f, %f, %f",
			  cal_pos(0),cal_pos(1),cal_pos(2));
		calibrated_flag = true;
		calibrate_count = 0;
	    }
	    return;
	}

    // now that we are calibrated, we are free to send the transforms:
    void send_frames(void)
	{
	    tf::Transform transform;

	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    cal_pos(1),
					    cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "optimization_frame"));

	    
	    // Publish /map frame based on robot calibration
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    0,
					    cal_pos(2)));
	    transform.setRotation(tf::Quaternion(.707107,0.0,0.0,-0.707107));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "map"));
	    
	    // publish one more frame that is the frame the robot
	    // calculates its odometry in.
	    transform.setOrigin(tf::Vector3(0,0,0));
	    transform.setRotation(tf::Quaternion(1,0,0,0));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "map",
						  "robot_odom_pov"));

	    // Reset transform values for transforming data from
	    // Kinect frame into optimization frame
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    cal_pos(1), cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));

	    
	    // // Transform the received point into the actual
	    // // optimization frame, store the old values, and store the
	    // // new transformed value
	    // if (!point.error)
	    // {
	    // 	Eigen::Affine3d gwo;
	    // 	Eigen::Vector3d tmp_point; 
	    // 	tf::TransformTFToEigen(transform, gwo);
	    // 	gwo = gwo.inverse();
	    // 	tmp_point << point.x, point.y, point.z;
	    // 	tmp_point = gwo*tmp_point;

	    // 	transformed_robot_last = transformed_robot;
	    // 	transformed_robot.header.frame_id = "optimization_frame";
	    // 	transformed_robot.header.stamp = tstamp;
	    // 	transformed_robot.point.x = tmp_point(0);
	    // 	transformed_robot.point.y = tmp_point(1);
	    // 	transformed_robot.point.z = tmp_point(2);


	    // 	if (first_flag == true)
	    // 	{
	    // 	    transformed_robot_last = transformed_robot;
	    // 	    first_flag = false;
	    // 	}
	    // }

	    return;
	}
   

    // return a Robots type that has the robots[] field sorted
    // according to the "order" variable
    puppeteer_msgs::Robots sort_bots_with_order(puppeteer_msgs::Robots *r)
	{
	    ROS_DEBUG("sorting method triggered");
	    puppeteer_msgs::Robots s;
	    // let's check if the number of robots in r is the same as
	    // the number of robots in the system:
	    if ((int) r->robots.size() != nr)
	    {
		ROS_WARN("Cannot rearrange Robots msg "
			 "according to order parameter");
		s = *r;
		return s;
	    }
	    
	    // now we need to order the robots from greatest to least
	    // in terms of x-position
	    std::vector<double> pos;
	    std::vector<int> act_ord;
	    int keyi=0;
	    double key=0;
	    int j=0;
	    for (j=0; j<nr; j++) {
		pos.push_back(r->robots[j].point.x);
		act_ord.push_back(j+1);
	    }

	    j=0;
	    for (unsigned int i=1; i<pos.size(); ++i) 
	    {
		key= pos[i];
		keyi = act_ord[i];
		j = i-1;
		while((j >= 0) && (pos[j] < key))
		{
		    pos[j+1] = pos[j];
		    act_ord[j+1] = act_ord[j];
		    j -= 1;
		}
		pos[j+1]=key;
		act_ord[j+1]=keyi;
	    }
	    // now we can use the mapping from actual order to
	    // reference order to re-arrange the robots
	    s.header = r->header;
	    s.number = r->number;
	    s.robots.resize(nr);
	    for (j=0; j<nr; j++)
		s.robots[ref_ord[j]-1] = r->robots[act_ord[j]-1];
	    return s;
	}

    
// convert a Robots message to an Eigen matrix
    void bots_to_eigen(Eigen::Matrix<double, Eigen::Dynamic, 3> *e,
		       puppeteer_msgs::Robots *r)
	{
	    // first we size the matrix:
	    int num = (int) r->robots.size();
	    e->resize(num, Eigen::NoChange);
	    
	    ROS_DEBUG("Conversion to Eigen detected %d robots",num);
	    // now we can fill in the info:
	    int j=0;
	    for (j=0; j<num; j++)
	    {
		(*e)(j,0) = r->robots[j].point.x;
		(*e)(j,1) = r->robots[j].point.y;
		(*e)(j,2) = r->robots[j].point.z;
	    }

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
		    ROS_ERROR_THROTTLE(1, "Cannot determine ordering!");
		    exit(1);
		    return true;
		}

		pos.push_back(tmp);		    
		ref_ord.push_back(j+1);
	    }

	    // now we can sort the stuff
	    int keyi=0;
	    double key=0;
	    int j=0;
	    for (unsigned int i=1; i<pos.size(); ++i) 
	    {
		key= pos[i];
		keyi = ref_ord[i];
		j = i-1;
		while((j >= 0) && (pos[j] < key))
		{
		    pos[j+1] = pos[j];
		    ref_ord[j+1] = ref_ord[j];
		    j -= 1;
		}
		pos[j+1]=key;
		ref_ord[j+1]=keyi;
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
		ur = ur*robot_radius[j];
	    
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
