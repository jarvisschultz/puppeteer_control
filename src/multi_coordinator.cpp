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

#include <vector>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
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
#define NUM_EKF_INITS (5)
#define DEFAULT_ERR  (10)

//---------------------------------------------------------------------------
// Prototypes
//---------------------------------------------------------------------------
void add_to_tab(char reset, int num, int *p, int **tab);
void move_perm_entry( int x, int d, int *p, int *pi);
void permute(int n, int num, int *dir, int *p, int *pi, int **tab);
void generate_perm_table(int num, int **tab);

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
    bool calibrated_flag, gen_flag;
    unsigned int calibrate_count;
    ros::Time tstamp;
    std::vector<int> ref_ord;
    int operating_condition;
    std::vector<double> robot_radius;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry kin_pose[MAX_ROBOTS];
    puppeteer_msgs::Robots current_bots, prev_bots, start_bots, cal_bots;
    puppeteer_msgs::Robots current_bots_sorted, prev_bots_sorted;
    puppeteer_msgs::Robots desired_bots;
    Eigen::Vector3d cal_pos;
    int **tab;
    int height;
    std::vector<double> robot_start_ori;
    bool *bad_array;
    boost::array<double,36ul> kincov;

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

	    // creating subscribers
	    
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
	gen_flag = true;
	calibrated_flag = false;
	tstamp = ros::Time::now();
	    
	// set covariance for the pose messages
	double kin_cov_dist = 0.5;	// in meters^2
	double kin_cov_ori = 100.0;	// radians^2
	boost::array<double,36ul> tmp = {{kin_cov_dist, 0, 0, 0, 0, 0,
					  0, kin_cov_dist, 0, 0, 0, 0,
					  0, 0,        99999, 0, 0, 0,
					  0, 0, 0,        99999, 0, 0,
					  0, 0, 0, 0,        99999, 0,
					  0, 0, 0, 0, 0,  kin_cov_ori}};
	kincov = tmp;

	for (int i=0; i<nr; i++)
	    kin_pose[i].pose.covariance = kincov;

	// initialize the size of the sorted bots messages:
	current_bots_sorted.robots.resize(nr);
	prev_bots_sorted.robots.resize(nr);

	// allocate memory for the permuatation table:
	height = 1;
	for (int j=1; j<=nr; j++)
	    height *= j;
	tab = new int*[height];
	for (int j=0; j<height; j++) 
	    tab[j] = new int[nr];
	generate_perm_table(nr, tab);

	// allocate memory for bad_array
	bad_array = new bool[nr];

	return;
    }


    void print_bots(const std::string name, const puppeteer_msgs::Robots &b)
	{
	    const puppeteer_msgs::Robots *bptr;
	    bptr = &b;
	    
	    std::cout << "Name = " << name << std::endl;
	    std::cout << "Pointer = ";
	    printf("%p\r\n", bptr);

	    std::cout << "Total number of robots = "
		      << b.robots.size() << std::endl;

	    for(int j=0; j<((int) b.robots.size()); j++)
		printf("x = %f, y = %f, z = %f\r\n",
		       b.robots[j].point.x,
		       b.robots[j].point.y,		
		       b.robots[j].point.z);
	    		    
	    std::cout << std::endl;
	    return;
	}

    
    
    void datacb(const puppeteer_msgs::Robots &bots)
	{
	    ROS_DEBUG("datacb triggered with OC = %d",operating_condition);
	    static bool first_flag = true;

	    // if we aren't calibrating or running, let's just exit
	    // this cb
	    if (operating_condition != 2 && operating_condition != 1)
		return;
	    puppeteer_msgs::Robots b;
	    b.robots.resize(bots.robots.size());
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
	    if ( !calibrated_flag )
	    {
		if ((int) current_bots.robots.size() == nr)
		{
		    prev_bots_sorted = current_bots_sorted;
		    current_bots_sorted = calibrate_routine();
		}
	    	return;
	    }
	    
	    // send all relevant transforms
	    if (calibrated_flag)
		send_frames();

	    // If we got here, we are calibrated.  That means we can
	    // sort robots based on previous locations
	    puppeteer_msgs::Robots tmp;
	    tmp.robots.resize(current_bots_sorted.robots.size());
	    tmp = current_bots_sorted;
	    // print_bots("tmp_bots (before)",tmp);
	    current_bots_sorted = associate_robots(current_bots, prev_bots_sorted);
	    prev_bots_sorted = tmp;

	    return;
	}
	
    

    void timercb(const ros::TimerEvent& e)
	{
	    static int num_delays = 0;
	    ROS_DEBUG("timercb triggered");
	    if (gen_flag)
	    {
		// Generate the robot ordering vector
		gen_flag = generate_order();
		return;
	    }

	    // get operating condition
	    if (ros::param::has("/operating_condition"))
		ros::param::get("/operating_condition", operating_condition);
	    else
	    {
		operating_condition = 4;
		ros::param::set("/operating_condition", operating_condition);
	    }
	    
	    // check to see if we are in run state
	    if(operating_condition == 1 || operating_condition == 2)
	    {
		if(calibrated_flag)
		{
		    if (num_delays > NUM_EKF_INITS)
			process_robots();
		    else
			num_delays++;
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
	    num_delays = 0;
	    return;
	}
    

    puppeteer_msgs::Robots calibrate_routine(void)
	{
	    ROS_DEBUG("calibration_routine triggered");
	    static Eigen::MatrixXd cal_eig(nr,3);
	    puppeteer_msgs::Robots sorted_bots;
	    sorted_bots.robots.resize(nr);
		
	    // if this is the first call to the function, let's get
	    // the starting pose for each of the robots.
	    if(calibrate_count == 0)
	    {
		ROS_DEBUG_THROTTLE(1,"Calibrating...");
		puppeteer_msgs::Robots r;
		double tmp;
		r.robots.resize(nr);
		robot_start_ori.clear();
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

		    ss << "/robot_" << j+1 << "/robot_th0";
		    ros::param::get(ss.str(), tmp);
		    robot_start_ori.push_back(tmp);
		}

		start_bots = r;
		
		// increment counter, and initialize transform values
		calibrate_count++;
		cal_pos << 0, 0, 0;
		for(int i=0; i<cal_eig.rows(); i++) {
		    cal_eig(i,0) = 0; cal_eig(i,1) = 0; cal_eig(i,2) = 0; }
		return sorted_bots;
	    }
	    // we are in the process of calibrating:
	    else if (calibrate_count <= NUM_CALIBRATES)
	    {
		ROS_DEBUG("summing the data");
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
		cal_eig /= (double) NUM_CALIBRATES; // average all vectors
		
		// get transform for each robot:
		sorted_bots = sort_bots_with_order(&current_bots);
		Eigen::Matrix<double, Eigen::Dynamic, 3> temp_eig;
		bots_to_eigen(&temp_eig, &start_bots);
		cal_eig = temp_eig-cal_eig;

		// Now find the mean of the transforms:
		for (int i=0; i<nr; i++)
		    cal_pos += cal_eig.block<1,3>(i,0);
		cal_pos /= nr;
		cal_pos = -1.0*cal_pos;

		ROS_DEBUG("calibration pose: %f, %f, %f",
			  cal_pos(0),cal_pos(1),cal_pos(2));
		calibrated_flag = true;
		calibrate_count = 0;
	    }
	    return sorted_bots;
	}

    // now that we are calibrated, we are free to send the transforms:
    void send_frames(void)
	{
	    ROS_DEBUG("Sending frames");
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
	    ROS_DEBUG("frames sent");
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
	    ref_ord.clear();
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

    // This function takes references to two Robot messages, a current
    // and a last.  It sorts the data such that the returned Robots
    // message has the same data as the current message, but it is
    // sorted according to the last message order.
    puppeteer_msgs::Robots associate_robots(
	puppeteer_msgs::Robots c,
	puppeteer_msgs::Robots l)
	{
	    ROS_DEBUG("Attempting data association problem");
	    puppeteer_msgs::Robots s;
	    s.robots.resize(nr);

	    ROS_DEBUG("Filling in missing points");
	    // first let's fill in the current Robots message that may
	    // be missing data
	    double def = DEFAULT_ERR;
	    geometry_msgs::PointStamped err_pt;
	    err_pt.header = c.robots[0].header;
	    err_pt.point.x = def;
	    err_pt.point.y = def;
	    err_pt.point.z = def;
	    for (int i=0; i<(nr- (int) c.robots.size()); i++)
		c.robots.push_back(err_pt);

	    // now for each permutation, we can calculate a norm:
	    Eigen::Matrix<double, Eigen::Dynamic, 3> bot_eig, clust_eig;
	    ROS_DEBUG("Converting bots to eig");
	    bots_to_eigen(&bot_eig, &l);
	    bots_to_eigen(&clust_eig, &c);
	    Eigen::VectorXd dist(height);
	    double err = 0;
	    ROS_DEBUG("Testing all permutations");
	    for (int i=0; i<height; i++)
	    {
		err = 0;
		for (int j=0; j<nr; j++)
		{
		    Eigen::Vector3d bvec = bot_eig.block<1,3>(j,0);
		    Eigen::Vector3d cvec = clust_eig.block<1,3>(tab[i][j]-1,0);
		    err += (bvec-cvec).norm();
		}
		dist(i) = err;
	    }

	    ROS_DEBUG("Finding the minimum");
	    // now find the entry in dist that has the minimum value:
	    int key = find_minimum_index(dist);

	    // now, use the mapping defined by key to fill out s:
	    s.header = c.header;
	    s.number = nr;
	    for (int i=0; i<nr; i++)
		s.robots[i] = c.robots[tab[key][i]-1];
	    // fill in error array:
	    for (int j=0; j<nr; j++)
	    {
		if (s.robots[j].point.x > DEFAULT_ERR-0.1)
		    bad_array[j] = true;
		else
		    bad_array[j] = false;
	    }
	    	    
	    return s;
	}


    // This function takes in an Eigen::VectorXd, and returns the
    // index to its minimum element:
    int find_minimum_index(Eigen::VectorXd v)
	{
	    std::vector<double> tmp;
	    // convert VectorXd to a std::vector
	    for (int i=0; i< (int) v.size(); i++)
		tmp.push_back(v(i));
	    return ((int) distance(tmp.begin(),
				   min_element(tmp.begin(), tmp.end())));
	}
    

    // process_robots simply iterates through a sorted list of robots,
    // and sends the appropriate transforms and topics
    void process_robots(void)
	{
	    for (int i=0; i<nr; i++)
	    {
		send_kinect_estimate(current_bots_sorted.robots[i],
				     prev_bots_sorted.robots[i], i);
	    }
	    return;
	}
	
    

    // This function is responsible for converting the data from the
    // kinect into an odometry message, and tranforming it to the same
    // frame that the ekf uses    
    void send_kinect_estimate(geometry_msgs::PointStamped pt,
			      geometry_msgs::PointStamped ptlast, int index)
	{
	    ROS_DEBUG("send_kinect_estimate triggered");

	    Eigen::Affine3d gwo;
	    Eigen::Vector3d tmp_point; 
	    tf::Transform transform;
	    geometry_msgs::PointStamped transpt, transptlast;
	    ros::Time tstamp = pt.header.stamp;
	    

	    // set transform parameters
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    cal_pos(1), cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));

	    // convert to Eigen:
	    tf::TransformTFToEigen(transform, gwo);
	    gwo = gwo.inverse();

	    tmp_point << pt.point.x, pt.point.y, pt.point.z;
	    tmp_point = gwo*tmp_point;
	    transpt.header.frame_id = "optimization_frame";
	    transpt.header.stamp = tstamp;
	    transpt.point.x = tmp_point(0);
	    transpt.point.y = tmp_point(1);
	    transpt.point.z = tmp_point(2);

	    tmp_point << ptlast.point.x, ptlast.point.y, ptlast.point.z;
	    tmp_point = gwo*tmp_point;
	    transptlast.header.frame_id = "optimization_frame";
	    transptlast.header.stamp = tstamp;
	    transptlast.point.x = tmp_point(0);
	    transptlast.point.y = tmp_point(1);
	    transptlast.point.z = tmp_point(2);
	    
	    ROS_DEBUG("Done transforming PointStamped messages");
	    // Let's first get the transform from /optimization_frame
	    // to /map
	    tf::StampedTransform trans_stamped;
	    geometry_msgs::PointStamped tmp;
	    try{
	    	tf.lookupTransform(
	    	    "map", "optimization_frame",
	    	    tstamp, trans_stamped);
	    	tf.transformPoint("map", transpt, tmp);

	    }
	    catch(tf::TransformException& ex){
	    	ROS_ERROR(
	    	    "Error trying to lookupTransform from /map "
	    	    "to /optimization_frame: %s", ex.what());
	    	return;
	    }
	    
	    // Now we can publish the Kinect's estimate of the robot's
	    // pose
	    std::stringstream ss;
	    ss << "base_footprint_kinect_robot_" << index;
	    kin_pose[index].header.stamp = ros::Time::now();
	    kin_pose[index].header.frame_id = "map";
	    kin_pose[index].child_frame_id = ss.str();
	    tmp.point.z = 0.0;
	    kin_pose[index].pose.pose.position = tmp.point;
	    double theta = 0.0;
	    if (operating_condition == 2)
	    {
	    	theta = atan2(transpt.point.x-
	    			     transptlast.point.x,
	    			     transpt.point.z-
	    			     transptlast.point.z);
	    	theta = clamp_angle(theta-M_PI/2.0);
		ROS_DEBUG("Calculated angle = %f",theta);					  
	    }
	    else
	    {
	    	theta = robot_start_ori[index];
	    	theta = clamp_angle(-theta);
		ROS_DEBUG("predetermined angle = %f",theta);
	    }
	    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	    kin_pose[index].pose.pose.orientation = quat;

	    // Let's check if this is a "bad" point
	    if (bad_array[index])
	    {
		boost::array<double,36ul> tmpcov;
		for (int i=0; i<36; i++)
		    tmpcov[i] = kincov[i]*1000.0;
		kin_pose[index].pose.covariance = tmpcov;
	    }
	    else
		kin_pose[index].pose.covariance = kincov;
	    
	    ROS_DEBUG("Done filling in Odometry message");

	    // Now let's publish the estimated pose as a
	    // nav_msgs/Odometry message on a topic called /vo
	    robots_pub[index].publish(kin_pose[index]);

	    // now, let's publish the transforms that goes along with it
	    geometry_msgs::TransformStamped kin_trans;
	    tf::Quaternion q1, q2;
	    q1 = tf::createQuaternionFromYaw(theta);
	    q2 = tf::Quaternion(1.0,0,0,0);
	    q1 = q1*q2;
	    tf::quaternionTFToMsg(q1, quat);

	    kin_trans.header.stamp = tstamp;
	    kin_trans.header.frame_id = kin_pose[index].header.frame_id;
	    kin_trans.child_frame_id = kin_pose[index].child_frame_id;
	    kin_trans.transform.translation.x = kin_pose[index].pose.pose.position.x;
	    kin_trans.transform.translation.y = kin_pose[index].pose.pose.position.y;
	    kin_trans.transform.translation.z = kin_pose[index].pose.pose.position.z;
	    kin_trans.transform.rotation = quat;

	    ROS_DEBUG("Sending transform for output of estimator node");
	    br.sendTransform(kin_trans);
	    
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
// PERMUTATION FUNCTIONS
//---------------------------------------------------------------------------

void add_to_tab(char reset, int num, int *p, int **tab)
{
    static int count=0;
    if (reset)
	count = 0;
    int i;
    for (i=1; i <= num; ++i)
	tab[count][i-1]=p[i];
    count++;
}

void move_perm_entry( int x, int d, int *p, int *pi)
{
   int z;
   z = p[pi[x]+d];
   p[pi[x]] = z;
   p[pi[x]+d] = x;
   pi[z] = pi[x];
   pi[x] = pi[x]+d;
} 


void permute(int n, int num, int *dir, int *p, int *pi, int **tab)
{
    int i;
    if (n > num)
	add_to_tab(0, num, p, tab);
    else
    {
    	permute(n+1, num, dir, p, pi, tab );
    	for (i=1; i<=n-1; ++i)
    	{
    	    move_perm_entry(n, dir[n], p, pi);
    	    permute(n+1, num, dir, p, pi, tab);
    	}
    	dir[n] = -dir[n];
    } 
}


void generate_perm_table(int num, int **tab)
{
    int *perm, *dir, *permi;
    int height = 1;
    for (int j=1; j<=num; j++)
	height *= j;
    // allocate memory:
    perm = new int[num];
    permi = new int[num];
    dir = new int[num];

    // initialize variables:
    perm[0]=0;
    permi[0]=0;
    dir[0]=0;
    for (int i=1; i<=num; ++i)
    {
	dir[i] = -1; perm[i] = i;
	permi[i] = i;
    }

    // build permutations and add to tab
    permute(1, num, dir, perm, permi, tab);

    delete [] dir;
    delete [] perm;
    delete [] permi;
    
    return;    
}





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

