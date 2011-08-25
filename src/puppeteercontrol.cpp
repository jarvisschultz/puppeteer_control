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
#include <time.h>
#include <assert.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

std::string working_dir;


//---------------------------------------------------------------------------
// Class Definitions
//---------------------------------------------------------------------------

class OpenLoopController {

private:
    typedef struct
    {
	int RobotMY;
	float DT;
	int num;
	float Vcontrols[][3];
    } Control;

    int i;
    int operating_condition;
    bool exit_flag;
    bool stop_flag;
    bool start_flag;
    Control *robot;
    ros::NodeHandle n_;
    ros::Timer timer;
    ros::ServiceClient client;
    puppeteer_msgs::speed_command srv;

 
public:
  OpenLoopController() {
    i = 0;
    stop_flag = false;
    exit_flag = false;
    start_flag = true;
    
    // note: the string at the end of this command matters
    client = n_.serviceClient<puppeteer_msgs::speed_command>("speed_command");
  
    ROS_INFO("Reading Controls\n");

    std::size_t found = working_dir.find("bin");  // find the string bin in the path
    working_dir = working_dir.substr(0, found);  // get substring from the beginning of path to location of string "bin"
    std::string file_dir = "data/";  // set name of directory where data is kept

    // Read necessary robot instructions:
    //sprintf(filename, "%s", "./TrajectoryData.txt");
    std::string filename = working_dir + file_dir + "TrajectoryData.txt";  // concatenate all of these strings together to get the new path


    // check for operating_condition parameter from keyboard_node
    if(ros::param::has("operating_condition")) {
      // set operating_condition to idle so the robot doesn't drive
      ros::param::set("operating_condition", 0);
    }
    else {
      ROS_WARN("Cannot Find Parameter: operating_condition");
      return;
    }


    // read file into robot struct
    robot = ReadControls(filename, 1);

    // create callback timer that will run with a period of DT from robot struct
    timer = n_.createTimer(ros::Duration(robot->DT), &OpenLoopController::timerCallback, this);
  }

  
  void timerCallback(const ros::TimerEvent& e) {
    //ROS_DEBUG("timerCallback triggered\n");

    // get operating_condition
    if(ros::param::has("operating_condition")) {
      ros::param::get("/operating_condition", operating_condition);
    }
    else {
      return;
    }
	
    // check robot_state_req and act accordingly
    // are we in idle mode?
    if(operating_condition == 0 || operating_condition == 1) {
      // reset flags
      start_flag = true;
      stop_flag = false;
      i = 0;  // reset line number to zero
    }
  
    // are we in run mode?
    else if(operating_condition == 2) {
      if(start_flag == true) {
	ROS_INFO("Beginning movement execution\n");
	srv.request.robot_index = robot->RobotMY;
	srv.request.type = 'm';
	srv.request.Vleft = 0.0;
	srv.request.Vright = 0.0;
	srv.request.Vtop = 0.0;
	srv.request.div = 0;
      }
      // check to see if we are done with the list of commands
      // if we aren't done, load next command to service request 
      else if(i < robot->num && start_flag == false) {
	ROS_DEBUG("%5.2f\n",100.0*i/robot->num);      
	    
	srv.request.robot_index = robot->RobotMY;
	srv.request.type = 'h';  // NEED TO CHECK THIS
	srv.request.Vleft = robot->Vcontrols[i][0];
	srv.request.Vright = robot->Vcontrols[i][1];
	srv.request.Vtop = robot->Vcontrols[i][2];
	srv.request.div = 3;  // NEED TO CHECK THIS
	    
	i++;
	ROS_DEBUG("Calling Service\n");
      }
      else {
	ros::param::set("operating_condition", 3);  // set state to stop
      }

      // send request to service
      if(client.call(srv)) {
	if(srv.response.error == false) {
	  ROS_DEBUG("Send Successful: speed_command\n");
	}
	else {
	  ROS_DEBUG("Send Request Denied: speed_command\n");
	  static bool request_denied_notify = true;
	  if(request_denied_notify) {
	    ROS_INFO("Send Requests Denied: speed_command\n");
	    request_denied_notify = false;
	  }
	}
      }
      else {
	ROS_ERROR("Failed to call service: speed_command\n");
      }
      
      start_flag = false;
    }
	
    // have we gotten a stop request or an emergency stop request?
    else if(operating_condition == 3 || operating_condition == 4) {
      if(operating_condition == 4) ROS_WARN("Emergency Stop Detected");

      srv.request.robot_index = robot->RobotMY;
      srv.request.type = 'q';
      srv.request.Vleft = 0.0;
      srv.request.Vright = 0.0;
      srv.request.Vtop = 0.0;
      srv.request.div = 3;

      if(stop_flag == false) ROS_INFO("Stopping Robots!\n");
      stop_flag = true;

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


  Control *ReadControls(std::string filename, unsigned int MY)
  {
    FILE *fp;
    Control *robot;
    char line[128];
    float current_val, timestep;
    
    // Open file:
    if((fp = fopen(filename.c_str(),"r")) == NULL)
      {
	printf("Error Opening File!\n");
	exit(1);
      }

    // Move to beginning of file:
    rewind(fp);

    // Now, we need to read the first line
    if(fscanf(fp,"%s%s%f",line,line,&timestep) == EOF) ROS_ERROR("fscanf failure");
 
    // Now, we get the number of elements:
    if(fscanf(fp,"%s%s%s%f",line,line,line,&current_val) == EOF) ROS_ERROR("fscanf failure");

    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");

    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");

    if(fgetc(fp) == EOF) ROS_ERROR("fscanf failure");

    size_t alloc;
    alloc = sizeof(*robot) + sizeof(robot->Vcontrols[0])*((int) current_val);
    robot = (Control*) malloc(alloc);
    
    // Set robot identification:
    robot->RobotMY = MY;
    // Set timestep:
    robot->DT = timestep;
    // Set number of entries:
    robot->num = (int) current_val;

    // Iterate through the list of data and fill in the float arrays:
    int count = 0;

    // LEFT FIRST:
    while(count < robot->num)
      {
	if(fscanf(fp,"%f%s",&current_val,line) == EOF) ROS_ERROR("fscanf failure");
	robot->Vcontrols[count][0] = current_val;
	count++;
      }
    fgetc(fp);
    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");
    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");
    if(fgetc(fp) == EOF) ROS_ERROR("fscanf failure");

    // THEN RIGHT:
    count = 0;
    while(count < robot->num)
      {
	if(fscanf(fp,"%f%s",&current_val,line) == EOF) ROS_ERROR("fscanf failure");
	robot->Vcontrols[count][1] = current_val;
	count++;
      }
    if(fgetc(fp) == EOF) ROS_ERROR("fscanf failure");
    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");
    if(*fgets(line,sizeof(line),fp) == EOF) ROS_ERROR("fscanf failure");
    if(fgetc(fp) == EOF) ROS_ERROR("fscanf failure");
    
    // THEN TOP:
    count = 0;
    while(count < robot->num)
      {
	if(fscanf(fp,"%f%s",&current_val,line) == EOF) ROS_ERROR("fscanf failure");
	robot->Vcontrols[count][2] = current_val;
	count++;
      }

    return(robot);
  }
};


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  working_dir = argv[0];

  // startup node
  ros::init(argc, argv, "puppeteer_control");
  ros::NodeHandle n;
  
  OpenLoopController controller1;

  // infinite loop
  ros::spin();

  return 0;
}
