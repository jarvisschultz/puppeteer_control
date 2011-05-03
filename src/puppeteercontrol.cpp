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

#include <command_msgs/speed_command.h>

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

  int exit_flag;
  int i;
  int stop_flag;
  Control *robot_2;
  ros::NodeHandle n_;
  ros::Timer timer;
  ros::ServiceClient client;
  command_msgs::speed_command srv;

 
public:
  OpenLoopController() {
    i = 0;
    stop_flag = 0;
    exit_flag = 0;
    
    // note: the string at the end of this command matters
    client = n_.serviceClient<command_msgs::speed_command>("send_serial_data");
  
    ROS_INFO("Reading Controls\n");

    std::cout << working_dir << "\n";

    std::size_t found = working_dir.find("bin");

    working_dir = working_dir.substr(0, found);

    std::cout << working_dir << "\n";

    std::string file_dir = "data/";

    // Read necessary robot instructions:
    //sprintf(filename, "%s", "./TrajectoryData.txt");
    std::string filename = working_dir + file_dir + "TrajectoryData.txt";

    std::cout << filename << "\n";

    robot_2 = ReadControls(filename, 2);
    timer = n_.createTimer(ros::Duration(robot_2->DT), &OpenLoopController::timerCallback, this);

    ROS_INFO("Beginning movement execution\n");
  }

  
  void timerCallback(const ros::TimerEvent& e) {
    ROS_DEBUG("timerCallback triggered\n");

    // check to see if we are done with the list of commands
    // if we aren't done, send another command 
    if(i < robot_2->num && exit_flag == 0) {
      ROS_DEBUG("%5.2f\n",100.0*i/robot_2->num);      

      // MANUALLY SETTING THESE FOR NOW
      srv.request.robot_index = 0;
      srv.request.type = 'h';

      //srv.request.Vleft = 1.0;
      //srv.request.Vright = 1.0;
      //srv.request.Vtop = 1.0;

      srv.request.Vleft = robot_2->Vcontrols[i][0];
      srv.request.Vright = robot_2->Vcontrols[i][1];
      srv.request.Vtop = robot_2->Vcontrols[i][2];

      // NEED TO CHECK THIS
      srv.request.div = 1;

      ROS_DEBUG("Calling Service\n");
      
      if(client.call(srv)) {
	ROS_DEBUG("Send Confirmation: %i\n", srv.response.confirm_sent);
      }
      else {
	ROS_ERROR("Failed to call service: speed_command\n");
      }
    }
    // if we are done, send stop command and close data file
    else {
      //stopRobots();
      if(stop_flag == 0) ROS_INFO("Stopping Robots!\n");
      stop_flag = 1;
    }
    
    // increment count
    i++;
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
    alloc = sizeof(*robot) + sizeof(robot->Vcontrols[0])*(1495);
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
