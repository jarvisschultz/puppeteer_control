/* 
 * File:   WavingMain.c
 * Author: Jarvis
 *
 * Created on September 17, 2010, 10:06 PM
 */

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
#include "kbhit.h"
#include "wiiuse.h"

#define	    BAUDRATE		B115200
#define	    MODEMDEVICE		"/dev/ttyUSB0"
#define     _POSIX_SOURCE	1 /* POSIX compliant source */
#define     PI			3.141519265
#define     PACKET_SIZE		12

/***Struct Definitions:*******************************/
typedef struct
{
    int RobotMY;
    float DT;
    int num;
    float Vcontrols[][3];
} Control;

/***Function Declarations:****************************/
void sendData(int id, unsigned char *DataString);
void sendArrayEntry(int index, Control *robot);
void initComm(void);
int KeyboardInterpreter(void);
void stopRobots(void);
void BuildNumber(unsigned char *destination, float value, short int divisor);
void MakeString(unsigned char *dest, char type, float fval,
		float sval, float tval, int div);
Control *ReadControls(unsigned char *filename, unsigned int MY);


/***Global Variables:********************************/
int fd;
struct termios oldtio,newtio;
struct timespec longdelay;
struct timespec shortdelay = {0, 0};
struct timespec delayrem;
unsigned char packet_prev[128];
int exit_flag = 0;


int main(int argc, char** argv)
{
    int i = 0;
    int j = 0;
    int test = 0;
    // Implement communication:
    printf("Initializing Wireless Communication\n");
    initComm();

    printf("Reading Controls\n");
    // Read necessary robot instructions:
    Control *robot_2;
    unsigned char filename[128];
    sprintf(filename, "%s", "/home/jarvis/Dropbox/ccode/PrescriptedTrepPlay/TrajectoryData.txt");
    robot_2 = ReadControls(filename, 2);
    
    longdelay.tv_sec = 0.0;
    longdelay.tv_nsec = robot_2->DT*1000000000;
	
    printf("Beginning movement execution\n");
    // Now we begin a loop where we run through each entry of the velocity
    // arrays and send out the info contained in each one
    i = 0;
 
    while(1)
    {
	if(kbhit() && KeyboardInterpreter())
	    break;
	if(i < robot_2->num && exit_flag == 0)
	{
	    printf("%5.2f\n",100.0*i/robot_2->num);
	    sendArrayEntry(i, robot_2);
	    if(nanosleep(&longdelay,&delayrem)) printf("Nanosleep Error\n");
	}
	else
	{
	    stopRobots();
	    if(j == 0) printf("Stopping Robots!\n");
	    j = 1;
//	    assert(0);
	}
	i++;
    }

    close(fd);

    printf("Program Complete!\n");
    return 0;
}

Control *ReadControls(unsigned char *filename, unsigned int MY)
{
    FILE *fp;
    Control *robot;
    unsigned char line[128];
    float current_val, timestep;
    
    // Open file:
    if((fp = fopen(filename,"r")) == NULL)
    {
    	printf("Error Opening File!\n");
    	exit(1);
    }

    // Move to beginning of file:
    rewind(fp);

    // Now, we need to read the first line
    fscanf(fp,"%s%s%f",line,line,&timestep);
 
    // Now, we get the number of elements:
    fscanf(fp,"%s%s%s%f",line,line,line,&current_val);
    fgets(line,sizeof(line),fp);
    fgets(line,sizeof(line),fp);
    fgetc(fp);

    size_t alloc;
    alloc = sizeof(*robot) + sizeof(robot->Vcontrols[0])*(1495);
    robot = malloc(alloc);
    
    // Set robot identification:
    robot->RobotMY = MY;
    // Set timestep:
    robot->DT = timestep;
    // Set number of entries:
    robot->num = (int) current_val;

    // Iterate through the list of data and fill in the float arrays:
    int count = 0;
    int i;

    // LEFT FIRST:
    while(count < robot->num)
    {
    	fscanf(fp,"%f%s",&current_val,line);
    	robot->Vcontrols[count][0] = current_val;
    	count++;
    }
    fgetc(fp);
    fgets(line,sizeof(line),fp);
    fgets(line,sizeof(line),fp);
    fgetc(fp);

    // THEN RIGHT:
    count = 0;
    while(count < robot->num)
    {
    	fscanf(fp,"%f%s",&current_val,line);
    	robot->Vcontrols[count][1] = current_val;
    	count++;
    }
    fgetc(fp);
    fgets(line,sizeof(line),fp);
    fgets(line,sizeof(line),fp);
    fgetc(fp);
    
    // THEN TOP:
    count = 0;
    while(count < robot->num)
    {
    	fscanf(fp,"%f%s",&current_val,line);
    	robot->Vcontrols[count][2] = current_val;
    	count++;
    }

    fclose(fp);

    return(robot);
}



void stopRobots(void)
{
    char szBufferToTransfer[16];

    // Let's make the data string:
    MakeString(szBufferToTransfer, 'q', 0.0, 0.0, 0.0, 3);

    // Now let's send out the data string:
    sendData(9,szBufferToTransfer);
    if(nanosleep(&shortdelay,&delayrem)) printf("Nanosleep Error\n");;    
    sendData(1,szBufferToTransfer);
    if(nanosleep(&shortdelay,&delayrem)) printf("Nanosleep Error\n");;    
    sendData(2,szBufferToTransfer);
    if(nanosleep(&shortdelay,&delayrem)) printf("Nanosleep Error\n");;
    sendData(3,szBufferToTransfer);
}


void sendData(int id, unsigned char *DataString)
{
    unsigned char packet[128];
    int i = 0;
    unsigned short address =  0;
    unsigned char address_prev = -1;
    unsigned int checksum = 0;

    // Initialize the packet to all zeros:
    memset(packet,0,sizeof(packet));

    address = id;

    // Now we can begin filling in the packet:
    packet[0] = DataString[0];
    sprintf(&packet[1],"%1d",address);
    for(i = 2; i < PACKET_SIZE-1; i++)
	packet[i] = DataString[i-1];

    // Now, let's calculate a checksum:
    checksum = 0;
    for(i = 0; i < PACKET_SIZE-1; i++)
	checksum += packet[i];

    checksum = 0xFF-(checksum & 0xFF);

    packet[PACKET_SIZE-1] = checksum;    

    // Let's make sure that we aren't sending the same packets over and over:
    /* if(address_prev == address && memcmp(packet, packet_prev, sizeof(packet)) == 0) */
    /*     return; */
     
    address_prev = address;
    /* memcpy(packet_prev, packet, sizeof(packet)); */
    write(fd, packet, PACKET_SIZE);
    fsync(fd);
    for(i = 0; i < PACKET_SIZE; i++)
	printf("%X ",packet[i]);
    printf("\n");
}


// The following function is used for opening a com port to
// communicate with the mobile robot.
void initComm(void)
{
	/*
	  Open modem device for reading and writing and not as controlling tty
 	  because we don't want to get killed if linenoise sends CTRL-C.
	*/
     fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
     if (fd <0) {perror(MODEMDEVICE); exit(-1); }

     tcgetattr(fd,&oldtio); /* save current serial port settings */
     bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*
	  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	  CRTSCTS : output hardware flow control (only used if the cable has
		        all necessary lines. See sect. 7 of Serial-HOWTO)
	  CS8     : 8n1 (8bit,no parity)
	  CSTOPB  : enable 2 stop bits
	  CLOCAL  : local connection, no modem contol
	  CREAD   : enable receiving characters
	*/
	 newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD | CSTOPB;

	 /*
	   IGNPAR  : ignore bytes with parity errors
	   ICRNL   : map CR to NL (otherwise a CR input on the other computer
	   will not terminate input)
	   otherwise make device raw (no other input processing)
	*/
	newtio.c_iflag = IGNPAR | ICRNL;

	/*
		Raw output.
	*/
	newtio.c_oflag = 0;

	/*
		ICANON  : enable canonical input
					disable all echo functionality, and don't send
					signals to calling program
	*/
	newtio.c_lflag = ICANON;

	/*
	  	now clean the modem line and activate the settings for the port
	*/
	 tcflush(fd, TCIFLUSH);
	 tcsetattr(fd,TCSANOW,&newtio);
}

// This function gets called if we have detected that the user is pressing a key
// on the keyboard.
int KeyboardInterpreter(void)
{
       int c = 0;
       c = fgetc(stdin);
       if(c == 'q') return 1;
       printf("Emergency Stop!!\n");

       exit_flag = 1;
       return 0;
}

// This function will actually send out the array data.
void sendArrayEntry(int index, Control *robot)
{
    unsigned char dataPtr[128];
 
    // Now, we can send out each velocity for robot 3:
    MakeString(dataPtr, 'h', robot->Vcontrols[index][0], robot->Vcontrols[index][1], robot->Vcontrols[index][2], 3);
    sendData(robot->RobotMY, dataPtr);
    if(nanosleep(&shortdelay,&delayrem)) printf("Nanosleep Error\n");
}


void BuildNumber(unsigned char *destination, float value, short int divisor)
{
    int valint = 0;
    int i = 0;
    short int i1;
    short unsigned int i2, i3;
    // First thing is to move the decimal point of the integer to the
    // right a "divisor" number of times:
    for(i = 0; i<3; i++) value = value*10.0;
    valint = (int) value;
    // Now build the three chars:
    i1 = ((valint<<3) & 0xFF0000)>>16;
    i2 = ((valint<<3) & 0x00FF00)>>8;
    i3 = ((valint<<3) & 0x0000FF);
    i3 = (((i3)&0xF0)) + ((i3&0x0F)|divisor);

    // Now, place the chars in the array:
    *(destination) = i1;
    *(destination+1) = i2;
    *(destination+2) = i3;
        
    return;
}


void MakeString(unsigned char *dest, char type, float fval,
		float sval, float tval, int div)
{
    *dest = type;
    BuildNumber((dest+1), fval, 3);
    BuildNumber((dest+4), sval, 3);
    BuildNumber((dest+7), tval, 3);
}
