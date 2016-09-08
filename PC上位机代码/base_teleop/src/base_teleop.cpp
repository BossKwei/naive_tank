#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

/*#define KEYDOWN_W 119
#define KEYDOWN_S 115
#define KEYDOWN_A 97
#define KEYDOWN_D 100*/


int getch()
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

#define KEYDOWN_W 119
#define KEYDOWN_S 115
#define KEYDOWN_A 97
#define KEYDOWN_D 100

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_teleop");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cmd_vel/base", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
	std_msgs::String msg;
    switch(getch())
    {
    case KEYDOWN_W:msg.data="W";break;
    case KEYDOWN_S:msg.data="S";break;
    case KEYDOWN_A:msg.data="A";break;
    case KEYDOWN_D:msg.data="D";break;
    default:break;
    }

    ROS_INFO("Publish [%s]", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    //loop_rate.sleep();

    ++count;
  }

  return 0;
}
