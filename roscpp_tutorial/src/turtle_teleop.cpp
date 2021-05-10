#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

void print_info(void);
int getch(void);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_teleop");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  geometry_msgs::Twist t;
  ros::Rate loop_rate(10);

  t.linear.x  = t.linear.y  = t.linear.z  = 0.0;
  t.angular.x = t.angular.y = t.angular.z = 0.0;

  int ch = 0, cnt = 0;

  print_info();

  while(ch != 'Q')
  {
    ch = getch();
    
    if(ch == 'w') {
      t.linear.x =  2.0;   t.angular.z =  0.0;  cnt++;
    }
    else if(ch == 's') {
      t.linear.x = -2.0;   t.angular.z =  0.0;  cnt++;
    }
    else if(ch == 'a') {
      t.linear.x =  0.0;   t.angular.z =  2.0;  cnt++;
    }
    else if(ch == 'd') {
      t.linear.x =  0.0;   t.angular.z = -2.0;  cnt++;
    }
    else;

    if(cnt == 10) {
      cnt = 0;  print_info();
    }

    pub.publish(t);

    t.linear.x  = t.linear.y  = t.linear.z  = 0.0;
    t.angular.x = t.angular.y = t.angular.z = 0.0;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void print_info()
{
  puts("Remote Control turtle of turtlesim_node");
  puts("---------------------------------------");
  puts("               (forward)               ");
  puts("                   w                   ");
  puts("  (turn-left) a    s    d (turn-right) ");
  puts("                (back)                 ");
  puts("---------------------------------------");
  puts("### type Q to quit                     ");
}

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR  | ICRNL | IXON  | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO  | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}
