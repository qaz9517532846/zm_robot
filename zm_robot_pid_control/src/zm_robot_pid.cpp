#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <zm_robot_pid_control/ZMPIDControllerConfig.h>
#include <zm_robot_pid_control/zm_pid_param.h>

double wheel1_p, wheel1_i, wheel1_d;
double wheel2_p, wheel2_i, wheel2_d;
double wheel3_p, wheel3_i, wheel3_d;
double wheel4_p, wheel4_i, wheel4_d;

void pid_callback(zm_robot_control_pid::ZMPIDControllerConfig &config, uint32_t level) 
{
    wheel1_p = config.wheel1_P;
    wheel1_i = config.wheel1_I; 
    wheel1_d = config.wheel1_D;
    wheel2_p = config.wheel2_P; 
    wheel2_i = config.wheel2_I; 
    wheel2_d = config.wheel2_D;
    wheel3_p = config.wheel3_P; 
    wheel3_i = config.wheel3_I; 
    wheel3_d = config.wheel3_D;
    wheel4_p = config.wheel4_P; 
    wheel4_i = config.wheel4_I; 
    wheel4_d = config.wheel4_D;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "zm_robot_pid");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<zm_robot_control_pid::ZMPIDControllerConfig> pid_server;
  dynamic_reconfigure::Server<zm_robot_control_pid::ZMPIDControllerConfig>::CallbackType f;
  
  ros::Publisher wheel1_pid = n.advertise<zm_robot_pid_control::zm_pid_param>("wheel1_PID", 50);
  ros::Publisher wheel2_pid = n.advertise<zm_robot_pid_control::zm_pid_param>("wheel2_PID", 50);
  ros::Publisher wheel3_pid = n.advertise<zm_robot_pid_control::zm_pid_param>("wheel3_PID", 50);
  ros::Publisher wheel4_pid = n.advertise<zm_robot_pid_control::zm_pid_param>("wheel4_PID", 50);

  f = boost::bind(pid_callback, _1, _2);
  pid_server.setCallback(f);

  ros::Rate loop_rate(50);

  while(n.ok())
  {
    ros::spinOnce();
    zm_robot_pid_control::zm_pid_param wheel1_PID_param;
    zm_robot_pid_control::zm_pid_param wheel2_PID_param;
    zm_robot_pid_control::zm_pid_param wheel3_PID_param;
    zm_robot_pid_control::zm_pid_param wheel4_PID_param;

    wheel1_PID_param.Controller_P = wheel1_p;
    wheel1_PID_param.Controller_I = wheel1_i;
    wheel1_PID_param.Controller_D = wheel1_d;

    wheel2_PID_param.Controller_P = wheel2_p;
    wheel2_PID_param.Controller_I = wheel2_i;
    wheel2_PID_param.Controller_D = wheel2_d;

    wheel3_PID_param.Controller_P = wheel3_p;
    wheel3_PID_param.Controller_I = wheel3_i;
    wheel3_PID_param.Controller_D = wheel3_d;

    wheel4_PID_param.Controller_P = wheel4_p;
    wheel4_PID_param.Controller_I = wheel4_i;
    wheel4_PID_param.Controller_D = wheel4_d;

    wheel1_pid.publish(wheel1_PID_param);
    wheel2_pid.publish(wheel2_PID_param);
    wheel3_pid.publish(wheel3_PID_param);
    wheel4_pid.publish(wheel4_PID_param);

    loop_rate.sleep();
  }

  return 0;
}
