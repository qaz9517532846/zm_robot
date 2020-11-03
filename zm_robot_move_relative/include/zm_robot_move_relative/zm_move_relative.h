#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <zm_robot_move_relative/zm_robot_MoveRelativeConfig.h>

class zm_robot_MoveRelative
{
    public:
      zm_robot_MoveRelative(std::string name);
      ~zm_robot_MoveRelative();

      void reconfig_callback(zm_robot_move_relative::zm_robot_MoveRelativeConfig &config, uint32_t level);
      void execute(const move_base_msgs::MoveBaseGoalConstPtr &goal);

    private:
      ros::NodeHandle nh_;

      actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> action_server_;
      dynamic_reconfigure::Server<zm_robot_move_relative::zm_robot_MoveRelativeConfig> reconfigure_server_;

      tf2_ros::Buffer tf_Buffer_;
      tf2_ros::TransformListener *tf_listener_;

      ros::Publisher goal_pub_;
      ros::Publisher vel_pub_;

      struct velocity_setting
      {
          double max_vel;
          double min_vel;
          double acceleration;
      };

      velocity_setting x_, y_, theta_;

      std::string fixed_frame_;
      std::string base_frame_;

      double rate_;
      double timeout_;
      double linear_tolerance_sq_, angular_tolerance_;

      double cal_vel(double difference, velocity_setting &set);
      void stop_vel();
};