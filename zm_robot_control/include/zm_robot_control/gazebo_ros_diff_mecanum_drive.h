#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH
 
#include <map>
 
// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
 
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
 
// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
 
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
 
namespace gazebo {
 
  class Joint;
  class Entity;
 
  class GazeboRosDiffDrive : public ModelPlugin {
 
    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      GazeboRosDiffDrive();
      ~GazeboRosDiffDrive();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();
 
    protected:
      virtual void UpdateChild();
      virtual void FiniChild();
 
    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();
      void publishWheelTF(); 
      void publishWheelJointState();
      void UpdateOdometryEncoder();
 
 
      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;
 
      double wheel_separation_;
      double wheel_diameter_;
      double wheel_torque;
      double wheel_speed_[2];
          double wheel_accel;
      double wheel_speed_instr_[2];
 
      std::vector<physics::JointPtr> joints_;
 
      // ROS STUFF
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      sensor_msgs::JointState joint_state_;
      ros::Publisher joint_state_publisher_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;
 
      boost::mutex lock;
 
      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      bool publish_tf_;
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();
 
      // DiffDrive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
 
      double x_;
      double y_;
      double rot_;
      bool alive_;
 
      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
 
      OdomSource odom_source_;
      geometry_msgs::Pose2D pose_encoder_;
      common::Time last_odom_update_;
 
      // Flags
      bool publishWheelTF_;
      bool publishOdomTF_;
      bool publishWheelJointState_;
 
  };
 
}
 
#endif
