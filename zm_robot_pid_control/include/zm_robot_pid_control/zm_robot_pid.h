#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <zm_robot_pid_control/agv_pid_position.h>


namespace gazebo
{
  class Mecanum : public ModelPlugin {
  public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void UpdateChild(const common::UpdateInfo & /*_info*/);
      
      void wheel1(const std_msgs::Float64::ConstPtr& msg);
      void wheel2(const std_msgs::Float64::ConstPtr& msg);
      void wheel3(const std_msgs::Float64::ConstPtr& msg);
      void wheel4(const std_msgs::Float64::ConstPtr& msg);
  private:
      // Pointer to the model
      physics::ModelPtr model;

      sdf::ElementPtr sdf;

      // wheel joint1
      physics::JointPtr joint_1;

      // wheel joint2
      physics::JointPtr joint_2;
        
      // wheel joint3
      physics::JointPtr joint_3;
        
      // wheel joint4
      physics::JointPtr joint_4;

      /// \brief A PID controller for the joint.
      common::PID wheel1_pid;
      common::PID wheel2_pid;
      common::PID wheel3_pid;
      common::PID wheel4_pid;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      boost::shared_ptr<ros::NodeHandle> mRosnode;

      // input wheel velocity.
      double wheel_1_value;
      double wheel_2_value;
      double wheel_3_value;
      double wheel_4_value;

      ros::Subscriber mfl_sub;
      ros::Subscriber mfr_sub;
      ros::Subscriber mbl_sub;
      ros::Subscriber mbr_sub;

      // publisher joint_state
      ros::Publisher joint_pub;

      // publisher agv position
      ros::Publisher pose_pub;

      sensor_msgs::JointState joint_states_;

      zm_robot_pid_control::agv_pid_position zm_robot_pos;

      double joint_1_position;
      double joint_2_position;
      double joint_3_position;
      double joint_4_position;

      // 4 wheel pid controller parameter.
      double wheel1_p;
      double wheel1_i;
      double wheel1_d;
      double wheel2_p;
      double wheel2_i;
      double wheel2_d;
      double wheel3_p;
      double wheel3_i;
      double wheel3_d;
      double wheel4_p;
      double wheel4_i;
      double wheel4_d;

      // 4 wheel pid controller output.
      double wheel1_output;
      double wheel2_output;
      double wheel3_output;
      double wheel4_output;

      ros::Publisher wheel1_vel;
      ros::Publisher wheel2_vel;
      ros::Publisher wheel3_vel;
      ros::Publisher wheel4_vel;

      std_msgs::Float64 wheel_1_output;
      std_msgs::Float64 wheel_2_output;
      std_msgs::Float64 wheel_3_output;
      std_msgs::Float64 wheel_4_output;

      ros::Time current_time;
      ros::Time last_time;
   };
}
