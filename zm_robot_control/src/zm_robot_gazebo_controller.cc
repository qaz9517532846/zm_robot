#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>

// These need to be pulled out to parameters...
const float WHEEL_RAD = 0.065; // meters
const float WHEELBASE = 0.6; // meters
const float TRACK = 0.5; // meters

namespace gazebo
{
  class Mecanum : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      wheel_1_value = 0;
      wheel_2_value = 0;
      wheel_3_value = 0;
      wheel_4_value = 0;

      joint_1_position = 0;
      joint_2_position = 0;
      joint_3_position = 0;
      joint_4_position = 0;

      wheel_1_num = 0;
      wheel_2_num = 0;
      wheel_3_num = 0;
      wheel_4_num = 0;
 
      this->model = _parent;
      
      mRosnode.reset(new ros::NodeHandle(""));

      mfl_sub = mRosnode->subscribe("/wheel2_velocity", 1000, &Mecanum::wheel2, this);
      mfr_sub = mRosnode->subscribe("/wheel3_velocity", 1000, &Mecanum::wheel3, this);
      mbl_sub = mRosnode->subscribe("/wheel1_velocity", 1000, &Mecanum::wheel1, this);
      mbr_sub = mRosnode->subscribe("/wheel4_velocity", 1000, &Mecanum::wheel4, this);

      joint_pub = mRosnode->advertise<sensor_msgs::JointState>("joint_states", 1000);

      pose_pub = mRosnode->advertise<geometry_msgs::Pose2D>("zm_robot_position", 1000);

      joint_states_.name.push_back("wheel_joint1");
      joint_states_.name.push_back("wheel_joint2");
      joint_states_.name.push_back("wheel_joint3");
      joint_states_.name.push_back("wheel_joint4");
      joint_states_.position.resize(4, 0.0);
      joint_states_.velocity.resize(4, 0.0);
      joint_states_.effort.resize(4, 0.0);

      current_time = ros::Time::now();
      last_time = ros::Time::now();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Mecanum::UpdateChild, this, _1));
      }

      public: void UpdateChild(const common::UpdateInfo & /*_info*/)
      {
        current_time = ros::Time::now(); 
        double dt = (current_time - last_time).toSec();

        joint_1_position = joint_1_position + wheel_1_value * dt ;
        joint_2_position = joint_2_position + wheel_2_value * dt ;
        joint_3_position = joint_3_position + wheel_3_value * dt ;
        joint_4_position = joint_4_position + wheel_4_value * dt ;

        joint_states_.position[0] = joint_1_position;
        joint_states_.position[1] = joint_2_position;
        joint_states_.position[2] = joint_3_position;
        joint_states_.position[3] = joint_4_position;
        joint_states_.velocity[0] = wheel_1_value;
        joint_states_.velocity[1] = wheel_2_value;
        joint_states_.velocity[2] = wheel_3_value;
        joint_states_.velocity[3] = wheel_2_value;

        float r = WHEEL_RAD;
        float L = TRACK; // left -> right
        float W = WHEELBASE; // front -> back
        float l = 1.0 / (2 * (W + L));
        float x = r * ( wheel_2_value / 4.0 + wheel_3_value / 4.0 + wheel_1_value / 4.0 + wheel_4_value / 4.0);
        float y = r * ( -wheel_2_value / 4.0 + wheel_3_value / 4.0 + wheel_1_value / 4.0 - wheel_4_value / 4.0);
        
        float rot = r * (-l * wheel_2_value + l * wheel_3_value - l * wheel_1_value + l * wheel_4_value);

        #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = this->model->WorldPose();
        #else
        ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();
        #endif

        float yaw = pose.Rot().Yaw();

        float x_a = x * cosf(yaw) - y * sinf(yaw);
        float y_a = y * cosf(yaw) + x * sinf(yaw);

        this->model->SetLinearVel(ignition::math::Vector3d(x_a, y_a, 0));
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, rot));
        this->model->GetJoint("wheel_joint1")->SetVelocity(0, wheel_1_value);
        this->model->GetJoint("wheel_joint2")->SetVelocity(0, wheel_2_value);
        this->model->GetJoint("wheel_joint3")->SetVelocity(0, wheel_3_value);
        this->model->GetJoint("wheel_joint4")->SetVelocity(0, wheel_4_value);

        zm_robot_pos.x  = pose.Pos().X();
        zm_robot_pos.y  = pose.Pos().Y();
        zm_robot_pos.theta = pose.Rot().Yaw();
        
        joint_states_.header.stamp = current_time;
        last_time = current_time;
        joint_pub.publish(joint_states_);
        pose_pub.publish(zm_robot_pos);
        }

        public: void wheel1(const std_msgs::Float64::ConstPtr& msg)
        {
          wheel_1_value = msg->data;
        }
        public: void wheel2(const std_msgs::Float64::ConstPtr& msg)
        {
          wheel_2_value = msg->data;
        }
        public: void wheel3(const std_msgs::Float64::ConstPtr& msg)
        {
          wheel_3_value = msg->data;
        }
        public: void wheel4(const std_msgs::Float64::ConstPtr& msg)
        {
          wheel_4_value = msg->data;
        }

      private:
        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        boost::shared_ptr<ros::NodeHandle> mRosnode;

        double wheel_1_value;
        double wheel_2_value;
        double wheel_3_value;
        double wheel_4_value;

        ros::Subscriber mfl_sub;
        ros::Subscriber mfr_sub;
        ros::Subscriber mbl_sub;
        ros::Subscriber mbr_sub;

        ros::Publisher joint_pub;

        ros::Publisher pose_pub;

        sensor_msgs::JointState joint_states_;

        geometry_msgs::Pose2D zm_robot_pos;

        double joint_1_position;
        double joint_2_position;
        double joint_3_position;
        double joint_4_position;

        double wheel_1_num;
        double wheel_2_num;
        double wheel_3_num;
        double wheel_4_num;

        ros::Time current_time;
        ros::Time last_time;
      };

      // Register this plugin with the simulator
      GZ_REGISTER_MODEL_PLUGIN(Mecanum)
    }
