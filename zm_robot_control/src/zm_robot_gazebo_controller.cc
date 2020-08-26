#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float32.h>
#include <iostream>

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
      this->model = _parent;

      wheel_1_value = 0;
      wheel_2_value = 0;
      wheel_3_value = 0;
      wheel_4_value = 0;

      mRosnode.reset(new ros::NodeHandle(""));

      mfl_sub = mRosnode->subscribe("/zm_robot/joint1_velocity_controller/state", 1000, &Mecanum::wheel1, this);
      mfr_sub = mRosnode->subscribe("/zm_robot/joint2_velocity_controller/state", 1000, &Mecanum::wheel2, this);
      mbl_sub = mRosnode->subscribe("/zm_robot/joint3_velocity_controller/state", 1000, &Mecanum::wheel3, this);
      mbr_sub = mRosnode->subscribe("/zm_robot/joint4_velocity_controller/state", 1000, &Mecanum::wheel4, this);


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Mecanum::UpdateChild, this, _1));
      }

      public: void UpdateChild(const common::UpdateInfo & /*_info*/)
      {
        float r = WHEEL_RAD;
        float L = TRACK; // left -> right
        float W = WHEELBASE; // front -> back
        float l = 1.0 / (2 * (W + L));
        float x = r * ( wheel_2_value / 4.0 + wheel_3_value / 4.0 + wheel_1_value / 4.0 + wheel_4_value / 4.0);
        float y = r * ( -wheel_2_value / 4.0 + wheel_3_value / 4.0 + wheel_1_value / 4.0 - wheel_4_value / 4.0);
        
        float rot = r * (-l * wheel_2_value + l * wheel_3_value - l * wheel_1_value + l * wheel_4_value);

        math::Pose pose = this->model->GetWorldPose();

        float yaw = pose.rot.GetYaw();

        this->model->SetLinearVel(math::Vector3(x * cosf(yaw) - y * sinf(yaw), y * cosf(yaw) + x * sinf(yaw), 0));
        this->model->SetAngularVel(math::Vector3(0, 0, rot));

        }

        public: void wheel1(const control_msgs::JointControllerState::ConstPtr& msg)
        {
          wheel_1_value = msg->process_value;
        }
        public: void wheel2(const control_msgs::JointControllerState::ConstPtr& msg)
        {
          wheel_2_value = msg->process_value;
        }
        public: void wheel3(const control_msgs::JointControllerState::ConstPtr& msg)
        {
          wheel_3_value = msg->process_value;
        }
        public: void wheel4(const control_msgs::JointControllerState::ConstPtr& msg)
        {
          wheel_4_value = msg->process_value;
        }

      private:
        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        boost::shared_ptr<ros::NodeHandle> mRosnode;

        float wheel_1_value;
        float wheel_2_value;
        float wheel_3_value;
        float wheel_4_value;

        ros::Subscriber mfl_sub;
        ros::Subscriber mfr_sub;
        ros::Subscriber mbl_sub;
        ros::Subscriber mbr_sub;
      };

      // Register this plugin with the simulator
      GZ_REGISTER_MODEL_PLUGIN(Mecanum)
    }
