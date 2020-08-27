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

      mfl_sub = mRosnode->subscribe("/wheel2_velocity", 1000, &Mecanum::wheel2, this);
      mfr_sub = mRosnode->subscribe("/wheel3_velocity", 1000, &Mecanum::wheel3, this);
      mbl_sub = mRosnode->subscribe("/wheel1_velocity", 1000, &Mecanum::wheel1, this);
      mbr_sub = mRosnode->subscribe("/wheel4_velocity", 1000, &Mecanum::wheel4, this);


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
