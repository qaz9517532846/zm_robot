#include <zm_robot_pid_control/zm_robot_pid.h>

// These need to be pulled out to parameters...
const float WHEEL_RAD = 0.065; // meters
const float WHEELBASE = 0.6; // meters
const float TRACK = 0.5; // meters

namespace gazebo
{

void Mecanum::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
 
    this->model = _parent;
    this->sdf = _sdf;

    this->joint_1 = _parent->GetJoints()[0];
    this->joint_2 = _parent->GetJoints()[1];
    this->joint_3 = _parent->GetJoints()[2];
    this->joint_4 = _parent->GetJoints()[3];
      
    mRosnode.reset(new ros::NodeHandle(""));

    mfl_sub = mRosnode->subscribe("/wheel2_velocity", 1000, &Mecanum::wheel2, this);
    mfr_sub = mRosnode->subscribe("/wheel3_velocity", 1000, &Mecanum::wheel3, this);
    mbl_sub = mRosnode->subscribe("/wheel1_velocity", 1000, &Mecanum::wheel1, this);
    mbr_sub = mRosnode->subscribe("/wheel4_velocity", 1000, &Mecanum::wheel4, this);

    joint_pub = mRosnode->advertise<sensor_msgs::JointState>("joint_states", 1000);

    pose_pub = mRosnode->advertise<zm_robot_pid_control::agv_pid_position>("zm_robot_position", 1000);

    joint_states_.name.push_back("wheel_joint1");
    joint_states_.name.push_back("wheel_joint2");
    joint_states_.name.push_back("wheel_joint3");
    joint_states_.name.push_back("wheel_joint4");
    joint_states_.position.resize(4, 0.0);
    joint_states_.velocity.resize(4, 0.0);
    joint_states_.effort.resize(4, 0.0);

    wheel1_vel = mRosnode->advertise<std_msgs::Float64>("wheel1_output_vel", 1000);
    wheel2_vel = mRosnode->advertise<std_msgs::Float64>("wheel2_output_vel", 1000);
    wheel3_vel = mRosnode->advertise<std_msgs::Float64>("wheel3_output_vel", 1000);
    wheel4_vel = mRosnode->advertise<std_msgs::Float64>("wheel4_output_vel", 1000);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Mecanum::UpdateChild, this, _1));
    }

void Mecanum::UpdateChild(const common::UpdateInfo & /*_info*/)
{
    current_time = ros::Time::now(); 
    double dt = (current_time - last_time).toSec();

    wheel1_p = 10.0; wheel1_i = 0.0; wheel1_d = 0.0;
    wheel2_p = 10.0; wheel2_i = 0.0; wheel2_d = 0.0;
    wheel3_p = 10.0; wheel3_i = 0.0; wheel3_d = 0.0;
    wheel4_p = 10.0; wheel4_i = 0.0; wheel4_d = 0.0;

    this->wheel1_pid = common::PID(wheel1_p, wheel1_i, wheel1_d);
    this->wheel2_pid = common::PID(wheel2_p, wheel2_i, wheel2_d);
    this->wheel3_pid = common::PID(wheel3_p, wheel3_i, wheel3_d);
    this->wheel4_pid = common::PID(wheel4_p, wheel4_i, wheel4_d);

    this->model->GetJointController()->SetVelocityPID(this->joint_1->GetScopedName(), this->wheel1_pid);
    this->model->GetJointController()->SetVelocityPID(this->joint_2->GetScopedName(), this->wheel2_pid);
    this->model->GetJointController()->SetVelocityPID(this->joint_3->GetScopedName(), this->wheel3_pid);
    this->model->GetJointController()->SetVelocityPID(this->joint_4->GetScopedName(), this->wheel4_pid);

    this->model->GetJointController()->SetVelocityTarget(this->joint_1->GetScopedName(), wheel_1_value);
    this->model->GetJointController()->SetVelocityTarget(this->joint_2->GetScopedName(), wheel_2_value);
    this->model->GetJointController()->SetVelocityTarget(this->joint_3->GetScopedName(), wheel_3_value);
    this->model->GetJointController()->SetVelocityTarget(this->joint_4->GetScopedName(), wheel_4_value);

    wheel1_output = this->joint_1->GetVelocity(0);
    wheel2_output = this->joint_2->GetVelocity(0);
    wheel3_output = this->joint_3->GetVelocity(0);
    wheel4_output = this->joint_4->GetVelocity(0);

    joint_1_position = joint_1_position + wheel1_output * dt ;
    joint_2_position = joint_2_position + wheel2_output * dt ;
    joint_3_position = joint_3_position + wheel3_output * dt ;
    joint_4_position = joint_4_position + wheel4_output * dt ;

    joint_states_.position[0] = joint_1_position;
    joint_states_.position[1] = joint_2_position;
    joint_states_.position[2] = joint_3_position;
    joint_states_.position[3] = joint_4_position;
    joint_states_.velocity[0] = wheel1_output;
    joint_states_.velocity[1] = wheel2_output;
    joint_states_.velocity[2] = wheel3_output;
    joint_states_.velocity[3] = wheel4_output;

    float r = WHEEL_RAD;
    float L = TRACK; // left -> right
    float W = WHEELBASE; // front -> back
    float l = 1.0 / (2 * (W + L));
    float x = r * ( wheel2_output / 4.0 + wheel3_output / 4.0 + wheel1_output / 4.0 + wheel4_output / 4.0);
    float y = r * ( -wheel2_output / 4.0 + wheel3_output / 4.0 + wheel1_output / 4.0 - wheel4_output / 4.0);
        
    float rot = r * (-l * wheel2_output + l * wheel3_output - l * wheel1_output + l * wheel4_output);

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

    zm_robot_pos.position_x  = pose.Pos().X();
    zm_robot_pos.position_y  = pose.Pos().Y();
    zm_robot_pos.position_th = pose.Rot().Yaw();

    wheel_1_output.data = wheel1_output;
    wheel_2_output.data = wheel2_output;
    wheel_3_output.data = wheel3_output;
    wheel_4_output.data = wheel4_output;
        
    joint_states_.header.stamp = current_time;
    last_time = current_time;
    wheel1_vel.publish(wheel_1_output);
    wheel2_vel.publish(wheel_2_output);
    wheel3_vel.publish(wheel_3_output);
    wheel4_vel.publish(wheel_4_output);
    joint_pub.publish(joint_states_);
    pose_pub.publish(zm_robot_pos);
}

void Mecanum::wheel1(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_1_value = msg->data;
}

void Mecanum::wheel2(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_2_value = msg->data;
}

void Mecanum::wheel3(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_3_value = msg->data;
}

void Mecanum::wheel4(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_4_value = msg->data;
}

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Mecanum)
}
