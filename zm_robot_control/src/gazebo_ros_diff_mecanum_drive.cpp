#include <algorithm>
#include <assert.h>
 
#include <zm_robot_control/gazebo_ros_diff_mecanum_drive.h>
 
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
 
namespace gazebo
{
 
enum {
    RIGHT_FRONT,
    LEFT_FRONT,
    RIGHT_BACK,
    LEFT_BACK
};
 
GazeboRosDiffDrive::GazeboRosDiffDrive() {}
 
// Destructor
GazeboRosDiffDrive::~GazeboRosDiffDrive()
{
    FiniChild();
}
 
// Load the controller
void GazeboRosDiffDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
 
    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.6 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.13 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 50.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

    joints_.resize ( 4 );
    joints_[LEFT_FRONT] = gazebo_ros_->getJoint ( parent, "left_front_Joint", "left_front_joint" );
    joints_[RIGHT_FRONT] = gazebo_ros_->getJoint ( parent, "right_front_Joint", "right_front_joint" );
    joints_[LEFT_BACK] = gazebo_ros_->getJoint ( parent, "left_back_Joint", "left_back_joint" );
    joints_[RIGHT_BACK] = gazebo_ros_->getJoint ( parent, "right_back_Joint", "right_back_joint" );
    joints_[LEFT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_BACK]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_BACK]->SetParam ( "fmax", 0, wheel_torque );
 
    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN_NAMED("diff_drive", "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
 
    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_BACK] = 0;
    wheel_speed_[LEFT_BACK] = 0;
 
    // Initialize velocity support stuff
    wheel_speed_instr_[RIGHT_FRONT] = 0;
    wheel_speed_instr_[LEFT_FRONT] = 0;
    wheel_speed_instr_[RIGHT_BACK] = 0;
    wheel_speed_instr_[LEFT_BACK] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;
 
 
    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("diff_drive", "%s: Advertise joint_states", gazebo_ros_->info());
    }
 
    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
 
    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosDiffDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);
 
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("diff_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
 
    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO_NAMED("diff_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
    }
 
    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosDiffDrive::QueueThread, this ) );
 
    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosDiffDrive::UpdateChild, this ) );

}
 
void GazeboRosDiffDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  y_ = 0;
  rot_ = 0;
  joints_[LEFT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[LEFT_BACK]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT_BACK]->SetParam ( "fmax", 0, wheel_torque );
}
 
void GazeboRosDiffDrive::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();
 
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 4; i++ ) {
        physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = position;
    }
    joint_state_publisher_.publish ( joint_state_ );
} 
 
void GazeboRosDiffDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 4; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}
 
// Update the controller
void GazeboRosDiffDrive::UpdateChild()
{
    for ( int i = 0; i < 4; i++ ) {
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }
 
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[4];

        current_speed[LEFT_FRONT] = joints_[LEFT_FRONT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT_FRONT] = joints_[RIGHT_FRONT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[LEFT_BACK] = joints_[LEFT_BACK]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT_BACK] = joints_[RIGHT_BACK]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

        if ( wheel_accel == 0 ||
                ( fabs ( wheel_speed_[LEFT_FRONT] - current_speed[LEFT_FRONT] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[RIGHT_FRONT] - current_speed[RIGHT_FRONT] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[LEFT_BACK] - current_speed[LEFT_BACK] ) < 0.01 )     ||
                ( fabs ( wheel_speed_[RIGHT_BACK] - current_speed[RIGHT_BACK] ) < 0.01 ) ) {
           //if max_accel == 0, or target speed is reached
            joints_[LEFT_FRONT]->SetParam ( "vel", 0, wheel_speed_[LEFT_FRONT]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_FRONT]->SetParam ( "vel", 0, wheel_speed_[RIGHT_FRONT]/ ( wheel_diameter_ / 2.0 ) );
            joints_[LEFT_BACK]->SetParam ( "vel", 0, wheel_speed_[LEFT_BACK]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_BACK]->SetParam ( "vel", 0, wheel_speed_[RIGHT_BACK]/ ( wheel_diameter_ / 2.0 ) );
        } else {
            if ( wheel_speed_[LEFT_FRONT]>=current_speed[LEFT_FRONT] )
                wheel_speed_instr_[LEFT_FRONT]+=fmin ( wheel_speed_[LEFT_FRONT]-current_speed[LEFT_FRONT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT_FRONT]+=fmax ( wheel_speed_[LEFT_FRONT]-current_speed[LEFT_FRONT], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[RIGHT_FRONT]>current_speed[RIGHT_FRONT] )
                wheel_speed_instr_[RIGHT_FRONT]+=fmin ( wheel_speed_[RIGHT_FRONT]-current_speed[RIGHT_FRONT], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT_FRONT]+=fmax ( wheel_speed_[RIGHT_FRONT]-current_speed[RIGHT_FRONT], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[LEFT_BACK]>=current_speed[LEFT_BACK] )
                wheel_speed_instr_[LEFT_BACK]+=fmin ( wheel_speed_[LEFT_BACK]-current_speed[LEFT_BACK],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT_BACK]+=fmax ( wheel_speed_[LEFT_BACK]-current_speed[LEFT_BACK], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[RIGHT_BACK]>current_speed[RIGHT_BACK] )
                wheel_speed_instr_[RIGHT_BACK]+=fmin ( wheel_speed_[RIGHT_BACK]-current_speed[RIGHT_BACK], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT_BACK]+=fmax ( wheel_speed_[RIGHT_BACK]-current_speed[RIGHT_BACK], -wheel_accel * seconds_since_last_update );

            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);
 
            joints_[LEFT_FRONT]->SetParam ( "vel", 0, wheel_speed_instr_[LEFT_FRONT] / ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_FRONT]->SetParam ( "vel", 0, wheel_speed_instr_[RIGHT_FRONT] / ( wheel_diameter_ / 2.0 ) );
            joints_[LEFT_BACK]->SetParam ( "vel", 0, wheel_speed_instr_[LEFT_BACK] / ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_BACK]->SetParam ( "vel", 0, wheel_speed_instr_[RIGHT_BACK] / ( wheel_diameter_ / 2.0 ) );
        }
        last_update_time_+= common::Time ( update_period_ );
    }
}
 
// Finalize the controller
void GazeboRosDiffDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosDiffDrive::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double vr = x_;
    double va = rot_;

    wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_BACK] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_BACK] = vr + va * wheel_separation_ / 2.0;
}
 
void GazeboRosDiffDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosDiffDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}
 
void GazeboRosDiffDrive::UpdateOdometryEncoder()
{
    double vl_front = joints_[LEFT_FRONT]->GetVelocity ( 0 );
    double vr_front = joints_[RIGHT_FRONT]->GetVelocity ( 0 );
    double vl_back = joints_[LEFT_BACK]->GetVelocity ( 0 );
    double vr_back = joints_[RIGHT_BACK]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;
 
    double b = wheel_separation_;
 
    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl_front = vl_front * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr_front = vr_front * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sl_back = vl_back * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr_back = vr_back * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double ssum_x = sl_front + sr_front + sl_back + sr_back;
    double ssum_y = -sl_front + sr_front - sl_back + sr_back;
    double ssum_rot = -sl_front - sr_front + sl_back + sr_back;

    double dx = ( ssum_x ) / 4.0 * cos ( pose_encoder_.theta + ( ssum_rot ) / ( 2.0*b ) ) - ( ssum_y ) / 4.0 * sin ( pose_encoder_.theta + ( ssum_rot ) / ( 2.0*b ) );
    double dy = ( ssum_x ) / 4.0 * sin ( pose_encoder_.theta + ( ssum_rot ) / ( 2.0*b ) ) + ( ssum_y ) / 4.0 * cos ( pose_encoder_.theta + ( ssum_rot ) / ( 2.0*b ) );
    double dtheta = ( ssum_rot ) / 0.55 / 4;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = v;
    odom_.twist.twist.linear.y = 0;
}

void GazeboRosDiffDrive::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );
 
    tf::Quaternion qt;
    tf::Vector3 vt;
 
    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );
 
    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
        linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }

 
    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;
  
    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;
    odometry_publisher_.publish ( odom_ );
}
 
GZ_REGISTER_MODEL_PLUGIN ( GazeboRosDiffDrive )
}
