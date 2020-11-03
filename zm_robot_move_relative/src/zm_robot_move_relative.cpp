#include <zm_robot_move_relative/zm_move_relative.h>

zm_robot_MoveRelative::zm_robot_MoveRelative(std::string name) : action_server_(nh_, 
                                                                                name,
                                                                                boost::bind(&zm_robot_MoveRelative::execute, this, _1),
                                                                                false)
{
    ros::NodeHandle private_nh;
    
    goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
    vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    dynamic_reconfigure::Server<zm_robot_move_relative::zm_robot_MoveRelativeConfig>::CallbackType f;

    f = boost::bind(&zm_robot_MoveRelative::reconfig_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

    tf_listener_ = new tf2_ros::TransformListener(tf_Buffer_);

    action_server_.start();
}

zm_robot_MoveRelative::~zm_robot_MoveRelative()
{

}

void zm_robot_MoveRelative::reconfig_callback(zm_robot_move_relative::zm_robot_MoveRelativeConfig &config, uint32_t level)
{
    rate_ = config.rate;
    timeout_ = config.timeout;
    base_frame_ = config.base_frame;
    fixed_frame_ = config.fixed_frame;
    x_.max_vel = config.max_vel_x;
    x_.min_vel = config.min_vel_x;
    y_.max_vel = config.max_vel_y;
    y_.min_vel = config.min_vel_y;
    theta_.max_vel = config.max_vel_theta;
    theta_.min_vel = config.min_vel_theta;
    x_.acceleration = config.acceleration_x;
    y_.acceleration = config.acceleration_y;
    theta_.acceleration = config.acceleration_theta;
    linear_tolerance_sq_ = config.linear_tolerance * config.linear_tolerance;
    angular_tolerance_ = config.angular_tolerance;
}

void zm_robot_MoveRelative::execute(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    ROS_INFO("Recieved goal");
    move_base_msgs::MoveBaseResult result;
    move_base_msgs::MoveBaseFeedback feedback;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::TransformStamped move_relative_fixed_transform;

    tf2::Stamped<tf2::Transform> move_relative_fixed_tf2;

    ros::Rate r(rate_);
    ros::Duration timeout = ros::Duration(timeout_);

    std::string move_relative_frame_ = goal->target_pose.header.frame_id;
    ROS_INFO("Move_relative_frame = %s", move_relative_frame_.c_str());

    goal_pub_.publish(goal->target_pose);

    ros::Time now = ros::Time::now();

    try
    {
      tf_Buffer_.transform(goal->target_pose, goal_pose, move_relative_frame_); //goal->target_pose relative to move_relative_frame_ convert goal_pose
      
      // lookupTransform (parent, child, ...)
      move_relative_fixed_transform = tf_Buffer_.lookupTransform(fixed_frame_, move_relative_frame_, now, timeout);
      tf2::convert(move_relative_fixed_transform, move_relative_fixed_tf2); // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Abort, goal invalid. " << ex.what());
      action_server_.setAborted(result, ex.what());
      return;
    }

    while(ros::ok())
    {
      if (action_server_.isPreemptRequested())
      {
        stop_vel();
        ROS_WARN_STREAM("Action canceled.");
        action_server_.setPreempted(result, "Action canceled");
        return;
      }

      geometry_msgs::TransformStamped fixed_base_transform;

      try
      {
        now = ros::Time::now();
        // lookupTransform (parent, child, ...)
        fixed_base_transform = tf_Buffer_.lookupTransform(base_frame_, fixed_frame_, now, timeout);
      }
      catch (tf2::TransformException &ex)
      {
        stop_vel();
        ROS_ERROR_STREAM("Abort. " << ex.what());
        action_server_.setAborted(result, ex.what());
        return;
      }

      tf2::Stamped<tf2::Transform> goal_tf2;
      tf2::Stamped<tf2::Transform> fixed_base_tf2;

      // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(goal_pose, goal_tf2);
      tf2::convert(fixed_base_transform, fixed_base_tf2);

      tf2::Transform diff_tf2;

      diff_tf2 = fixed_base_tf2 * move_relative_fixed_tf2 * goal_tf2;

      double diff_x = diff_tf2.getOrigin().x();
      double diff_y = diff_tf2.getOrigin().y();
      double diff_yaw = tf2::getYaw(diff_tf2.getRotation());
      if (diff_x * diff_x + diff_y * diff_y <= linear_tolerance_sq_ && fabs(diff_yaw) <= angular_tolerance_)
      {
        ROS_INFO_STREAM("diff_tf2.getOrigin().x(): " << diff_tf2.getOrigin().x() << " diff_tf2.getOrigin().y(): " << diff_tf2.getOrigin().y() );
        stop_vel();
        ROS_INFO_STREAM("Success");
        action_server_.setSucceeded(result, "Success");
        return;
      }

      feedback.base_position.header =  goal->target_pose.header;
      feedback.base_position.header.frame_id =  base_frame_;
      feedback.base_position.pose.position.x = diff_x;
      feedback.base_position.pose.position.x = diff_y;
      geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(diff_yaw);
      feedback.base_position.pose.orientation = yaw_quat;

      // tf2::toMsg(fixed_reference_tf2, feedback.base_position); // this works
      // tf2::convert(fixed_reference_tf2, feedback.base_position); // this does not work, why?
      action_server_.publishFeedback(feedback);

      geometry_msgs::Twist pub_vel;
      pub_vel.linear.x = cal_vel(diff_x, x_);
      pub_vel.linear.y = cal_vel(diff_y, y_);
      pub_vel.angular.z = cal_vel(diff_yaw, theta_);
      vel_pub_.publish(pub_vel);

      r.sleep();
    }
}

void zm_robot_MoveRelative::stop_vel()
{
    geometry_msgs::Twist pub_vel;
    vel_pub_.publish(pub_vel);
}

double zm_robot_MoveRelative::cal_vel(double difference, velocity_setting &set)
{
  double velocity = 0.0;
  int sign = 1;
  if (difference == 0.0)
  {
    return 0;
  }
  else if (difference < 0.0)
  {
    sign = -1;
  }
  velocity = sign * sqrt(fabs(difference) * 2 * set.acceleration);
  if (fabs(velocity) > set.max_vel)
  {
    velocity = sign * set.max_vel;
  }
  else if (fabs(velocity) < set.min_vel)
  {
    velocity = sign * set.min_vel;
  }
  return velocity;
}