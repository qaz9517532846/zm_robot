#include "zm_safety/zm_safety.h"

#define PI 3.141592653

ZMSafety::ZMSafety() : nh_("~"), 
                       stop_bumper_(false), 
					   total_slow(false),
                       total_stop(false),
					   slow_range_l(0.70),
                       slow_range_w(0.30),
					   stop_range_l(0.40),
                       stop_range_w(0.25),
					   init_vel_x(0),
					   init_vel_y(0),
					   init_vel_th(0),
                       node_loop_rate_ (20)
{
	nh_.param<int>("scan_num", scan_num_, 2);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	slow_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("slow_range_marker", 10);
	stop_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("stop_range_marker", 10);

	cmd_vel_sub_ = nh_.subscribe("/zm_cmd_vel", 1, &ZMSafety::CmdVelCallback, this);
	bumper_sub_ = nh_.subscribe("/bumper", 1, &ZMSafety::bumperCallback, this);

	scan_sub_.resize(scan_num_);
	stop_laser_.resize(scan_num_);
	slow_laser_.resize(scan_num_);
	for(int i = 0; i < scan_num_; i++)
	{
		std::string ScanTopicName;
		ScanTopicName = "/scan_" + std::to_string(i);
		scan_sub_[i] = nh_.subscribe<sensor_msgs::LaserScan>(ScanTopicName.c_str(), 10, boost::bind(&ZMSafety::scanCallback, this, _1, i));
	}

	dynamic_reconfigure::Server<zm_robot_safety::zm_RobotSafetyConfig>::CallbackType f;
    f = boost::bind(&ZMSafety::reconfig_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

	buildRectangleVizMsgs();
}

ZMSafety::~ZMSafety()
{
    cmd_vel_pub_.shutdown();
	cmd_vel_sub_.shutdown();
	bumper_sub_.shutdown();
	for(int i = 0; i < scan_num_; i++)
	{
		scan_sub_[i].shutdown();
	}
}

void ZMSafety::spin()
{
	ros::Rate lr(node_loop_rate_);
	while(nh_.ok())
	{
		ros::spinOnce();
		cmd_vel_msg_.linear.x = unlock_safe_ == true ? init_vel_x : dist_* init_vel_x;
		cmd_vel_msg_.linear.y = unlock_safe_ == true ? init_vel_y : dist_ * init_vel_y;
		cmd_vel_msg_.angular.z = unlock_safe_ == true ? init_vel_th : dist_ * init_vel_th;
		cmd_vel_pub_.publish(cmd_vel_msg_);
		lr.sleep();
	}
}

void ZMSafety::reconfig_callback(zm_robot_safety::zm_RobotSafetyConfig &config, uint32_t level)
{
	unlock_safe_ = config.unlock_safe;
	slow_range_l = config.slow_range_l;
	slow_range_w = config.slow_range_w;
	stop_range_l = config.stop_range_l;
	stop_range_w = config.stop_range_w;
	node_loop_rate_ = config.node_loop_rate;
}

void ZMSafety::buildRectangleVizMsgs()
{
	stop_viz_msg_.header.frame_id = slow_viz_msg_.header.frame_id = "base_link";
	stop_viz_msg_.header.stamp = slow_viz_msg_.header.stamp = ros::Time::now();
	slow_viz_msg_.id = 1;
	stop_viz_msg_.id = 2;
	stop_viz_msg_.ns = "stop_range";
	slow_viz_msg_.ns = "slow_range";
	stop_viz_msg_.action = slow_viz_msg_.action = visualization_msgs::Marker::ADD;
	stop_viz_msg_.type = slow_viz_msg_.type = visualization_msgs::Marker::LINE_STRIP;

	// Color the ellipses green
	stop_viz_msg_.color.g = 1.0;
	stop_viz_msg_.color.a = 1.0;
	slow_viz_msg_.color.g = 1.0;
	slow_viz_msg_.color.a = 1.0;

	// Scale the points
	stop_viz_msg_.scale.x = 0.01;
	slow_viz_msg_.scale.x = 0.01;

	// Now we populate the msgs
	stop_viz_msg_.points = draw_range(stop_range_w, stop_range_l);
	slow_viz_msg_.points = draw_range(slow_range_w, slow_range_l);
}

std::vector<geometry_msgs::Point> ZMSafety::draw_range(double rang_w, double rang_l)
{
	std::vector<geometry_msgs::Point> range_result;

	geometry_msgs::Point draw_point;
	draw_point.x = rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = rang_l / 2;
	draw_point.y = -rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = -rang_l / 2;
	draw_point.y = -rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = -rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	return range_result;
}

void ZMSafety::CmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	init_vel_x = msg->linear.x;
	init_vel_y = msg->linear.y;
	init_vel_th = msg->angular.z;
}

void ZMSafety::bumperCallback(const std_msgs::BoolConstPtr& msg)
{
	if(msg->data)
	{
		ROS_ERROR("Bumper hit! Shutting down node!");
		ros::shutdown();
		return;
	}
}

void ZMSafety::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx)
{
	sensor_msgs::PointCloud cloud;

	try
	{
		tfListener_.waitForTransform("/base_link", msg->header.frame_id,
                                                   msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                                                   ros::Duration(1.0));
		projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, tfListener_);
	}
	catch(tf::LookupException& ex)
	{
		ROS_WARN("Lookup exception: %s\n", ex.what());
		return;
	}
	catch(tf::ConnectivityException& ex)
	{
		ROS_WARN("Connectivity exception: %s\n", ex.what());
		return;
	}
	catch(tf::ExtrapolationException& ex)
	{
		ROS_WARN("Extrapolation exception: %s\n", ex.what());
		return;
	}
	check(cloud, idx);
	visualizeRectangles();
}

void ZMSafety::check(sensor_msgs::PointCloud cloud, int idx)
{
	stop_laser_[idx] = false;
	slow_laser_[idx] = false;
	dist_ = 1.0;

	for(unsigned int i=0; i < cloud.points.size(); ++i)
	{
		inE2(cloud.points[i], idx);
	}
}

void ZMSafety::inE2(geometry_msgs::Point32 point, int idx)
{
	bool check_slow = (2 * abs(point.x) < slow_range_l) && (2 * abs(point.y) < slow_range_w);
	bool check_stop = (2 * abs(point.x) < stop_range_l) && (2 * abs(point.y) < stop_range_w);
	total_slow = total_stop = false;

	if(check_slow && !check_stop)
	{
		slow_laser_[idx] = true;
	}
	else if(check_slow && check_stop)
	{
		stop_laser_[idx] = true;
	}

	for(int i = 0; i < scan_num_; i++)
	{
		if(slow_laser_[i]) total_slow = true;
		if(stop_laser_[i]) total_stop = true;
	}

	if(total_slow && !total_stop)
	{
		dist_ = solveE1(point) * 0.1;
	}
	else if(total_slow && total_stop)
	{
		dist_ = 0.0;
	}
}

double ZMSafety::solveE1(geometry_msgs::Point32 point)
{
	double dis_vel;
	if((abs(point.x) / slow_range_l) >= (abs(point.y) / slow_range_w))
	{
		dis_vel = abs(point.y) / slow_range_w;
	}
	else
	{
		dis_vel = abs(point.x) / slow_range_l;
	}

	return dis_vel;
}

void ZMSafety::visualizeRectangles(bool show)
{
	stop_viz_msg_.header.stamp = slow_viz_msg_.header.stamp = ros::Time::now();

	// Color the ellipses green
	stop_viz_msg_.color.r = 0.0;
	stop_viz_msg_.color.g = 1.0;
	stop_viz_msg_.color.b = 0.0;

	slow_viz_msg_.color.r = 0.0;
	slow_viz_msg_.color.g = 1.0;
	slow_viz_msg_.color.b = 0.0;

	if(total_stop)
	{
		// Color the ellipse e1 red
		stop_viz_msg_.color.r = 1.0;
		stop_viz_msg_.color.g = 0.0;
		stop_viz_msg_.color.b = 0.0;
	}
	if (total_slow)
	{
		// Color the ellipse e2 red
		slow_viz_msg_.color.r = 1.0;
		slow_viz_msg_.color.g = 0.0;
		slow_viz_msg_.color.b = 0.0;
	}

	stop_viz_pub_.publish(stop_viz_msg_);
	slow_viz_pub_.publish(slow_viz_msg_);
}