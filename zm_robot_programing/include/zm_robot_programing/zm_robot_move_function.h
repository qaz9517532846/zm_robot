#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>

class zm_robot {
  public:
    zm_robot(std::string name);
    ~zm_robot();
    void move_map(double x, double y, double theta);
    void move_base(double x, double y, double theta);

  private:
    void spinThread();
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    move_base_msgs::MoveBaseGoal goal;
};
