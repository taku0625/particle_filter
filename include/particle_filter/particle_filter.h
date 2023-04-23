#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle& nh);
    ~ParticleFilter();

protected:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber tf_sub_;
};