#include "particle_filter/particle_filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle& nh)
    : nh_(nh)
{
    odom_sub_ = nh_.subscribe("/odom", 1, &ParticleFilter::odomCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &ParticleFilter::scanCallback, this);
    tf_sub_ = nh_.subscribe("/tf", 1, &ParticleFilter::tfCallback, this);
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("%f", msg->pose.pose.position.x);
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("%f", msg->scan_time);
}

void ParticleFilter::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}