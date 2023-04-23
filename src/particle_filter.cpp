#include "particle_filter/particle_filter.h"
#include <random>
#include <bits/stdc++.h>

ParticleFilter::ParticleFilter(ros::NodeHandle& nh, const int particle_nums)
    : nh_(nh)
{
    initializeParticles(particle_nums);

    odom_sub_ = nh_.subscribe("/odom", 1, &ParticleFilter::odomCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &ParticleFilter::scanCallback, this);
    tf_sub_ = nh_.subscribe("/tf", 1, &ParticleFilter::tfCallback, this);
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::initializeParticles(const int particle_nums)
{
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<double> dist_1(0.0, 1.0);
    std::uniform_real_distribution<double> dist_2(-M_PI, M_PI);

    for(int i=0; i < particle_nums; ++i)
    {
        particles_[i][0] = dist_1(engine);
        particles_[i][1] = dist_1(engine);
        particles_[i][2] = dist_2(engine);
    }
}

void ParticleFilter::updateParticlesByOperatingModel(double delta_linear, double delta_angular)
{
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<double> dist_1(0.0, 1.0);
    std::normal_distribution<double> dist_1(0.0, 1.0);

    for(int i=0; i < particles_.size(); ++i)
    {
        particles_[i][0] += delta_linear[0] + ;
        particles_[i][1] = dist_1(engine);
        particles_[i][2] = dist_2(engine);
    }
};

void ParticleFilter::calculateLikelihood()
{
};

void ParticleFilter::estimatePose()
{
};

void ParticleFilter::resampleParticles()
{
};

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double linear_vel = msg->twist.twist.linear.x;
    double angular_vel = msg->twist.twist.angular.z;
    double delta_linear = linear_vel / ODOM_HZ_;
    double delta_angular = angular_vel / ODOM_HZ_;

    
    
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("%f", msg->scan_time);
}

void ParticleFilter::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}