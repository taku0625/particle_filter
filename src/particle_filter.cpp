#include "particle_filter/particle_filter.h"
#include <random>
#include <bits/stdc++.h>

using namespace std;

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

    for(size_t i = 0; i < particle_nums; ++i)
    {
        particles_[i][0] = dist_1(engine);
        particles_[i][1] = dist_1(engine);
        particles_[i][2] = dist_2(engine);
    }
}

void ParticleFilter::updateParticlesByOperatingModel(const double delta_linear, const double delta_angular)
{
    double linear_noise_std = std::sqrt(
        ODOM_NOISES_[0] * std::pow(delta_linear, 2.0) + ODOM_NOISES_[1] * std::pow(delta_angular, 2.0)
    );
    double angular_noise_std = std::sqrt(
        ODOM_NOISES_[2] * std::pow(delta_linear, 2.0) + ODOM_NOISES_[3] * std::pow(delta_angular, 2.0)
    );

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<double> delta_linear_with_noise(delta_linear, linear_noise_std);
    std::normal_distribution<double> delta_angular_with_noise(delta_angular, angular_noise_std);

    for(size_t i = 0; i < particles_.size(); ++i)
    {
        particles_[i][0] += delta_linear_with_noise(engine) * std::cos(particles_[i][2]);
        particles_[i][1] += delta_linear_with_noise(engine) * std::sin(particles_[i][2]);
        particles_[i][2] += delta_angular_with_noise(engine);
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

std::vector<int> ParticleFilter::pMax(const std::vector<double>& laser_ranges)
{
    std::vector<int> p(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); ++i)
    {
        p[i] = laser_ranges[i] == MAX_LASER_RANGE_ ? 1 : 0;
    }
    return p;
}

std::vector<double> ParticleFilter::pRand(const std::vector<double>& laser_ranges)
{
    std::vector<double> p(laser_ranges.size());
    std::fill(p.begin(), p.end(), 1 / MAX_LASER_RANGE_);
    return p;
}

std::vector<double> ParticleFilter::pHit(const std::vector<double>& laser_ranges)
{
    std::vector<double> p(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); ++i)
    {
        double laser_angle = pose_.angule_z + SCAN_ANGLE_MIN_ + (double)i * SCAN_ANGLE_INCREMENT_;
        double laser_position_x = pose_.position_x + laser_ranges[i] * std::cos(laser_angle);
        double laser_position_y = pose_.position_y + laser_ranges[i] * std::sin(laser_angle);
    }
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double linear_vel = msg->twist.twist.linear.x;
    double angular_vel = msg->twist.twist.angular.z;
    double delta_linear = linear_vel / ODOM_HZ_;
    double delta_angular = angular_vel / ODOM_HZ_;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int laser_nums = msg->ranges.size();
    std::copy(msg->ranges.begin(), msg->ranges.end(), laser_ranges_);
}

void ParticleFilter::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}