#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle& nh, const int particle_num=100);
    ~ParticleFilter();

protected:
    void initializeParticles(const int particle_num);

    void updateParticlesByOperatingModel(const double delta_linear, const double delta_angular);
    void calculateLikelihood();
    void estimatePose();
    void resampleParticles();

    void calculateLikelihoodField();
    std::vector<int> pMax(const std::vector<double>& laser_ranges);
    std::vector<double> pRand(const std::vector<double>& laser_ranges);
    std::vector<double> pHit(const std::vector<double>& laser_ranges);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber tf_sub_;

    struct Pose
    {
        double position_x;
        double position_y;
        double angule_z;
    };
    Pose pose_ = {0.0, 0.0, 0.0};

    std::vector<std::vector<double>> particles_;
    std::vector<std::vector<double>> laser_ranges_;

    const double ODOM_HZ_ = 50;
    const double ODOM_NOISES_[4] = {4.0, 1.0, 1.0, 4.0};
    const double SCAN_ANGLE_MIN_ = -2.356194496154785;
    const double SCAN_ANGLE_MAX_ = 2.356194496154785;
    const double SCAN_ANGLE_INCREMENT_ = 0.004363323096185923;
    const double SCAN_RANGE_MAX_ = 30.0;
    const double MAX_LASER_RANGE_ = 30.0;
    const double LFM_VAR_ = 0.01;
};

#endif