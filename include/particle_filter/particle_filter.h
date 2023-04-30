#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>
#include "particle_filter/utils.h"

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle& nh);
    ~ParticleFilter();

protected:
    void mclLoop();

    void readMap(std::string yaml_file_path, std::string img_file_path);
    void initializeParticles();

    void updateParticlesByOperatingModel(const double delta_linear, const double delta_angular);
    std::vector<double> calculateLikelihoods(const std::vector<double>& laser_ranges);
    void estimatePose(const std::vector<double>& particle_weights);
    void resampleParticles(const std::vector<double>& particle_weights);

    double calulateLogLikelihoodPerParticle(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges);
    std::vector<double> pMax(const std::vector<double>& laser_ranges);
    double pRand();
    std::vector<double> pHit(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges);

    geometry_msgs::PoseArray::Ptr createPoseArrayOfParticles();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;

    ros::Publisher pub_particles_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;

    tf2_ros::TransformBroadcaster br_;

    geometry_msgs::Pose2D::Ptr estimate_pose_;
    std::vector<geometry_msgs::Pose2D::Ptr> particles_;
    std::vector<double> laser_ranges_;

    double delta_linear_;
    double delta_angular_;

    double map_resolution_;
    std::vector<double> map_origin_;
    int map_width_;
    int map_height_;
    cv::Mat distance_field_;

    const int PARTICLE_NUM_ = 100;
    const double ODOM_HZ_ = 50;
    const double ODOM_NOISES_[4] = {4.0, 1.0, 1.0, 4.0};
    const double SCAN_ANGLE_MIN_ = -2.356194496154785;
    const double SCAN_ANGLE_MAX_ = 2.356194496154785;
    const double SCAN_ANGLE_INCREMENT_ = 0.004363323096185923;
    const double SCAN_RANGE_MAX_ = 30.0;
    const double MAX_LASER_RANGE_ = 30.0;
    const int SCAN_STEP_ = 5;
    const double LFM_VAR_ = 0.01;
    const double Z_MAX_ = 0.05;
    const double Z_RAND_ = 0.05;
    const double Z_HIT_ = 0.9;
    const double RESAMPLE_THRESHOLD_ = 0.5;
    const double X_LIDER_ = 0.5;
    const std::string FRAME_ID_ = "/map";
    const std::string CHILD_FRAME_ID_ = "/estimate_pose";
};

#endif