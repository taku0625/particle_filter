#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>
#include "particle_filter/likelihood_field_model.h"
#include "particle_filter/utils.h"

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle& nh);
    ~ParticleFilter();

protected:
    void mclLoop();

    void initializeParticles();

    void updateParticlesByOperatingModel(const double delta_linear, const double delta_angular);
    std::vector<double> calculateLikelihoods(const std::vector<double>& laser_ranges);
    void estimatePose(const std::vector<double>& particle_weights);
    void resampleParticles(const std::vector<double>& particle_weights);

    geometry_msgs::PoseArray::Ptr createPoseArrayOfParticles();
    geometry_msgs::TransformStamped::Ptr createTransformStampedOfOdomOnMap();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;

    ros::Publisher pub_particles_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::Pose2D::Ptr estimate_pose_;
    std::vector<geometry_msgs::Pose2D::Ptr> particles_;
    std::vector<double> laser_ranges_;

    std::unique_ptr<LikelihoodFieldModel> lfm_;

    double delta_linear_;
    double delta_angular_;

    const int PARTICLE_NUM_ = 100;
    const double ODOM_HZ_ = 50;
    const double ODOM_NOISES_[4] = {4.0, 1.0, 1.0, 4.0};
    const double RESAMPLE_THRESHOLD_ = 0.5;
    const double X_LIDER_ = 0.5;

    bool scan_flag_ = false;
};

#endif