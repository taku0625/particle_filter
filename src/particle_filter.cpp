#include "particle_filter/particle_filter.h"
#include <random>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Point.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std;

ParticleFilter::ParticleFilter(ros::NodeHandle& nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    lfm_.reset(new LikelihoodFieldModel(
        "/home/takumiti/AutoNavi/ros/maps/nic_garage/ogm.yaml",
        "/home/takumiti/AutoNavi/ros/maps/nic_garage/ogm.pgm"
    ));
    estimate_pose_.reset(new geometry_msgs::Pose2D());
    initializeParticles();

    pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("/particles", 1);

    sub_odom_ = nh_.subscribe("/odom", 1, &ParticleFilter::odomCallback, this);
    sub_scan_ = nh_.subscribe("/scan", 1, &ParticleFilter::scanCallback, this);
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::mclLoop()
{
    // copy values so they are not updated during processing
    double delta_linear = delta_linear_;
    double delta_angular = delta_angular_;
    std::vector<double> laser_ranges = laser_ranges_;

    updateParticlesByOperatingModel(delta_linear, delta_angular);
    std::vector<double> likelihoods = calculateLikelihoods(laser_ranges);
    estimatePose(likelihoods);

    double total_squrares_of_likelihoods = std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0,
                                                           [](double a, double b){ return a + b * b; });
    double effective_sample_size = 1.0 / total_squrares_of_likelihoods;
    double threshold = (double)particles_.size() * RESAMPLE_THRESHOLD_;

    if(! (effective_sample_size > threshold))
    {
        resampleParticles(likelihoods);
    }
}

void ParticleFilter::initializeParticles()
{
    particles_.resize(PARTICLE_NUM_);

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<double> dist_1(0.0, 0.5);
    std::uniform_real_distribution<double> dist_2(0.0, 3.0 * M_PI / 180.0);

    for(size_t i = 0; i < static_cast<size_t>(PARTICLE_NUM_); ++i)
    {
        particles_[i].reset(new geometry_msgs::Pose2D());
        particles_[i]->x = dist_1(engine);
        particles_[i]->y = dist_1(engine);
        particles_[i]->theta = dist_2(engine);
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

    for(size_t i = 0; i < static_cast<size_t>(particles_.size()); ++i)
    {
        particles_[i]->x += delta_linear_with_noise(engine) * std::cos(particles_[i]->theta);
        particles_[i]->y += delta_linear_with_noise(engine) * std::sin(particles_[i]->theta);
        particles_[i]->theta += delta_angular_with_noise(engine);
    }
}

std::vector<double> ParticleFilter::calculateLikelihoods(const std::vector<double>& laser_ranges)
{
    std::vector<double> log_likelihoods(particles_.size());
    std::vector<double> likelihoods(particles_.size());
    std::vector<double> normalized_likelihoods(particles_.size());

    for(size_t i = 0; i < static_cast<size_t>(particles_.size()); ++i)
    {
        log_likelihoods[i] = lfm_->calulateLogLikelihoodPerParticle(particles_[i], laser_ranges);
    }

    // likelihood shift to prevent underflow
    double max_log_likelihood = *std::max_element(log_likelihoods.begin(), log_likelihoods.end());
    std::transform(log_likelihoods.begin(), log_likelihoods.end(), log_likelihoods.begin(),
                   [max_log_likelihood](double x){ return x - max_log_likelihood; });

    std::transform(log_likelihoods.begin(), log_likelihoods.end(), likelihoods.begin(),
                   [](double x){ return std::exp(x); });
    double total_likelihood = std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0);
    std::transform(likelihoods.begin(), likelihoods.end(), normalized_likelihoods.begin(),
                   [total_likelihood](double x){ return x / total_likelihood; });

    return normalized_likelihoods;
}

void ParticleFilter::estimatePose(const std::vector<double>& particle_weights)
{
    geometry_msgs::Pose2D::Ptr estimate_pose(new geometry_msgs::Pose2D());
    estimate_pose->x = 0.0;
    estimate_pose->y = 0.0;
    estimate_pose->theta = 0.0;

    for(size_t i = 0; i < static_cast<size_t>(particles_.size()); ++i)
    {
        estimate_pose->x += particle_weights[i] * particles_[i]->x;
        estimate_pose->y += particle_weights[i] * particles_[i]->y;

        double delta_theta = estimate_pose_->theta - particles_[i]->theta;
        // bring delta_theta into -pi < z < pi
        delta_theta = normalizeAngle(delta_theta);
        estimate_pose->theta += particle_weights[i] * delta_theta;
    }
    estimate_pose->theta = estimate_pose_->theta - estimate_pose->theta;
    // bring theta into -pi < z < pi
    estimate_pose->theta = normalizeAngle(estimate_pose->theta);
    estimate_pose_ = estimate_pose;
}

void ParticleFilter::resampleParticles(const std::vector<double>& particle_weights)
{
    std::vector<double> particle_weight_partial_sums(particle_weights);
    std::partial_sum(particle_weights.begin(), particle_weights.end(), particle_weight_partial_sums.begin());

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<double> dist_1(0.0, 1.0);

    for(size_t i = 0; i < static_cast<size_t>(particles_.size()); ++i)
    {
        auto it = std::lower_bound(particle_weight_partial_sums.begin(), particle_weight_partial_sums.end(), dist_1(engine));
        size_t j = std::distance(particle_weight_partial_sums.begin(), it);
        *particles_[i] = *particles_[j];
    }
}

geometry_msgs::PoseArray::Ptr ParticleFilter::createPoseArrayOfParticles()
{
    geometry_msgs::PoseArray::Ptr pa(new geometry_msgs::PoseArray());
    pa->poses.resize(particles_.size());
    pa->header.stamp = ros::Time::now();
    pa->header.frame_id = "map";
    for(size_t i = 0; i < static_cast<size_t>(particles_.size()); ++i)
    {
        pa->poses[i] = *createPose(particles_[i]);
    }
    return pa;
}

// Calculate the position of the odom relative to the map to align 
// the base_link with the estimate_pose of the map coordinate system.
geometry_msgs::TransformStamped::Ptr ParticleFilter::createTransformStampedOfOdomOnMap()
{
    geometry_msgs::TransformStamped::Ptr base_link_on_odom(new geometry_msgs::TransformStamped());
    try
    {
        *base_link_on_odom = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch(const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return nullptr;
    }

    geometry_msgs::Transform::Ptr odom_on_map(new geometry_msgs::Transform());
    geometry_msgs::Transform::Ptr estimate_pose_on_map = createTransform(estimate_pose_);

    // odom on map rotation
    tf2::Quaternion q_base_link_on_odom(0.0, 0.0, 0.0, 1.0);
    tf2::Quaternion q_estimate_pose_on_map(0.0, 0.0, 0.0, 1.0);
    q_base_link_on_odom.setZ(base_link_on_odom->transform.rotation.z);
    q_base_link_on_odom.setW(base_link_on_odom->transform.rotation.w);
    q_estimate_pose_on_map.setZ(estimate_pose_on_map->rotation.z);
    q_estimate_pose_on_map.setW(estimate_pose_on_map->rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_base_link_on_odom).getRPY(roll, pitch, yaw);
    double odom_on_map_theta = normalizeAngle(estimate_pose_->theta -yaw);
    odom_on_map->rotation.z = std::sin(odom_on_map_theta / 2);
    odom_on_map->rotation.w = std::cos(odom_on_map_theta / 2);

    // convert base_link on odom to polar coordinate system because of odom on map rotation
    double x = base_link_on_odom->transform.translation.x;
    double y = base_link_on_odom->transform.translation.y;
    double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double theta = std::atan2(y, x);
    // translation
    odom_on_map->translation.x = estimate_pose_on_map->translation.x - r * std::cos(theta + odom_on_map_theta);
    odom_on_map->translation.y = estimate_pose_on_map->translation.y - r * std::sin(theta + odom_on_map_theta);

    geometry_msgs::TransformStamped::Ptr tfs = createTransformStamped(odom_on_map, "map", "odom");
    return tfs;
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    delta_linear_ = msg->twist.twist.linear.x / ODOM_HZ_;
    delta_angular_ = msg->twist.twist.angular.z / ODOM_HZ_;
    
    if(scan_flag_)
    {
        mclLoop();
    }

    if(geometry_msgs::TransformStamped::Ptr tfs = createTransformStampedOfOdomOnMap())
    {
        tf_broadcaster_.sendTransform(*tfs);
    }

    geometry_msgs::Transform::Ptr tf = createTransform(estimate_pose_);
    geometry_msgs::TransformStamped::Ptr tfs = createTransformStamped(tf, "map", "estimate_pose");
    tf_broadcaster_.sendTransform(*tfs);

    geometry_msgs::PoseArray::Ptr pa = createPoseArrayOfParticles();
    pub_particles_.publish(pa);

    ROS_INFO("   x   : %lf", estimate_pose_->x);
    ROS_INFO("   y   : %lf", estimate_pose_->y);
    ROS_INFO(" theta : %lf", estimate_pose_->theta);
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ranges_.resize(msg->ranges.size());
    std::transform(msg->ranges.begin(), msg->ranges.end(), laser_ranges_.begin(),
                   [](const float& f) { return static_cast<double>(f); });
    scan_flag_ = true;
}