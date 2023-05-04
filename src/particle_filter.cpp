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
    readMap("/home/takumiti/catkin_ws/src/particle_filter/config/ogm.yaml", "/home/takumiti/catkin_ws/src/particle_filter/config/ogm.pgm");
    estimate_pose_.reset(new geometry_msgs::Pose2D());
    estimate_pose_->x = 0.0;
    estimate_pose_->y = 0.0;
    estimate_pose_->theta = 0.0;
    initializeParticles();

    pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("/particles", 1, this);

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

void ParticleFilter::readMap(std::string yaml_file_path, std::string img_file_path)
{
    YAML::Node lconf = YAML::LoadFile(yaml_file_path);
    map_resolution_ = lconf["resolution"].as<double>();
    map_origin_ = lconf["origin"].as<std::vector<double>>();

    cv::Mat map_img = cv::imread(img_file_path, cv::IMREAD_GRAYSCALE);
    map_width_ = map_img.cols;
    map_height_ = map_img.rows;

    cv::Mat bin_img;
    cv::threshold(map_img, bin_img, 0, 255, cv::THRESH_BINARY);

    cv::Mat distance_field(map_height_, map_width_, CV_32FC1);
    cv::distanceTransform(bin_img, distance_field, cv::DIST_L2, 5);
    for(int y = 0; y < map_height_; ++y)
    {
        for(int x = 0; x < map_width_; ++x)
        {
            float distance = distance_field.at<float>(y, x) * (float)map_resolution_;
            distance_field.at<float>(y, x) = distance;
        }
    }
    distance_field_ = distance_field.clone();
}

void ParticleFilter::initializeParticles()
{
    particles_.resize(PARTICLE_NUM_);

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<double> dist_1(0.0, 0.5);
    std::uniform_real_distribution<double> dist_2(0.0, 3.0 * M_PI / 180.0);

    for(size_t i = 0; i < PARTICLE_NUM_; ++i)
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

    for(size_t i = 0; i < particles_.size(); ++i)
    {
        particles_[i]->x += delta_linear_with_noise(engine) * std::cos(particles_[i]->theta);
        particles_[i]->y += delta_linear_with_noise(engine) * std::sin(particles_[i]->theta);
        particles_[i]->theta += delta_angular_with_noise(engine);
    }
};

std::vector<double> ParticleFilter::calculateLikelihoods(const std::vector<double>& laser_ranges)
{
    std::vector<double> log_likelihoods(particles_.size());
    std::vector<double> likelihoods(particles_.size());
    std::vector<double> normalized_likelihoods(particles_.size());

    for(size_t i = 0; i < particles_.size(); ++i)
    {
        log_likelihoods[i] = calulateLogLikelihoodPerParticle(particles_[i], laser_ranges);
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

    for(size_t i = 0; i < particles_.size(); ++i)
    {
        estimate_pose->x += particle_weights[i] * particles_[i]->x;
        estimate_pose->y += particle_weights[i] * particles_[i]->y;

        double delta_theta = estimate_pose_->theta - particles_[i]->theta;
        // bring delta_theta into -pi < z < pi
        delta_theta = normalize_angle(delta_theta);
        estimate_pose->theta += particle_weights[i] * delta_theta;
    }
    estimate_pose->theta = estimate_pose_->theta - estimate_pose->theta;
    // bring theta into -pi < z < pi
    estimate_pose->theta = normalize_angle(estimate_pose->theta);
    estimate_pose_ = estimate_pose;
}

void ParticleFilter::resampleParticles(const std::vector<double>& particle_weights)
{
    std::vector<double> particle_weight_partial_sums(particle_weights);
    std::partial_sum(particle_weights.begin(), particle_weights.end(), particle_weight_partial_sums.begin());

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<double> dist_1(0.0, 1.0);

    for(size_t i = 0; i < particles_.size(); ++i)
    {
        auto it = std::lower_bound(particle_weight_partial_sums.begin(), particle_weight_partial_sums.end(), dist_1(engine));
        size_t j = std::distance(particle_weight_partial_sums.begin(), it);
        *particles_[i] = *particles_[j];
    }
}

double ParticleFilter::calulateLogLikelihoodPerParticle(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges)
{
    double log_likelihood = 0.0;

    std::vector<double> p_max = pMax(laser_ranges);
    double p_rand = pRand();
    std::vector<double> p_hit = pHit(pose, laser_ranges);

    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        double p_LFM = Z_MAX_ * p_max[i] + Z_RAND_ * p_rand + Z_HIT_ * p_hit[i];
        p_LFM = p_LFM > 1.0 ? 1.0 : p_LFM;
        log_likelihood += std::log(p_LFM);
    }

    return log_likelihood;
}

std::vector<double> ParticleFilter::pMax(const std::vector<double>& laser_ranges)
{
    std::vector<double> p_max(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        p_max[i] = laser_ranges[i] == SCAN_RANGE_MAX_ ? 1.0 : 0.0;
    }
    return p_max;
}

double ParticleFilter::pRand()
{
    return 1 / SCAN_RANGE_MAX_;
}

std::vector<double> ParticleFilter::pHit(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges)
{
    double sum = 0.0;
    std::vector<double> p_hit(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        if(laser_ranges[i] < SCAN_RANGE_MIN_ || SCAN_RANGE_MAX_ < laser_ranges[i])
        {
            p_hit[i] = 0.0;
            continue;
        }
        double laser_angle = pose->theta + SCAN_ANGLE_MIN_ + (double)i * SCAN_ANGLE_INCREMENT_;
        // double laser_position_x = pose->x + laser_ranges[i] * std::cos(laser_angle);
        // double laser_position_y = pose->y + laser_ranges[i] * std::sin(laser_angle);
        double laser_position_x = pose->x + X_LIDER_ * std::cos(pose->theta) + laser_ranges[i] * std::cos(laser_angle);
        double laser_position_y = pose->y + X_LIDER_ * std::sin(pose->theta) + laser_ranges[i] * std::sin(laser_angle);
        // converted to opencv coordinate axes
        int ogm_laser_position_x = (laser_position_x - map_origin_[0]) / map_resolution_;
        int ogm_laser_position_y = map_height_ - 1 - (laser_position_y - map_origin_[1]) / map_resolution_;
        if(0 <= ogm_laser_position_x && ogm_laser_position_x < map_width_ && 0 <= ogm_laser_position_y && ogm_laser_position_y < map_height_)
        {
            double distance_field_value = (double)distance_field_.at<float>(ogm_laser_position_y, ogm_laser_position_x);
            p_hit[i] = 1.0 / std::sqrt(2.0 * M_PI * LFM_VAR_)
                       * std::exp(- std::pow(distance_field_value, 2.0) / (2.0 * LFM_VAR_));
        }
        else
        {
            p_hit[i] = 0.0;
        }
    }
    return p_hit;
}

geometry_msgs::PoseArray::Ptr ParticleFilter::createPoseArrayOfParticles()
{
    geometry_msgs::PoseArray::Ptr pa(new geometry_msgs::PoseArray());
    pa->poses.resize(particles_.size());
    pa->header.stamp = ros::Time::now();
    pa->header.frame_id = FRAME_ID_;
    for(size_t i = 0; i < particles_.size(); ++i)
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

    // translation
    odom_on_map->translation.x = estimate_pose_on_map->translation.x - base_link_on_odom->transform.translation.x;
    odom_on_map->translation.y = estimate_pose_on_map->translation.y - base_link_on_odom->transform.translation.y;

    // rotation
    tf2::Quaternion q_base_link, q_estimate_pose;
    q_base_link.setZ(base_link_on_odom->transform.rotation.z);
    q_base_link.setW(base_link_on_odom->transform.rotation.w);
    q_estimate_pose.setZ(estimate_pose_on_map->rotation.z);
    q_estimate_pose.setW(estimate_pose_on_map->rotation.w);
    tf2::Quaternion q_diff = q_base_link * q_estimate_pose.inverse();
    odom_on_map->rotation.z = q_diff.getZ();
    odom_on_map->rotation.w = q_diff.getW();

    geometry_msgs::TransformStamped::Ptr tfs = createTransformStamped(odom_on_map, "map", "odom");
    return tfs;
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    delta_linear_ = msg->twist.twist.linear.x / ODOM_HZ_;
    delta_angular_ = msg->twist.twist.angular.z / ODOM_HZ_;
    
    if(scan_flag_ == true)
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

    ROS_INFO("x%lf", estimate_pose_->x);
    ROS_INFO("y%lf", estimate_pose_->y);
    ROS_INFO("theta%lf", estimate_pose_->theta);
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ranges_.resize(msg->ranges.size());
    std::transform(msg->ranges.begin(), msg->ranges.end(), laser_ranges_.begin(),
                   [](const float& f) { return static_cast<double>(f); });
    scan_flag_ = true;
}