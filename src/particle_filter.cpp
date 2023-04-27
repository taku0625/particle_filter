#include "particle_filter/particle_filter.h"
#include <random>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Point.h>

using namespace std;

ParticleFilter::ParticleFilter(ros::NodeHandle& nh)
    : nh_(nh)
{
    readMap("/home/takumiti/catkin_ws/src/particle_filter/config/ogm.yaml", "/home/takumiti/catkin_ws/src/particle_filter/config/ogm.pgm");
    initializeParticles();

    pub_tf_ = nh_.advertise<geometry_msgs::Point>("/estimate_pose", 1, this);

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

    double total_squrares_of_likelihoods = std::accumulate(
        likelihoods.begin(), likelihoods.end(), 0.0, [](double a, double b)
        {
            return a + b * b;
        }
    );
    double effective_sample_size = 1.0 / total_squrares_of_likelihoods;
    double threshold = (double)particles_.size() * RESAMPLE_THRESHOLD_;

    if(! (effective_sample_size > threshold))
    {
        resampleParticles(likelihoods);
        geometry_msgs::Point estimate_pose;
        estimate_pose.x = pose_.position_x;
        estimate_pose.y = pose_.position_y;
        estimate_pose.z = pose_.angule_z;
        pub_tf_.publish(estimate_pose);
    }

    ROS_INFO("%lf", pose_.position_x);
    ROS_INFO("%lf", pose_.position_y);
    ROS_INFO("%lf", pose_.angule_z);
}

void ParticleFilter::readMap(std::string yaml_file_path, std::string img_file_path)
{
    YAML::Node lconf = YAML::LoadFile(yaml_file_path);
    map_resolution_ = lconf["resolution"].as<double>();

    map_origin_ = lconf["origin"].as<std::vector<double> >();

    cv::Mat tmp_map_img = cv::imread(img_file_path, 0);
    map_width_ = tmp_map_img.cols;
    map_height_ = tmp_map_img.rows;

    cv::Mat map_img = tmp_map_img.clone();
    for(int y = 0; y < map_height_; ++y)
    {
        for(int x = 0; x < map_width_; ++x)
        {
            uchar val = map_img.at<uchar>(y, x);
            if (val == 0)
            {
                map_img.at<uchar>(y, x) = 0;
            }
            else
                map_img.at<uchar>(y, x) = 1;
        }
    }
    cv::Mat distance_field(map_height_, map_width_, CV_32FC1);
    cv::distanceTransform(map_img, distance_field, cv::DIST_L2, 5);
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
    std::normal_distribution<double> dist_1(0.0, std::sqrt(0.5));
    std::uniform_real_distribution<double> dist_2(0.0, std::sqrt(3.0 * M_PI / 180.0));

    for(size_t i = 0; i < PARTICLE_NUM_; ++i)
    {
        particles_[i].position_x = dist_1(engine);
        particles_[i].position_y = dist_1(engine);
        particles_[i].angule_z = dist_2(engine);
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
        particles_[i].position_x += delta_linear_with_noise(engine) * std::cos(particles_[i].angule_z);
        particles_[i].position_y += delta_linear_with_noise(engine) * std::sin(particles_[i].angule_z);
        particles_[i].angule_z += delta_angular_with_noise(engine);
    }
};

std::vector<double> ParticleFilter::calculateLikelihoods(const std::vector<double>& laser_ranges)
{
    std::vector<double> likelihoods(particles_.size());
    std::vector<double> normalized_likelihoods(particles_.size());
    double total_likelihood = 0;
    for(size_t i = 0; i < particles_.size(); ++i)
    {
        double likelihood = calulateLikelihoodPerParticle(particles_[i], laser_ranges);
        likelihoods[i] = likelihood;
        total_likelihood += likelihood;
    }
    // normalize likelihood
    for(size_t i = 0; i < particles_.size(); ++i)
    {
        normalized_likelihoods[i] = likelihoods[i] / total_likelihood;
    }
    return normalized_likelihoods;
}

void ParticleFilter::estimatePose(const std::vector<double>& particle_weights)
{
    ParticleFilter::Pose estimate_pose = {0.0, 0.0, 0.0};
    for(size_t i = 0; i < particles_.size(); ++i)
    {
        estimate_pose.position_x += particle_weights[i] * particles_[i].position_x;
        estimate_pose.position_y += particle_weights[i] * particles_[i].position_y;

        double delta_angule_z = pose_.angule_z - particles_[i].angule_z;
        // bring delta_angule_z into -pi < z < pi
        int n = delta_angule_z / M_PI;
        delta_angule_z -= n * M_PI;
        estimate_pose.angule_z += particle_weights[i] * delta_angule_z;
    }
    estimate_pose.angule_z = pose_.angule_z - estimate_pose.angule_z;
    pose_ = estimate_pose;
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
        particles_[i] = particles_[j];
    }
}

double ParticleFilter::calulateLikelihoodPerParticle(const ParticleFilter::Pose pose, const std::vector<double>& laser_ranges)
{
    double likelihood = 0.0;

    std::vector<double> p_max = pMax(laser_ranges);
    double p_rand = pRand();
    std::vector<double> p_hit = pHit(pose, laser_ranges);

    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        double p_LFM = z_max_ * p_max[i] + z_rand_ * p_rand + z_hit_ * p_hit[i];
        p_LFM = p_LFM > 1.0 ? 1.0 : p_LFM;
        likelihood += std::log(p_LFM);
    }

    return std::exp(likelihood);
}

std::vector<double> ParticleFilter::pMax(const std::vector<double>& laser_ranges)
{
    std::vector<double> p_max(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        p_max[i] = laser_ranges[i] == MAX_LASER_RANGE_ ? 1.0 / map_resolution_ : 0.0;
    }
    return p_max;
}

double ParticleFilter::pRand()
{
    return 1 / MAX_LASER_RANGE_ * map_resolution_;
}

std::vector<double> ParticleFilter::pHit(const ParticleFilter::Pose pose, const std::vector<double>& laser_ranges)
{
    // ROS_INFO("%lf", pose.position_x);
    std::vector<double> p_hit(laser_ranges.size());
    for(size_t i = 0; i < laser_ranges.size(); i += SCAN_STEP_)
    {
        double laser_angle = pose.angule_z + SCAN_ANGLE_MIN_ + (double)i * SCAN_ANGLE_INCREMENT_;
        double laser_position_x = pose.position_x + laser_ranges[i] * std::cos(laser_angle);
        double laser_position_y = pose.position_y + laser_ranges[i] * std::sin(laser_angle);
        // converted to opencv coordinate axes
        int ogm_laser_position_x = (laser_position_x - map_origin_[0]) / map_resolution_;
        int ogm_laser_position_y = map_height_ - 1 - (laser_position_y - map_origin_[1]) / map_resolution_;
        if(0 <= ogm_laser_position_x && ogm_laser_position_x < map_width_ && 0 <= ogm_laser_position_y && ogm_laser_position_y < map_height_)
        {
            double distance_field_value = (double)distance_field_.at<float>(ogm_laser_position_x, ogm_laser_position_y);
            
            p_hit[i] = 1.0 / std::sqrt(2.0 * M_PI * LFM_VAR_) 
                       * std::exp(- std::pow(distance_field_value, 2.0) / (2.0 * LFM_VAR_))
                       * map_resolution_;
        }
        else
        {
            p_hit[i] = 0.0;
        }
    }
    return p_hit;
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    delta_linear_ = msg->twist.twist.linear.x / ODOM_HZ_;
    delta_angular_ = msg->twist.twist.angular.z / ODOM_HZ_;
    mclLoop();
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ranges_.resize(msg->ranges.size());
    std::transform(msg->ranges.begin(), msg->ranges.end(), laser_ranges_.begin(),
                   [](const float& f) { return static_cast<double>(f); });
}