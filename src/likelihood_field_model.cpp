#include "particle_filter/likelihood_field_model.h"
#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

LikelihoodFieldModel::LikelihoodFieldModel(std::string yaml_file_path, std::string img_file_path)
{
    readMap(yaml_file_path, img_file_path);
}

LikelihoodFieldModel::~LikelihoodFieldModel()
{
}

void LikelihoodFieldModel::readMap(std::string yaml_file_path, std::string img_file_path)
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

double LikelihoodFieldModel::calulateLogLikelihoodPerParticle(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges)
{
    double log_likelihood = 0.0;

    std::vector<double> p_max = pMax(laser_ranges);
    double p_rand = pRand();
    std::vector<double> p_hit = pHit(pose, laser_ranges);

    for(size_t i = 0; i < static_cast<size_t>(laser_ranges.size()); i += SCAN_STEP_)
    {
        double p_LFM = Z_MAX_ * p_max[i] + Z_RAND_ * p_rand + Z_HIT_ * p_hit[i];
        p_LFM = std::max(p_LFM, 1.0);
        log_likelihood += std::log(p_LFM);
    }

    return log_likelihood;
}

std::vector<double> LikelihoodFieldModel::pMax(const std::vector<double>& laser_ranges)
{
    std::vector<double> p_max(laser_ranges.size());
    for(size_t i = 0; i < static_cast<size_t>(laser_ranges.size()); i += SCAN_STEP_)
    {
        p_max[i] = laser_ranges[i] == SCAN_RANGE_MAX_ ? 1.0 : 0.0;
    }
    return p_max;
}

double LikelihoodFieldModel::pRand()
{
    return 1 / SCAN_RANGE_MAX_;
}

std::vector<double> LikelihoodFieldModel::pHit(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges)
{
    std::vector<double> p_hit(laser_ranges.size());
    for(size_t i = 0; i < static_cast<size_t>(laser_ranges.size()); i += SCAN_STEP_)
    {
        if(laser_ranges[i] < SCAN_RANGE_MIN_ || SCAN_RANGE_MAX_ < laser_ranges[i])
        {
            p_hit[i] = 0.0;
            continue;
        }
        double laser_angle = pose->theta + SCAN_ANGLE_MIN_ + (double)i * SCAN_ANGLE_INCREMENT_;
        double laser_position_x = pose->x + laser_ranges[i] * std::cos(laser_angle);
        double laser_position_y = pose->y + laser_ranges[i] * std::sin(laser_angle);
        // double laser_position_x = pose->x + X_LIDER_ * std::cos(pose->theta) + laser_ranges[i] * std::cos(laser_angle);
        // double laser_position_y = pose->y + X_LIDER_ * std::sin(pose->theta) + laser_ranges[i] * std::sin(laser_angle);
        // converted to opencv coordinate axes
        int ogm_laser_position_x = std::rint((laser_position_x - map_origin_[0]) / map_resolution_);
        int ogm_laser_position_y = std::rint(map_height_ -1 - (laser_position_y - map_origin_[1]) / map_resolution_);
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
