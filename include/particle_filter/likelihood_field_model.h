#ifndef LIKELIHOOD_FIELD_MODEL_H
#define LIKELIHOOD_FIELD_MODEL_H

#include <geometry_msgs/Pose2D.h>
#include <opencv2/opencv.hpp>

class LikelihoodFieldModel
{
public:
    LikelihoodFieldModel(std::string yaml_file_path, std::string img_file_path);
    ~LikelihoodFieldModel();
    double calulateLogLikelihoodPerParticle(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges);

protected:
    void readMap(std::string yaml_file_path, std::string img_file_path);

    std::vector<double> pMax(const std::vector<double>& laser_ranges);
    double pRand();
    std::vector<double> pHit(const geometry_msgs::Pose2D::ConstPtr& pose, const std::vector<double>& laser_ranges);

private:
    double map_resolution_;
    std::vector<double> map_origin_;
    int map_width_;
    int map_height_;
    cv::Mat distance_field_;

    const double SCAN_ANGLE_MIN_ = -2.356194496154785;
    const double SCAN_ANGLE_MAX_ = 2.356194496154785;
    const double SCAN_ANGLE_INCREMENT_ = 0.004363323096185923;
    const double SCAN_RANGE_MIN_ = 0.05000000074505806;
    const double SCAN_RANGE_MAX_ = 30.0;
    const int SCAN_STEP_ = 5;
    const double LFM_VAR_ = 0.01;
    const double Z_MAX_ = 0.05;
    const double Z_RAND_ = 0.05;
    const double Z_HIT_ = 0.9;
    const double X_LIDER_ = 0.5;
};

#endif