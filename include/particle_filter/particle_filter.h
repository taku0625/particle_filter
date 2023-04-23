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

    void updateParticlesByOperatingModel(double delta_linear, double delta_angular);
    void calculateLikelihood();
    void estimatePose();
    void resampleParticles();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber tf_sub_;

    std::vector<std::vector<double>> particles_;
    double odom_[3];

    const double ODOM_HZ_ = 50;
    const double ODOM_NOISES[4] = {4.0, 1.0, 1.0, 4.0};
};