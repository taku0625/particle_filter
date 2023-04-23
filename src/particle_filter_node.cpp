#include "particle_filter/particle_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;
    ParticleFilter filter(nh);
    ros::spin();

    return 0;
}