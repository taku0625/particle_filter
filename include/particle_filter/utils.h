#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::Pose::Ptr createPose(const geometry_msgs::Pose2D::ConstPtr& pose2d);
geometry_msgs::Transform::Ptr createTransform(const geometry_msgs::Pose2D::ConstPtr& pose2d);
geometry_msgs::TransformStamped::Ptr createTransformStamped(
    const geometry_msgs::Transform::ConstPtr& transform, const std::string frame_id, const std::string child_frame_id
);
geometry_msgs::Pose::Ptr convertTransform2Pose(const geometry_msgs::Transform::ConstPtr& tf_data);
geometry_msgs::Transform::Ptr convertPose2Transform(const geometry_msgs::Pose::ConstPtr& pose);

double normalize_angle(double angle);

#endif