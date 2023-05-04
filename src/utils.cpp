#include "particle_filter/utils.h"

geometry_msgs::Pose::Ptr createPose(const geometry_msgs::Pose2D::ConstPtr& pose2d)
{
    geometry_msgs::Pose::Ptr pose(new geometry_msgs::Pose());
    pose->position.x = pose2d->x;
    pose->position.y = pose2d->y;
    pose->position.z = 0;
    pose->orientation.x = 0;
    pose->orientation.y = 0;
    pose->orientation.z = sin(pose2d->theta / 2);
    pose->orientation.w = cos(pose2d->theta / 2);
    return pose;
}

geometry_msgs::Transform::Ptr createTransform(const geometry_msgs::Pose2D::ConstPtr& pose2d)
{
    geometry_msgs::Transform::Ptr transform(new geometry_msgs::Transform());
    transform->translation.x = pose2d->x;
    transform->translation.y = pose2d->y;
    transform->translation.z = 0;
    transform->rotation.x = 0;
    transform->rotation.y = 0;
    transform->rotation.z = sin(pose2d->theta / 2);
    transform->rotation.w = cos(pose2d->theta / 2);
    return transform;
}

geometry_msgs::TransformStamped::Ptr createTransformStamped(
    const geometry_msgs::Transform::ConstPtr& transform, const std::string frame_id, const std::string child_frame_id
)
{
    geometry_msgs::TransformStamped::Ptr transform_stamped(new geometry_msgs::TransformStamped());
    transform_stamped->header.stamp = ros::Time::now();
    transform_stamped->header.frame_id = frame_id;
    transform_stamped->child_frame_id = child_frame_id;
    transform_stamped->transform = *transform;
    return transform_stamped;
}


geometry_msgs::Pose::Ptr convertTransform2Pose(const geometry_msgs::Transform::ConstPtr& transform)
{
    geometry_msgs::Pose::Ptr pose(new geometry_msgs::Pose());
    pose->position.x = transform->translation.x;
    pose->position.y = transform->translation.y;
    pose->position.z = transform->translation.z;
    pose->orientation.x = transform->rotation.x;
    pose->orientation.y = transform->rotation.y;
    pose->orientation.z = transform->rotation.z;
    pose->orientation.w = transform->rotation.w;
    return pose;
}

geometry_msgs::Transform::Ptr convertPose2Transform(const geometry_msgs::Pose::ConstPtr& pose)
{
    geometry_msgs::Transform::Ptr transform(new geometry_msgs::Transform());
    transform->translation.x = pose->position.x;
    transform->translation.y = pose->position.y;
    transform->translation.z = pose->position.z;
    transform->rotation.x = pose->orientation.x;
    transform->rotation.y = pose->orientation.y;
    transform->rotation.z = pose->orientation.z;
    transform->rotation.w = pose->orientation.w;
    return transform;
}

double normalizeAngle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}