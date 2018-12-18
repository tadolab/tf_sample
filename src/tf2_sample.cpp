#include "tf_sample/tf2_sample.h"

const std::string TF2Sample::PARENT_FRAME_ID = "tf2/parent";
const std::string TF2Sample::CHILD_FRAME_ID = "tf2/child";

TF2Sample::TF2Sample()
    : buf_{}
    , tf_listener_{buf_}
    , tf_broadcaster_{}
    , tf_static_{}
{
}

void
TF2Sample::sample_broadcast() {
    geometry_msgs::TransformStamped tform;
    tform.header.stamp = ros::Time::now();
    tform.header.frame_id = PARENT_FRAME_ID;
    tform.child_frame_id = CHILD_FRAME_ID;
    tform.transform.translation.x = 3;
    tform.transform.rotation.w = 1;

    tf_broadcaster_.sendTransform(tform);

    // Static broadcasting using tf2_ros::StaticTransformBroadcaster
    geometry_msgs::TransformStamped tform_static;
    tform_static.header.stamp = ros::Time::now();
    tform_static.header.frame_id = PARENT_FRAME_ID + "/static";
    tform_static.child_frame_id = CHILD_FRAME_ID + "/static";
    tform_static.transform.translation.y = 1.5;
    tform_static.transform.rotation.z = 1;

    tf_static_.sendTransform(tform_static);
}

void
TF2Sample::sample_listen() {
    const auto timeout = ros::Duration{5};

    ROS_INFO_STREAM("Waiting " << timeout.toSec() << " secs for transform");
    /*
     * Note:
     * tf2_ros::Buffer does not have waitForTransform, instead, use
     * the optional timeout argument when calling the transform methods such
     * as canTransform, lookupTransform, transform, etc.
     */

    if (buf_.canTransform(PARENT_FRAME_ID, CHILD_FRAME_ID, ros::Time{0})) {
        const auto tform = buf_.lookupTransform(PARENT_FRAME_ID,
                                                CHILD_FRAME_ID,
                                                ros::Time{0});

        ROS_INFO_STREAM("Transform received: " << std::endl << tform);
    }
    else {
        ROS_WARN_STREAM("Transformation between "
                                << PARENT_FRAME_ID
                                << " and "
                                << CHILD_FRAME_ID
                                << " not available");
    }
}

void
TF2Sample::sample_transform() {
    auto parent_pose = geometry_msgs::PoseStamped{};
    parent_pose.header.frame_id = PARENT_FRAME_ID;
    parent_pose.pose.position.x = 1;
    parent_pose.pose.position.y = 2;
    parent_pose.pose.position.z = 0.5;
    parent_pose.pose.orientation.w = 1;

    /*
     * Can simply be:
     * const auto child_pose = buf_.transform(parent_pose, CHILD_FRAME_ID);
     */
    auto child_pose = geometry_msgs::PoseStamped{};
    // Full definition
    buf_.transform(parent_pose,
                   child_pose,
                   CHILD_FRAME_ID,
                   ros::Time{0},
                   PARENT_FRAME_ID);

    ROS_INFO_STREAM("Parent: " << std::endl << parent_pose);
    ROS_INFO_STREAM("Child: " << std::endl << child_pose);
}

std::tuple<double, double, double>
TF2Sample::quaternion_to_rpy(const geometry_msgs::Quaternion& q) {
    auto q_tf = tf2::Quaternion{};
    // Convert a ROS quaternion message to tf2 quaternion
    tf2::convert(q, q_tf);

    double r, p, y;
    tf2::Matrix3x3{q_tf}.getRPY(r, p, y);

    return std::make_tuple(r, p, y);
}

geometry_msgs::Quaternion
TF2Sample::rpy_to_quaternion(const double r, const double p, const double y) {
    auto mat = tf2::Matrix3x3{};
    mat.setRPY(r, p, y);

    tf2::Quaternion q;
    mat.getRotation(q);

    auto q_msg = geometry_msgs::Quaternion{};
    // Convert tf2 quaternion to ROS quaternion message
    tf2::convert(q, q_msg);
    return q_msg;
}
