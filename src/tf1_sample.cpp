#include <tf_sample/tf1_sample.h>

const std::string TF1Sample::PARENT_FRAME_ID = "tf1/parent";
const std::string TF1Sample::CHILD_FRAME_ID = "tf1/child";

TF1Sample::TF1Sample()
    : tf_broadcaster_{}
    , tf_listener_{}
{
}

void
TF1Sample::sample_broadcast() {
    geometry_msgs::TransformStamped tform;
    tform.header.stamp = ros::Time::now();
    tform.header.frame_id = PARENT_FRAME_ID;
    tform.child_frame_id = CHILD_FRAME_ID;
    tform.transform.translation.x = 3;
    tform.transform.rotation.w = 1;

    /*
     * Note:
     * sendTransform can accept four types of arguments:
     * - geometry_msgs::TransformStamped
     * - std::vector<geometry_msgs::TransformStamped>
     * - tf::StampedTransform
     * - std::vector<tf::StampedTransform>
     *
     * It is recommended that you use geometry_msgs::TransformStamped if you
     * do need to use TF1. However, use TF2 instead of TF1 in the first place.
     */
    tf_broadcaster_.sendTransform(tform);
    ROS_INFO_STREAM("Sent transform: " << std::endl << tform);
}

void
TF1Sample::sample_listen() {
    const auto parent = PARENT_FRAME_ID;
    const auto child = CHILD_FRAME_ID;

    const auto timeout = ros::Duration{5};

    ROS_INFO_STREAM("Waiting " << timeout.toSec() << " secs for transform");
    // Note: waitForTransform returns false if timed out
    tf_listener_.waitForTransform(parent, child, ros::Time{0}, timeout);

    if (tf_listener_.canTransform(parent, child, ros::Time{0})) {
        tf::StampedTransform tform;
        tf_listener_.lookupTransform(parent, child, ros::Time{0}, tform);

        geometry_msgs::TransformStamped tform_msg;
        tf::transformStampedTFToMsg(tform, tform_msg);
        ROS_INFO_STREAM("Transform received: " << std::endl << tform_msg);
    }
    else {
        ROS_WARN_STREAM("Transformation between "
                                << parent
                                << " and "
                                << child
                                << " not available");
    }
}

void
TF1Sample::sample_transform() {
    auto parent_pose = geometry_msgs::PoseStamped{};
    parent_pose.header.frame_id = PARENT_FRAME_ID;
    parent_pose.pose.position.x = 1;
    parent_pose.pose.position.y = 2;
    parent_pose.pose.position.z = 0.5;
    parent_pose.pose.orientation.w = 1;

    auto child_pose = geometry_msgs::PoseStamped{};
    tf_listener_.transformPose(CHILD_FRAME_ID,
                               ros::Time{0},
                               parent_pose,
                               PARENT_FRAME_ID,
                               child_pose);

    ROS_INFO_STREAM("Parent: " << std::endl << parent_pose);
    ROS_INFO_STREAM("Child: " << std::endl << child_pose);
}

std::tuple<double, double, double>
TF1Sample::quaternion_to_rpy(const geometry_msgs::Quaternion& q) {
    // Note: The following can be used to simply get the yaw
    // const auto yaw = tf::getYaw(q);

    tf::Quaternion q_tf;
    tf::quaternionMsgToTF(q, q_tf);

    double r, p, y;
    tf::Matrix3x3{q_tf}.getRPY(r, p, y);

    return std::make_tuple(r, p, y);
}

geometry_msgs::Quaternion
TF1Sample::rpy_to_quaternion(const double r, const double p, const double y) {
    auto mat = tf::Matrix3x3{};
    mat.setRPY(r, p, y);

    auto q_tf = tf::Quaternion{};
    mat.getRotation(q_tf);

    auto q_msg = geometry_msgs::Quaternion{};
    tf::quaternionTFToMsg(q_tf, q_msg);

    return q_msg;
}
