#ifndef TF_SAMPLE_TF2_SAMPLE_H
#define TF_SAMPLE_TF2_SAMPLE_H

#include <tf_sample/tf_sample.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TF2Sample : public TFSample {
public:
    static const std::string PARENT_FRAME_ID;
    static const std::string CHILD_FRAME_ID;

    TF2Sample();

    ~TF2Sample() = default;

    // No copy constructor since tf2_ros::Buffer prohibits copying
    TF2Sample(const TF2Sample& other) = delete;

    // Unfortunately deleted because tf2_ros::Buffer can't me moved
    TF2Sample(TF2Sample&& other) = delete;

    // No copy assignment since tf2_ros::Buffer prohibits copying
    TF2Sample&
    operator=(const TF2Sample& other) = delete;

    // Unfortunately deleted because tf2_ros::Buffer can't me moved
    TF2Sample&
    operator=(TF2Sample&& other) = delete;

    void
    sample_broadcast() override;

    void
    sample_listen() override;

    void
    sample_transform() override;

    std::tuple<double, double, double>
    quaternion_to_rpy(const geometry_msgs::Quaternion& q) override;

    geometry_msgs::Quaternion
    rpy_to_quaternion(const double r, const double p, const double y) override;

protected:
    tf2_ros::Buffer buf_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tf_static_;
};


#endif //TF_SAMPLE_TF2_SAMPLE_H
