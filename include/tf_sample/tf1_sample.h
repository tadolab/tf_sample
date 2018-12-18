#ifndef TF_SAMPLE_TF1_SAMPLE_H
#define TF_SAMPLE_TF1_SAMPLE_H

#include <tf_sample/tf_sample.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class TF1Sample : public TFSample {
public:
    static const std::string PARENT_FRAME_ID;
    static const std::string CHILD_FRAME_ID;

    TF1Sample();

    ~TF1Sample() = default;

    // No copy constructor since tf::TransformListener prohibits copying
    TF1Sample(const TF1Sample& other) = delete;

    // Unfortunately deleted because tf::TransformListener can't me moved
    TF1Sample(TF1Sample&& other) = delete;

    // No copy assignment since tf::TransformListener prohibits copying
    TF1Sample&
    operator=(const TF1Sample& other) = delete;

    // Unfortunately deleted because tf::TransformListener can't me moved
    TF1Sample&
    operator=(TF1Sample&& other) = delete;

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
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
};

#endif //TF_SAMPLE_TF1_SAMPLE_H
