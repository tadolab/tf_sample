#ifndef TF_SAMPLE_TF_SAMPLE_H
#define TF_SAMPLE_TF_SAMPLE_H

#include <geometry_msgs/Quaternion.h>

struct TFSample {
    virtual
    void
    sample_broadcast() = 0;

    virtual
    void
    sample_listen() = 0;

    virtual
    void
    sample_transform() = 0;

    virtual
    std::tuple<double, double, double>
    quaternion_to_rpy(const geometry_msgs::Quaternion& q) = 0;

    virtual
    geometry_msgs::Quaternion
    rpy_to_quaternion(const double r, const double p, const double y) = 0;
};

#endif //TF_SAMPLE_TF_SAMPLE_H
