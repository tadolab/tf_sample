#include <ros/ros.h>

#include <tf_sample/tf_sample.h>
#include <tf_sample/tf1_sample.h>
#include <tf_sample/tf2_sample.h>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "tf_sample_node");

    TF1Sample tf1;
    TF2Sample tf2;

    ROS_INFO_STREAM("Waiting for listeners to start up");
    ros::Duration{1}.sleep();

    tf1.sample_broadcast();
    tf1.sample_listen();
    tf1.sample_transform();

    tf2.sample_broadcast();
    tf2.sample_listen();
    tf2.sample_transform();

    return 0;
}
