#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdlib.h>

using namespace Eigen;

int main(int argc, char **argv){
    ros::init(argc, argv, "InitialPoser");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ros::Publisher pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);

    int count = 0;
    while(ros::ok()){
        double roll = atof(argv[4]);
        double pitch = atof(argv[5]);
        double yaw = atof(argv[6]);

        Quaternion<float> q;
        q = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(yaw, Vector3f::UnitZ());
        geometry_msgs::PoseWithCovarianceStamped initialPose;

        initialPose.header.seq = count;
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = "map";

        initialPose.pose.pose.position.x = atof(argv[1]);
        initialPose.pose.pose.position.y = atof(argv[2]);
        initialPose.pose.pose.position.z = atof(argv[3]);

        initialPose.pose.pose.orientation.x = q.x();
        initialPose.pose.pose.orientation.y = q.y();
        initialPose.pose.pose.orientation.z = q.z();
        initialPose.pose.pose.orientation.w = q.w();


        initialPose.pose.covariance[0] = 0.25;
        initialPose.pose.covariance[7] = 0.25;
        initialPose.pose.covariance[35] = 0.06853892326654787;

        pub.publish(initialPose);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if(count > 50){
            break;
        }
    }

    return 0;
}
