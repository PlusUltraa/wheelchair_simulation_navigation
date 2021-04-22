#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include <stdlib.h>

using namespace Eigen;

int main(int argc, char **argv){
    ros::init(argc, argv, "Goal");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);

    double x = atof(argv[1]);
    double y = atof(argv[2]);
    double a = atof(argv[3]);

    int count = 1;
    while(ros::ok()){
        Quaternion<float> q;
        q = AngleAxisf(0, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(a, Vector3f::UnitZ());
        geometry_msgs::PoseStamped Pose;

        Pose.header.seq = count;
        Pose.header.stamp = ros::Time::now();
        Pose.header.frame_id = "map";

        Pose.pose.position.x = x;
        Pose.pose.position.y = y;
        Pose.pose.position.z = 0.0;

        Pose.pose.orientation.x = q.x();
        Pose.pose.orientation.y = q.y();
        Pose.pose.orientation.z = q.z();
        Pose.pose.orientation.w = q.w();

        pub.publish(Pose);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        
        if(count > 3){
            break;
        }
    }

    return 0;
}
