#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "publish_traversed_path_in_rviz");
    auto nh = ros::NodeHandle();
    auto spin_rate = ros::Rate(10);

    auto path = nav_msgs::Path{};
    path.header.frame_id = "map";

    auto sub_odom = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        [&path](const geometry_msgs::PoseStamped::ConstPtr& msg) { path.poses.push_back(*msg); });

    auto pub_path = nh.advertise<nav_msgs::Path>("/traversed_path", 10);
    auto seq = 0;
    auto publish = [&](const auto& msg) {
        path.header.seq = seq++;
        path.header.stamp = ros::Time::now();
        pub_path.publish(msg);
        ROS_INFO("published path");
        ros::spinOnce();
        spin_rate.sleep();
        // path.poses.clear();
    };

    auto publish_rate = ros::Rate(5);
    while (ros::ok()) {
        publish_rate.sleep();
        publish(path);
    }

    return 0;
}
