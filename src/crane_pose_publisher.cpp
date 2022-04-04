#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <random>

auto random01() -> float {
    static auto gen = std::mt19937{std::random_device{}()};
    static std::uniform_real_distribution<float> dis(
        0, std::nextafter(1.f, std::numeric_limits<float>::max()));
    return dis(gen);
}

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "crane_pose_publisher");
    auto nh = ros::NodeHandle();
    auto pub_crane_pose = nh.advertise<geometry_msgs::PoseStamped>("crane_pose", 10);
    auto loop_rate = ros::Rate(10);

    const auto crane_x = (6 - 3) * random01() + 3;
    const auto crane_y = (6 - 3) * random01() + 3;
    const auto crane_z = (3 - 2) * random01() + 2;

    auto crane_pose_msg = geometry_msgs::PoseStamped{};
    crane_pose_msg.header.frame_id = "crane";
    crane_pose_msg.pose.position.x = crane_x;
    crane_pose_msg.pose.position.y = crane_y;
    crane_pose_msg.pose.position.z = crane_z;
    crane_pose_msg.pose.orientation.x = 0;
    crane_pose_msg.pose.orientation.y = 0;
    crane_pose_msg.pose.orientation.z = 0.1613559;
    crane_pose_msg.pose.orientation.w = -0.9868963;

    auto publish = [&]() {
        crane_pose_msg.header.stamp = ros::Time::now();
        ROS_INFO("publishing crane_pose");
        pub_crane_pose.publish(crane_pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    };

    while (ros::ok()) {
        publish();
    }

    return 0;
}
