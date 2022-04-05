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
    auto& header = crane_pose_msg.header;
    auto& position = crane_pose_msg.pose.position;
    auto& orientation = crane_pose_msg.pose.orientation;
    header.frame_id = "inspection";

    position.x = crane_x;
    position.y = crane_y;
    position.z = crane_z;
    orientation.x = 0;
    orientation.y = 0;
    orientation.z = 0.1613559;
    orientation.w = -0.9868963;

    const auto publish = [&]() {
        ROS_INFO("publishing crane_pose");
        header.stamp = ros::Time::now();
        pub_crane_pose.publish(crane_pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    };

    while (ros::ok()) {
        publish();
    }

    return 0;
}
