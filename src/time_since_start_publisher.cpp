#include <ros/ros.h>
#include <std_msgs/Float64.h>

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "time_since_start_publisher");
    auto nh = ros::NodeHandle();
    auto spin_rate = ros::Rate(10);

    const auto start = ros::Time::now();
    auto pub = nh.advertise<std_msgs::Float64>("time_since_start", 10);
    const auto publish = [&]() {
        const auto now = ros::Time::now();
        const auto dt = (now - start);
        const auto sec = dt.toSec();
        auto msg = std_msgs::Float64{};
        msg.data = sec;
        pub.publish(msg);
        ros::spinOnce();
        spin_rate.sleep();
    };
    while (ros::ok()) {
        publish();
    }

    return 0;
}
