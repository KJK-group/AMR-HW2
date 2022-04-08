#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "time_since_start_publisher");
    auto nh = ros::NodeHandle();
    auto spin_rate = ros::Rate(10);

    auto start = ros::Time::now();
    auto pub = nh.advertise<std_msgs::Float64>("time_since_start", 10);
    auto publish = [&]() {
        const auto now = ros::Time::now();
        auto dt = (now - start);
        auto nsec = dt.toNSec();
        auto sec = dt.toSec();
        auto time = std_msgs::Float64{};
        time.data = sec;  //
        // time.data = nsec / 10e9;  //
        // time.data = ros::Time(sec, nsec);
        // time.time = t.to_time_t();
        pub.publish(time);
        ros::spinOnce();
        spin_rate.sleep();
    };
    while (ros::ok()) {
        publish();
    }

    return 0;
}
