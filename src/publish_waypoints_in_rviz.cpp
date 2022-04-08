#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <optional>
#include <string_view>
#include <vector>

#include "utils/rviz.hpp"
#include "utils/transformlistener.hpp"

auto main(int argc, char* argv[]) -> int {
    using Eigen::Vector3f;
    using waypoints = std::vector<Vector3f>;

    ros::init(argc, argv, "publish_waypoints_in_rviz");
    auto nh = ros::NodeHandle();
    auto pub_waypoints = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    auto loop_rate = ros::Rate(10);
    auto publish = [&](const auto& msg) {
        pub_waypoints.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    };

    const auto loop_count = 10;

    auto tf_listener = utils::transform::TransformListener{};
    const auto to_frame = "map";
    const auto from_frame = "inspection";

    const auto origin = Vector3f{0, 0, 0};
    const auto hover_waypoint = Vector3f{0.f, 0.f, 2.f};
    const auto wps = waypoints{{-2, -2, 0}, {-2, 2, 0}, {2, 2, 0}, {2, -2, 0}, {-2, -2, 0}};
    const auto& wp0 = wps[0];

    auto arrow_msg_gen = utils::rviz::arrow_msg_gen{};
    arrow_msg_gen.header.frame_id = "map";
    arrow_msg_gen.scale.x = 0.05f;
    arrow_msg_gen.scale.y = 0.2f;
    arrow_msg_gen.scale.z = 0.5f;

    for (size_t i = 0; i < loop_count; i++) {
        const auto msg = arrow_msg_gen({origin, hover_waypoint});
        publish(msg);
    }

    if (const auto opt = tf_listener.transform_vec3(to_frame, from_frame, wp0)) {
        const auto point = *opt;
        for (size_t i = 0; i < loop_count; i++) {
            const auto msg = arrow_msg_gen({hover_waypoint, point});
            publish(msg);
        }
    }

    arrow_msg_gen.header.frame_id = "inspection";
    for (size_t i = 0; i < loop_count; i++) {
        auto it = wps.begin();
        it = std::next(it);
        for (; it != wps.end(); ++it) {
            const auto& wp1 = *std::prev(it);
            const auto& wp2 = *(it);
            const auto msg = arrow_msg_gen({wp1, wp2});
            pub_waypoints.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
