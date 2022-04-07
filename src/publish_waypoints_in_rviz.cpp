#include <ros/ros.h>
// #include <tf2/utils.h>
// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <optional>
#include <string_view>
#include <vector>

#include "utils/rviz.hpp"
#include "utils/transformlistener.hpp"

#define let const auto
#define var auto
#define val const auto
#define ref var&
#define cref val&

template <typename T>
using Option = std::optional<T>;
// using None = std::nullopt;

// template <typename T>
// using Some = std::make_optional<T>(T&&);

using Eigen::Vector3f;
using waypoints = std::vector<Vector3f>;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "publish_waypoints_in_rviz");
    var nh = ros::NodeHandle();
    var pub_waypoints = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    var loop_rate = ros::Rate(10);
    var publish = [&](val& msg) {
        pub_waypoints.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    };

    val loop_count = 10;

    var tf_listener = utils::transform::TransformListener{};
    val to_frame = "map";
    val from_frame = "inspection";

    val origin = Vector3f{0, 0, 0};
    val hover_waypoint = Vector3f{0.f, 0.f, 2.f};
    val wps = waypoints{{-2, -2, 0}, {-2, 2, 0}, {2, 2, 0}, {2, -2, 0}, {-2, -2, 0}};
    val& wp0 = wps[0];

    var arrow_msg_gen = utils::rviz::arrow_msg_gen{};
    arrow_msg_gen.header.frame_id = "map";
    arrow_msg_gen.scale.x = 0.05f;
    arrow_msg_gen.scale.y = 0.2f;
    arrow_msg_gen.scale.z = 0.5f;

    for (size_t i = 0; i < loop_count; i++) {
        val msg = arrow_msg_gen({origin, hover_waypoint});
        publish(msg);
    }

    if (val opt = tf_listener.transform_vec3(to_frame, from_frame, wp0)) {
        val point = *opt;
        for (size_t i = 0; i < loop_count; i++) {
            val msg = arrow_msg_gen({hover_waypoint, point});
            publish(msg);
        }
    }

    arrow_msg_gen.header.frame_id = "inspection";
    for (size_t i = 0; i < loop_count; i++) {
        var it = wps.begin();
        it = std::next(it);
        for (; it != wps.end(); ++it) {
            val& wp1 = *std::prev(it);
            val& wp2 = *(it);
            val msg = arrow_msg_gen({wp1, wp2});
            pub_waypoints.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
