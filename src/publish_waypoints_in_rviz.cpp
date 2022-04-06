#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <optional>
#include <string_view>
#include <vector>

#include "utils/rviz.hpp"

using Eigen::Vector3f;
using waypoints = std::vector<Vector3f>;

auto lookup_tf(const std::string& to_frame, const std::string& from_frame,
               tf2_ros::Buffer& tf_buffer, ros::Time time = ros::Time(0))
    -> std::optional<geometry_msgs::TransformStamped> {
    try {
        return tf_buffer.lookupTransform(to_frame, from_frame, time);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return std::nullopt;
    }
}

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "publish_waypoints_in_rviz");
    auto nh = ros::NodeHandle();
    auto pub_waypoints = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    auto loop_rate = ros::Rate(10);
    auto publish = [&](const auto& msg) {
        pub_waypoints.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ;
    };

    auto tf_buffer = tf2_ros::Buffer{};
    auto tf_listener = tf2_ros::TransformListener(tf_buffer);
    auto transform = geometry_msgs::TransformStamped{};
    const auto to_frame = "map";
    const auto from_frame = "inspection";

    const auto origin = Vector3f{0, 0, 0};
    const auto hover_waypoint = Vector3f{0.f, 0.f, 2.f};

    const auto wps = waypoints{{-2, -2, 0}, {-2, 2, 0}, {2, 2, 0}, {2, -2, 0}, {-2, -2, 0}};

    auto arrow_msg_gen = utils::rviz::arrow_msg_gen{};

    arrow_msg_gen.header.frame_id = "map";
    arrow_msg_gen.scale.x = 0.05f;
    arrow_msg_gen.scale.y = 0.2f;
    arrow_msg_gen.scale.z = 0.5f;
    for (size_t i = 0; i < 25; i++) {
        const auto msg = arrow_msg_gen({origin, hover_waypoint});
        publish(msg);
    }

    if (const auto transform = lookup_tf(to_frame, from_frame, tf_buffer)) {
        auto point_from_frame = geometry_msgs::PointStamped();
        const auto& wp0 = wps.at(0);
        point_from_frame.point.x = wp0.x();
        point_from_frame.point.y = wp0.y();
        point_from_frame.point.z = wp0.z();

        auto point_to_frame = geometry_msgs::PointStamped();
        tf2::doTransform(point_from_frame, point_to_frame, *transform);
        for (size_t i = 0; i < 25; i++) {
            const auto& point = point_from_frame.point;
            const auto x = point.x;
            const auto y = point.y;
            const auto z = point.z;
            const auto msg = arrow_msg_gen({hover_waypoint, {x, y, z}});
            publish(msg);
        }
    }

    arrow_msg_gen.header.frame_id = "inspection";
    for (size_t i = 0; i < 25; i++) {
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
