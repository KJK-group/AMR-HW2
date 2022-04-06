#ifndef _MULTI_DRONE_INSPECTION_RVIZ_HPP_
#define _MULTI_DRONE_INSPECTION_RVIZ_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cassert>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <string_view>

namespace utils::rviz {

struct RGBA {
    float r, g, b = 0.f, a = 1.f;
};

struct Scale {
    float x = 1.f, y = 1.f, z = 1.f;
};

struct Arrow {
    Eigen::Vector3f start, end;
};

struct Header {
    unsigned long long id = 0;
    std::string ns = "";
    std::string frame_id = "map";
};

enum class MarkerType {
    SPHERE,
    ARROW,
    CUBE,
};

namespace time {
// make the decltype slightly easier to the eye
using seconds_t = std::chrono::seconds;

auto get_seconds_since_epoch() -> decltype(seconds_t().count()) {
    const auto now = std::chrono::system_clock::now();
    const auto epoch = now.time_since_epoch();
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    return seconds.count();
}
}  // namespace time

struct visualization_marker_msg_gen {
    RGBA color{0, 1, 0, 1};
    Scale scale{0.1, 0.1, 0.1};
    Header header{};
    bool auto_incrementing_id = true;
    bool use_namespace_id_suffix = true;

    // visualization_marker_msg_gen() = delete;
    auto delete_all_markers_msg() -> visualization_msgs::Marker {
        // need to make a copy of the marker msg object to not modify the existing marker.
        auto delete_all_markers_msg = visualization_msgs::Marker{};
        msg.ns = header.ns;
        delete_all_markers_msg.action = visualization_msgs::Marker::DELETEALL;
        return delete_all_markers_msg;
    }

    //    private:
    virtual ~visualization_marker_msg_gen() {}

   protected:
    visualization_msgs::Marker msg{};
    unsigned long long id = header.id;
    auto set_fields() -> void {
        msg.ns = header.ns;
        msg.header.frame_id = header.frame_id;
        msg.id = get_id();
        msg.color.r = color.r;
        msg.color.g = color.g;
        msg.color.b = color.b;
        msg.color.a = color.a;
        msg.scale.x = scale.x;
        msg.scale.y = scale.y;
        msg.scale.z = scale.z;
    }
    auto get_id() -> decltype(id) { return auto_incrementing_id ? ++id : id; }
    auto append_epoch_suffix_if_enabled(const std::string& ns) -> std::string {
        const auto namespace_id_suffix = time::get_seconds_since_epoch();
        return ns + (use_namespace_id_suffix ? "/" + std::to_string(namespace_id_suffix) : "");
    }
};

struct arrow_msg_gen : public visualization_marker_msg_gen {
    arrow_msg_gen(const std::string& ns = "arrow") {
        msg.points.resize(2);
        msg.type = visualization_msgs::Marker::ARROW;
        header.ns = append_epoch_suffix_if_enabled(ns);
    }

    auto operator()(Arrow arrow, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;
        geometry_msgs::Pose pose{};
        pose.orientation.w = 1.0f;
        msg.pose = pose;

        msg.points[0] = vec3_to_geometry_msg_point_(arrow.start);
        msg.points[1] = vec3_to_geometry_msg_point_(arrow.end);

        return msg;
    }

   private:
    auto vec3_to_geometry_msg_point_(const Eigen::Vector3f& v) -> geometry_msgs::Point {
        auto p = geometry_msgs::Point{};
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    }
};

struct sphere_msg_gen : public visualization_marker_msg_gen {
    sphere_msg_gen(const std::string& ns = "sphere") {
        msg.type = visualization_msgs::Marker::SPHERE;
        header.ns = append_epoch_suffix_if_enabled(ns);
    }

    auto operator()(geometry_msgs::Pose pose, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;
        msg.pose = pose;

        return msg;
    }
};

struct text_msg_gen : public visualization_marker_msg_gen {
    text_msg_gen(float text_height = 0.5f, const std::string& ns = "text") {
        msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        header.ns = append_epoch_suffix_if_enabled(ns);
        scale.z = text_height;
    }
    auto operator()(std::string_view text, geometry_msgs::Pose pose,
                    ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;

        msg.pose = pose;
        msg.text = text;

        return msg;
    }
};

}  // namespace utils::rviz

#endif  // _MULTI_DRONE_INSPECTION_RVIZ_HPP_
