#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <optional>
#include <vector>

#include "amr_hw2/PointNormStamped.h"
#include "boost/format.hpp"
#include "utils/rviz.hpp"

constexpr auto TOLERANCE = 0.25;
constexpr auto TOLERANCE_VELOCITY = 0.15;
constexpr auto TOLERANCE_ACCELERATION = 0.25;
// escape codes
constexpr auto MAGENTA = "\u001b[35m";
constexpr auto GREEN = "\u001b[32m";
constexpr auto RESET = "\u001b[0m";
constexpr auto BOLD = "\u001b[1m";
constexpr auto ITALIC = "\u001b[3m";
constexpr auto UNDERLINE = "\u001b[4m";
// frames
constexpr auto FRAME_WORLD = "map";
constexpr auto FRAME_INSPECTION = "inspection";
constexpr auto FRAME_BODY = "odom";

using boost::format;
using boost::io::group;
using Eigen::Vector3f;
using std::setfill;
using std::setw;
using std::stof;
using std::string;
using std::vector;

// state enumeration
enum state { PASSIVE, HOVER, INSPECTION, LAND };
auto state_to_string(state s) -> string {
    switch (s) {
        case PASSIVE:
            return "PASSIVE";
            break;
        case HOVER:
            return "HOVER";
            break;
        case INSPECTION:
            return "INSPECTION";
            break;
        case LAND:
            return "LAND";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}
state mission_state = PASSIVE;

// publishers
ros::Publisher pub_velocity;
ros::Publisher pub_waypoints;
ros::Publisher pub_error;

// subscribers
ros::Subscriber sub_state;
ros::Subscriber sub_odom;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;
ros::ServiceClient client_land;

// state variables
mavros_msgs::State drone_state;
nav_msgs::Odometry odom;
auto inspection_completed = false;

// transform utilities
tf2_ros::Buffer tf_buffer;

// controller gains
auto kp = 1.f;
auto ki = 1.f;
auto kd = 1.f;

// sequence counters
auto seq_tf = 0;
auto seq_error = 0;

// waypoints
auto hover_waypoint = Vector3f(0, 0, 2);
auto inspection_waypoints =
    vector<Vector3f>{{-2, -2, 0}, {-2, 2, 0}, {2, 2, 0}, {2, -2, 0}, {-2, -2, 0}};
auto waypoint_idx = 0;

// error
auto error_integral = Vector3f(0, 0, 0);
auto error_previous = Vector3f(0, 0, 0);
// previous command - used for logging
geometry_msgs::TwistStamped command_previous;

// time
ros::Time start_time;

auto sphere_msg_gen = utils::rviz::sphere_msg_gen();

auto previous_velocity = Vector3f(0, 0, 0);
auto acceleration = Vector3f(0, 0, 0);
auto velocity = Vector3f(0, 0, 0);
auto acceleration_from_odom() -> Vector3f {
    acceleration =
        previous_velocity -
        Vector3f(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    return acceleration;
}
auto velocity_from_odom() -> Vector3f {
    velocity =
        Vector3f(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    return velocity;
}

//--------------------------------------------------------------------------------------------------
// utility functions
//--------------------------------------------------------------------------------------------------
// transforms `point` from `from_frame` to `to_frame`
// returns the transformed point
geometry_msgs::TransformStamped transform;
auto transform_point(Vector3f point, string from_frame, string to_frame)
    -> std::optional<Vector3f> {
    // std::cout << MAGENTA << "input point:\n" << point << RESET << std::endl;

    // lookup transform
    try {
        transform = tf_buffer.lookupTransform(to_frame, from_frame, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(1.0).sleep();
        return std::nullopt;
    }

    // stamped point message in frame `from_frame`
    auto point_from_frame = geometry_msgs::PointStamped();
    // fill header
    point_from_frame.header.seq = seq_tf++;
    point_from_frame.header.stamp = ros::Time::now();
    point_from_frame.header.frame_id = from_frame;
    // fill point data
    point_from_frame.point.x = point.x();
    point_from_frame.point.y = point.y();
    point_from_frame.point.z = point.z();

    // stamped point message in frame `to_frame`
    auto point_to_frame = geometry_msgs::PointStamped();
    // fill header
    point_to_frame.header.seq = seq_tf++;
    point_to_frame.header.stamp = ros::Time::now();
    point_to_frame.header.frame_id = to_frame;
    // fill point data by applying transform
    tf2::doTransform(point_from_frame, point_to_frame, transform);

    // std::cout << MAGENTA << "output point:\n" << point_to_frame << RESET << std::endl;

    auto transformed_point =
        Vector3f(point_to_frame.point.x, point_to_frame.point.y, point_to_frame.point.z);
    auto msg = sphere_msg_gen(point);
    msg.header.frame_id = from_frame;
    pub_waypoints.publish(msg);
    auto msg2 = sphere_msg_gen(transformed_point);
    msg2.color.r = 1.f;
    msg2.color.g = 0.f;
    msg2.header.frame_id = to_frame;
    pub_waypoints.publish(msg2);

    return transformed_point;
}
//--------------------------------------------------------------------------------------------------
// takes a 3D euclidean position error `error`,
// updates integrat and derivative errors,
// applies PID controller to produce velocity commands
auto command_drone(Vector3f error) -> geometry_msgs::TwistStamped {
    if (drone_state.armed) {
        error_integral += error;
    }                                                // update integral error
    auto error_derivative = error - error_previous;  // change in error since previous time step

    // std::cout << MAGENTA << "error: " << error << std::endl;
    // std::cout << MAGENTA << "error_integral: " << error_integral << std::endl;
    // std::cout << MAGENTA << "error_derivative: " << error_integral << std::endl;

    // error terms
    // auto proportional_term = kp * error;
    // auto integral_term = ki * error_integral;
    // auto derivative_term = kd * error_derivative;

    auto c = kp * error + ki * error_integral + kd * error_derivative;

    // std::cout << MAGENTA << "proportional_term: " << proportional_term << std::endl;
    // std::cout << MAGENTA << "integral_term: " << integral_term << std::endl;
    // std::cout << MAGENTA << "derivative_term: " << derivative_term << std::endl;

    // message to publish
    geometry_msgs::TwistStamped command{};
    command.twist.linear.x = c.x();
    command.twist.linear.y = c.y();
    command.twist.linear.z = c.z();

    // current error is the previous error for the next time step
    error_previous = error;

    command_previous = command;

    return command;
}

//--------------------------------------------------------------------------------------------------
// callback functions
//--------------------------------------------------------------------------------------------------
auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    odom = *msg;
    // static auto path_msg_gen = utils::rviz::sphere_msg_gen("drone_path");
    // auto path_msg = path_msg_gen(odom.pose.pose);
    // path_msg.color.b = 1.f;
    // path_msg.color.a = 0.5f;
    // pub_waypoints.publish(path_msg);
    // ros::spinOnce();
}
auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { drone_state = *msg; }

//--------------------------------------------------------------------------------------------------
// main program body
//--------------------------------------------------------------------------------------------------
auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mission_state_machine_node");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    start_time = ros::Time::now();
    //----------------------------------------------------------------------------------------------
    // transform utilities
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //----------------------------------------------------------------------------------------------
    // pass potential controller gain arguments
    if (argc > 1) kp = stof(argv[1]);
    if (argc > 2) ki = stof(argv[2]);
    if (argc > 3) kd = stof(argv[3]);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    pub_waypoints = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    pub_error = nh.advertise<amr_hw2::PointNormStamped>("/amr_hw2/error", 10);

    //----------------------------------------------------------------------------------------------
    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // land service client
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    //----------------------------------------------------------------------------------------------
    // wait for FCU connection
    while (ros::ok() && !drone_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    auto previous_request_time = ros::Time(0);
    // offboard mode message
    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = "OFFBOARD";
    // arm message
    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    //----------------------------------------------------------------------------------------------
    // ROS control loop
    while (ros::ok()) {
        //------------------------------------------------------------------------------------------
        // control the drone

        geometry_msgs::TransformStamped transform;
        geometry_msgs::TwistStamped command;
        // std::optional<Vector3f> error;

        switch (mission_state) {
            case PASSIVE:
                // state change
                // in case the drone is ready start the mission
                // if (drone_state.armed && drone_state.mode == "OFFBOARD") {
                //     mission_state = HOVER;
                // }
                mission_state = HOVER;
                break;
            case HOVER:
                // control
                if (auto error = transform_point(hover_waypoint, FRAME_WORLD, FRAME_BODY)) {
                    command = command_drone(*error);
                    pub_velocity.publish(command);

                    // state change
                    if ((*error).norm() < TOLERANCE &&
                        acceleration_from_odom().norm() < TOLERANCE_ACCELERATION &&
                        velocity_from_odom().norm() <
                            TOLERANCE_VELOCITY) {    // within tolerance, change state
                        if (inspection_completed) {  // change to waypoint navigation or land
                            mission_state = LAND;
                        } else {
                            mission_state = INSPECTION;
                        }
                    }
                }
                break;
            case INSPECTION:
                // control
                if (auto error = transform_point(inspection_waypoints[waypoint_idx],
                                                 FRAME_INSPECTION, FRAME_BODY)) {
                    command = command_drone(*error);
                    pub_velocity.publish(command);

                    // state change
                    if ((*error).norm() < TOLERANCE &&
                        acceleration_from_odom().norm() < TOLERANCE_ACCELERATION &&
                        velocity_from_odom().norm() <
                            TOLERANCE_VELOCITY) {  // within tolerance, change to
                        // next waypoint, reset integral error
                        waypoint_idx++;
                        error_integral = Vector3f(0, 0, 0);
                        // change back to HOVER if all waypoints have been visited
                        if (waypoint_idx >= inspection_waypoints.size()) {
                            inspection_completed = true;
                            mission_state = HOVER;
                        }
                    }
                }
                break;
            case LAND:
                // request to land every 5 seconds until drone is landing
                if (drone_state.mode != "AUTO.LAND" &&
                    (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
                    mavros_msgs::CommandTOL land_msg;
                    land_msg.request.altitude = 2;

                    if (client_land.call(land_msg)) {
                        ROS_INFO("drone landing service request: success");
                    } else {
                        ROS_INFO("drone landing service request: fail");
                    }
                    previous_request_time = ros::Time::now();
                }
                break;
            default:
                ROS_WARN_STREAM("Unknown state");
                break;
        }

        // publish custom error msg for plotting
        amr_hw2::PointNormStamped error_msg;
        error_msg.header.seq = seq_error++;
        error_msg.header.stamp = ros::Time::now();
        error_msg.point.x = error_previous.x();
        error_msg.point.y = error_previous.y();
        error_msg.point.z = error_previous.z();
        error_msg.norm = error_previous.norm();
        pub_error.publish(error_msg);

        //------------------------------------------------------------------------------------------
        // request to set drone mode to OFFBOARD every 5 seconds until drone is in OFFBOARD mode
        if (drone_state.mode != "OFFBOARD" &&
            (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
            if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
                ROS_INFO("mode set: OFFBOARD");
            } else {
                ROS_INFO("mode set: fail");
            }
            previous_request_time = ros::Time::now();
        }
        //------------------------------------------------------------------------------------------
        // request to arm throttle every 5 seconds until drone is armed
        if (!drone_state.armed && (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
            if (client_arm.call(srv)) {
                ROS_INFO("throttle armed: success");
            } else {
                ROS_INFO("throttle armed: fail");
            }
            previous_request_time = ros::Time::now();
        }

        auto delta_time = (ros::Time::now() - start_time).toSec();
        int w;
        int time = delta_time;
        for (w = 0; time > 0; w++) {
            time /= 10;
        }

        //------------------------------------------------------------------------------------------
        // ROS logging
        //------------------------------------------------------------------------------------------
        // mission state
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "mission:" << RESET);
        ROS_INFO_STREAM("  time:  " << format("%1.2f") %
                                           group(setfill(' '), setw(w + 2), delta_time));
        ROS_INFO_STREAM("  index: " << format("%1.2f") %
                                           group(setfill(' '), setw(w), waypoint_idx + 1));
        ROS_INFO_STREAM("  state: " << format("%1.2f") % group(setfill(' '), setw(w),
                                                               state_to_string(mission_state)));
        ROS_INFO_STREAM("  waypoint:");
        ROS_INFO_STREAM(
            "    x: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx].x()));
        ROS_INFO_STREAM(
            "    y: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx].y()));
        ROS_INFO_STREAM(
            "    z: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx].z()));
        //------------------------------------------------------------------------------------------
        // drone position
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "position:" << RESET);
        ROS_INFO_STREAM("  x:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), odom.pose.pose.position.x));
        ROS_INFO_STREAM("  y:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), odom.pose.pose.position.y));
        ROS_INFO_STREAM("  z:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), odom.pose.pose.position.z));
        ROS_INFO_STREAM("  norm: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                              Vector3f(odom.pose.pose.position.x,
                                                                       odom.pose.pose.position.y,
                                                                       odom.pose.pose.position.z)
                                                                  .norm()));
        //------------------------------------------------------------------------------------------
        // position errors
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "errors:" << RESET);
        ROS_INFO_STREAM("  x:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), error_previous.x()));
        ROS_INFO_STREAM("  y:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), error_previous.y()));
        ROS_INFO_STREAM("  z:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), error_previous.z()));
        ROS_INFO_STREAM("  norm: " << format("%1.5f") %
                                          group(setfill(' '), setw(8), error_previous.norm()));
        //------------------------------------------------------------------------------------------
        // controller outputs
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "controller outputs:" << RESET);
        ROS_INFO_STREAM("  x_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.x));
        ROS_INFO_STREAM("  y_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.y));
        ROS_INFO_STREAM("  z_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.z));
        ROS_INFO_STREAM("  norm:  " << format("%1.5f") %
                                           group(setfill(' '), setw(8),
                                                 Vector3f(command_previous.twist.linear.x,
                                                          command_previous.twist.linear.y,
                                                          command_previous.twist.linear.z)
                                                     .norm()));
        //------------------------------------------------------------------------------------------
        // acceleration
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "acceleration:" << RESET);
        ROS_INFO_STREAM("  x:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), acceleration.x()));
        ROS_INFO_STREAM("  y:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), acceleration.y()));
        ROS_INFO_STREAM("  z:    " << format("%1.5f") %
                                          group(setfill(' '), setw(8), acceleration.z()));
        ROS_INFO_STREAM("  norm: " << format("%1.5f") %
                                          group(setfill(' '), setw(8), acceleration.norm()));
        //------------------------------------------------------------------------------------------
        // velocity
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "velocity:" << RESET);
        ROS_INFO_STREAM("  x:    " << format("%1.5f") % group(setfill(' '), setw(8), velocity.x()));
        ROS_INFO_STREAM("  y:    " << format("%1.5f") % group(setfill(' '), setw(8), velocity.y()));
        ROS_INFO_STREAM("  z:    " << format("%1.5f") % group(setfill(' '), setw(8), velocity.z()));
        ROS_INFO_STREAM("  norm: " << format("%1.5f") %
                                          group(setfill(' '), setw(8), velocity.norm()));

        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
}
