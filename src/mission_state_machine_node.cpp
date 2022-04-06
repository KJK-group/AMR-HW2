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

#include "boost/format.hpp"

#define TOLERANCE 0.1
// escape codes
#define MAGENTA "\u001b[35m"
#define GREEN "\u001b[32m"
#define RESET "\u001b[0m"
#define BOLD "\u001b[1m"
#define ITALIC "\u001b[3m"
#define UNDERLINE "\u001b[4m"
// frames
#define FRAME_WORLD "map"
#define FRAME_INSPECTION "inspection"
#define FRAME_BODY "odom"

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
state mission_state = PASSIVE;

// publishers
ros::Publisher pub_velocity;

// subscribers
ros::Subscriber sub_state;
ros::Subscriber sub_odom;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;
ros::ServiceClient client_land;

// state variables
mavros_msgs::State drone_state;
geometry_msgs::Pose pose;
auto inspection_completed = false;

// transform utilities
tf2_ros::Buffer tf_buffer;

// controller gains
auto kp = 1.f;
auto ki = 1.f;
auto kd = 1.f;

// sequence counters
auto seq_tf = 0;

// waypoints
auto hover_waypoint = Vector3f(0, 0, 2);
auto inspection_waypoints =
    vector<Vector3f>{{2, 2, 0}, {2, -2, 0}, {-2, -2, 0}, {-2, 2, 0}, {2, 2, 0}};
auto waypoint_idx = 0;

// error
auto error_integral = Vector3f(0, 0, 0);
auto error_previous = Vector3f(0, 0, 0);
// previous command - used for logging
geometry_msgs::TwistStamped command_previous;

// time
ros::Time start_time;

//--------------------------------------------------------------------------------------------------
// utility functions
//--------------------------------------------------------------------------------------------------
// transforms `point` from `from_frame` to `to_frame`
// returns the transformed point
geometry_msgs::TransformStamped transform;
auto transform_point(Vector3f point, string from_frame, string to_frame)
    -> std::optional<Vector3f> {
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
    point_from_frame.header.frame_id = FRAME_WORLD;
    // fill point data
    point_from_frame.point.x = hover_waypoint(0);
    point_from_frame.point.y = hover_waypoint(1);
    point_from_frame.point.z = hover_waypoint(2);

    // stamped point message in frame `to_frame`
    auto point_to_frame = geometry_msgs::PointStamped();
    // fill header
    point_to_frame.header.seq = seq_tf++;
    point_to_frame.header.stamp = ros::Time::now();
    point_to_frame.header.frame_id = FRAME_BODY;
    // fill point data by applying transform
    tf2::doTransform(point_from_frame, point_to_frame, transform);

    return Vector3f(point_to_frame.point.x, point_to_frame.point.y, point_to_frame.point.z);
}
//--------------------------------------------------------------------------------------------------
// takes a 3D euclidean position error `error`,
// updates integrat and derivative errors,
// applies PID controller to produce velocity commands
auto command_drone(Vector3f error) -> geometry_msgs::TwistStamped {
    error_integral += error;                         // update integral error
    auto error_derivative = error - error_previous;  // change in error since previous time step

    std::cout << MAGENTA << "error: " << error << std::endl;
    std::cout << MAGENTA << "error_integral: " << error_integral << std::endl;
    std::cout << MAGENTA << "error_derivative: " << error_integral << std::endl;

    // error terms
    auto proportional_term = kp * error;
    auto integral_term = ki * error_integral;
    auto derivative_term = kd * error_derivative;

    std::cout << MAGENTA << "proportional_term: " << proportional_term << std::endl;
    std::cout << MAGENTA << "integral_term: " << integral_term << std::endl;
    std::cout << MAGENTA << "derivative_term: " << derivative_term << std::endl;

    // message to publish
    geometry_msgs::TwistStamped command;
    command.twist.linear.x = kp * error(0) + ki * error_integral(0) + kd * error_derivative(0);
    command.twist.linear.y = kp * error(1) + ki * error_integral(1) + kd * error_derivative(1);
    command.twist.linear.z = kp * error(2) + ki * error_integral(2) + kd * error_derivative(2);

    // current error is the previous error for the next time step
    error_previous = error;

    command_previous = command;

    return command;
}

//--------------------------------------------------------------------------------------------------
// callback functions
//--------------------------------------------------------------------------------------------------
auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { pose = msg->pose.pose; }
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

    ros::Time previous_request_time = ros::Time(0);
    // offboard mode message
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = "OFFBOARD";
    // arm message
    mavros_msgs::CommandBool srv;
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
                // error = transform_point(hover_waypoint, FRAME_WORLD, FRAME_BODY);
                if (auto error = transform_point(hover_waypoint, FRAME_WORLD, FRAME_BODY)) {
                    command = command_drone(*error);
                    pub_velocity.publish(command);

                    // state change
                    if ((*error).norm() < TOLERANCE) {  // within tolerance, change state
                        if (inspection_completed) {     // change to waypoint navigation or land
                            mission_state = LAND;
                        } else {
                            mission_state = INSPECTION;
                        }
                    }
                }
                break;
            case INSPECTION:
                // control
                // error = transform_point(inspection_waypoints[waypoint_idx], FRAME_INSPECTION,
                //                        FRAME_BODY);
                if (auto error = transform_point(inspection_waypoints[waypoint_idx],
                                                 FRAME_INSPECTION, FRAME_BODY)) {
                    command = command_drone(*error);
                    pub_velocity.publish(command);

                    // state change
                    std::cout << "(*error).norm()" << (*error).norm() << std::endl;
                    if ((*error).norm() < TOLERANCE) {  // within tolerance, change to
                        // next waypoint
                        waypoint_idx++;
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
        }
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
        ROS_INFO_STREAM("  state: " << format("%1.2f") %
                                           group(setfill(' '), setw(w), mission_state));
        ROS_INFO_STREAM("  index: " << format("%1.2f") %
                                           group(setfill(' '), setw(w), waypoint_idx + 1));
        ROS_INFO_STREAM("  waypoint:");
        ROS_INFO_STREAM(
            "    x: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx](0)));
        ROS_INFO_STREAM(
            "    y: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx](1)));
        ROS_INFO_STREAM(
            "    z: " << format("%1.5f") %
                             group(setfill(' '), setw(8), inspection_waypoints[waypoint_idx](2)));
        //------------------------------------------------------------------------------------------
        // drone position
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "position:" << RESET);
        ROS_INFO_STREAM("  x: " << format("%1.5f") % group(setfill(' '), setw(8), pose.position.x));
        ROS_INFO_STREAM("  y: " << format("%1.5f") % group(setfill(' '), setw(8), pose.position.y));
        ROS_INFO_STREAM("  z: " << format("%1.5f") % group(setfill(' '), setw(8), pose.position.z));
        //------------------------------------------------------------------------------------------
        // position errors
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "errors:" << RESET);
        ROS_INFO_STREAM("  x: " << format("%1.5f") %
                                       group(setfill(' '), setw(8), error_previous(0)));
        ROS_INFO_STREAM("  y: " << format("%1.5f") %
                                       group(setfill(' '), setw(8), error_previous(1)));
        ROS_INFO_STREAM("  z: " << format("%1.5f") %
                                       group(setfill(' '), setw(8), error_previous(2)));
        //------------------------------------------------------------------------------------------
        // controller outputs
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "controller outputs:" << RESET);
        ROS_INFO_STREAM("  x_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.x));
        ROS_INFO_STREAM("  y_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.y));
        ROS_INFO_STREAM("  z_vel: " << format("%1.5f") % group(setfill(' '), setw(8),
                                                               command_previous.twist.linear.z));

        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
}
