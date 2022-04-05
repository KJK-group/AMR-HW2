#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>

#include "boost/format.hpp"

// escape codes
#define MAGENTA "\u001b[35m"
#define GREEN "\u001b[32m"
#define RESET "\u001b[0m"
#define BOLD "\u001b[1m"
#define ITALIC "\u001b[3m"
#define UNDERLINE "\u001b[4m"

// publishers
ros::Publisher pub_velocity;

// subscribers
ros::Subscriber sub_state;
ros::Subscriber sub_odom;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;

// state variables
mavros_msgs::State state;

// transform utilities
tf2_ros::Buffer tf_buffer;
// frames
auto frame_world = "map";
auto frame_body = "inspection";

// controller gains
auto kp = 1.f;
auto kd = 1.f;
auto ki = 1.f;

// callback functions
auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // current position
    auto pos = msg->pose.pose.position;
    // current yaw
    auto yaw = tf2::getYaw(msg->pose.pose.orientation);
}

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { state = *msg; }

auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_test_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    //----------------------------------------------------------------------------------------------
    // transform utilities
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //----------------------------------------------------------------------------------------------
    // pass potential controller gain arguments
    if (argc > 1) kp = stof(argv[1]);
    if (argc > 2) kd = stof(argv[2]);
    if (argc > 3) ki = stof(argv[3]);

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
    // wait for FCU connection
    while (ros::ok() && !state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
    // arm the drone
    if (!state.armed) {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (client_arm.call(srv)) {
            ROS_INFO("throttle armed: success");
        } else {
            ROS_INFO("throttle armed: fail");
        }
    }
    //----------------------------------------------------------------------------------------------
    // set drone mode to OFFBOARD
    if (state.mode != "OFFBOARD") {
        mavros_msgs::SetMode mode_msg;
        mode_msg.request.custom_mode = "OFFBOARD";

        if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
            ROS_INFO("mode set: OFFBOARD");
        } else {
            ROS_INFO("mode set: fail");
        }
    }
    //----------------------------------------------------------------------------------------------
    // ROS spin
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
}
