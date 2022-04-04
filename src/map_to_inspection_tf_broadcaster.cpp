#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

auto crane_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) -> void {
    static auto map_to_inspection_tf_broadcaster = tf2_ros::TransformBroadcaster{};
    auto tf = geometry_msgs::TransformStamped{};

    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "inspection";

    const auto& position = msg->pose.position;
    tf.transform.translation.x = position.x;
    tf.transform.translation.y = position.y;
    tf.transform.translation.z = position.z;

    const auto& quat = msg->pose.orientation;
    tf.transform.rotation.x = quat.x;
    tf.transform.rotation.y = quat.y;
    tf.transform.rotation.z = quat.z;
    tf.transform.rotation.w = quat.w;

    map_to_inspection_tf_broadcaster.sendTransform(tf);
}

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "map_to_inspection_tf_broadcaster");
    auto nh = ros::NodeHandle();
    auto crane_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/crane_pose", 10, crane_pose_cb);
    auto loop_rate = ros::Rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
