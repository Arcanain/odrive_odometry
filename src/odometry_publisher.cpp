#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// HajimeCart parameter
const double wheel_radius           = 0.095f;   
const double base_width             = 0.340f; 
const float wheel_to_wheel_distance = 0.310f;

// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 100;

// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value_left  = 0;
int encoder_value_right = 0;

// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle_left  = 0.0f;
float rotation_angle_right = 0.0f;

// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle_left = 0.0f;
float pre_rotation_angle_right = 0.0f;

// 現時刻でのロータリーエンコーダの角速度を保持
float rotation_angular_velocity_left  = 0.0f;
float rotation_angular_velocity_right = 0.0f;

// 両車輪の移動速度
float left_vel  = 0.0f;
float right_vel = 0.0f;

// ロボットの並進速度
float linear_vel = 0.0f;
// ロボットの回転速度
float angular_vel = 0.0f;

// ロボットの位置と姿勢
float x     = 0.0f;
float y     = 0.0f;
float theta = 0.0f;

// first flag
int flag_firsttime = 1;

// ros time
ros::Publisher odom_pub;
ros::Publisher rotation_angular_velocity_pub;
ros::Time current_time;
ros::Time last_time;

std_msgs::Float32MultiArray rotation_angular_velocity_data;

void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    ros::Subscriber encoder_sub = nh.subscribe("encoder", 10, encoderCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    rotation_angular_velocity_pub  = nh.advertise<std_msgs::Float32MultiArray>("rotation_angular_velocity", 10);
    ros::spin(); // check for incoming messages

    return 0;
}

void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    static tf::TransformBroadcaster odom_broadcaster;

    encoder_value_left  = msg->data[0];
    encoder_value_right = msg->data[1];

    if( flag_firsttime )
    {
        flag_firsttime = 0;

     	last_time = ros::Time::now();

        return;
    }

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    /**
     ***********************************************************************
     * Odometry
     ***********************************************************************
    */
    //-------------------------------------------
    // タイヤの回転角
    //-------------------------------------------
    rotation_angle_left  = encoder_value_left * (2 * 3.14f)/(4 * rotary_encoder_resolution);
    rotation_angle_right = encoder_value_right * (2 * 3.14f)/(4 * rotary_encoder_resolution);

    //-------------------------------------------
    // タイヤの回転速度
    //-------------------------------------------
    rotation_angular_velocity_left  = (rotation_angle_left - pre_rotation_angle_left) / dt;
    rotation_angular_velocity_right = (rotation_angle_right - pre_rotation_angle_right) / dt;

    rotation_angular_velocity_data.data[0] = rotation_angular_velocity_left;
    rotation_angular_velocity_data.data[1] = rotation_angular_velocity_right; 
    rotation_angular_velocity_pub.publish(rotation_angular_velocity_data);

    //-------------------------------------------
    // ロボットの移動速度
    //-------------------------------------------
    left_vel    = rotation_angular_velocity_left * wheel_radius;
    right_vel   = rotation_angular_velocity_right * wheel_radius;
    linear_vel  = (right_vel + left_vel) / 2.0f;
    angular_vel = (right_vel - left_vel) / wheel_to_wheel_distance;

    //-------------------------------------------
    // ロボットの位置
    //-------------------------------------------
    x     = x + linear_vel * dt * cos(theta + angular_vel / 2.0f);
    y     = y + linear_vel * dt * sin(theta + angular_vel / 2.0f);
    theta = theta + angular_vel * dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = angular_vel;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}