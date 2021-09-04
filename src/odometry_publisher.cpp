#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

float base_width = 0.32f;       // Distance between the left and right wheels of the robot
float ticks_meter = 172.8f;     // Encoder count of motor per meter　[count / m]

float encoder_left = 0.0f;      // encoder left count value
float encoder_right = 0.0f;     // encoder right count value
float d_left = 0.0f;            // calculates micro left travel distance from encoder value
float d_right = 0.0f;           // calculates micro right travel distance from encoder value
float pre_encoder_left = 0.0f;  // save the encoder value left from one step ago
float pre_encoder_right = 0.0f; // save the encoder value right from one step ago

float dist = 0.0f;              // average travel distance of left and right wheels

float x = 0.0f;
float y = 0.0f;
float th = 0.0f;
float dx = 0.0f;
float dy = 0.0f;
float dth = 0.0f;

float dt = 0.0f;
float vx = 0.0f;
float vy = 0.0f;
float vth = 0.0f;

ros::Time current_time;
ros::Time last_time;

ros::Publisher odom_pub;
 
/**
***********************************************************************
* Odometry publish
***********************************************************************
*/
void encoderCallback(const std_msgs::Float32MultiArray& msg){
    tf::TransformBroadcaster odom_broadcaster;
    current_time = ros::Time::now();

    //-------------------------------------------
    // Get the encoder values of the left and right wheels
    //-------------------------------------------
    encoder_left  = msg.data[0];
    encoder_right = msg.data[1];

    //-------------------------------------------
    // Calculates micro travel distance from encoder value
    //-------------------------------------------
    d_left  = (encoder_left - pre_encoder_left) / ticks_meter;   //convert encoder count into m 
    d_right = (encoder_right - pre_encoder_right) / ticks_meter; //convert encoder count into m 
    pre_encoder_left  = encoder_left;
    pre_encoder_right = encoder_right;
    
    //-------------------------------------------
    // Micro travel distance of robot
    //-------------------------------------------
    dist = (d_left + d_right) / 2.0f;      // distance traveled is the average of the two wheels 
    dth = (d_right - d_left) / base_width; // this approximation works (in radians) for small angles    
    dx = dist * cos(th);
    dy = dist * sin(th);

    //-------------------------------------------
    // Robot movement speed
    //-------------------------------------------
    dt = (current_time - last_time).toSec(); //calc velocities
    vx = dist / dt; //v is in base_link frame
    vy = 0;
    vth = dth / dt;

    //-------------------------------------------
    // Position and orientation of the robot
    //-------------------------------------------
    x += (vx * cos(th) - vy * sin(th)) * dt;
    y += (vx * sin(th) + vy * cos(th)) * dt;
    //x += dx;
    //y += dy;
    th += dth;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom; //create nav_msgs::odometry 
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x; //set positions 
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom); //publish odom message

    last_time = current_time;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle nh; 
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    ros::Subscriber encoder_sub = nh.subscribe("/encoder", 50, encoderCallback);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::spin();
    return 0;
}