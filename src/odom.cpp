#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

const double steering_factor = 32.0;
const double vehicle_length = 1.765;
double x = 0.0, y = 0.0, theta = 0.0;
ros::Time prev_time(0);
ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster;

void Callback(const geometry_msgs::PointStamped::ConstPtr& msg){

    ros::Time curr_time = msg->header.stamp;
    if(prev_time.toSec() == 0){prev_time = curr_time; return;}
    
    double dt = (curr_time - prev_time).toSec();
    double steer_angle = msg->point.x;
    if(steer_angle > -10 && steer_angle < 10){
    	steer_angle = 0;
    }
    
    //kmph to mps
    double vel = msg->point.y / 3.6;
    //steer angle to wheel angle to wheel angle rad
    double wheel_angle_rad = (steer_angle / steering_factor) * 3.14159 / 180.0;
    
    x += vel * cos(theta) * dt;
    y += vel * sin(theta) * dt;

    theta += vel * tan(wheel_angle_rad) / vehicle_length * dt;
    
    
    //tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = curr_time;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "odom-vehicle";

    //base change to match gps more closely
    odom_trans.transform.translation.x = -y;
    odom_trans.transform.translation.y = x;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster->sendTransform(odom_trans);
    
    //topic
    
    nav_msgs::Odometry odom;
    odom.header.stamp = curr_time;
    odom.header.frame_id = "world";
    
    //base change
    odom.pose.pose.position.x = -y;
    odom.pose.pose.position.y = x;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    odom.child_frame_id = "odom-vehicle";
    
    //base change
    odom.twist.twist.linear.x = vel * -sin(theta);
    odom.twist.twist.linear.y = vel * cos(theta);
    odom.twist.twist.angular.z = vel * tan(wheel_angle_rad) / vehicle_length;
    
    odom_pub.publish(odom);
    
    prev_time = curr_time;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/speedsteer", 1000, Callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    odom_broadcaster = new tf::TransformBroadcaster();
    
    ros::spin();
    delete odom_broadcaster;
    
    return 0;
 }

