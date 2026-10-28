#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "first_project/sector_times.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

double LAT_A, LONG_A, LAT_B, LONG_B, LAT_C, LONG_C;

int prev_sector = -1;
ros::Time sector_start_time(0);
int num_messages = 0;
float avg_velocities = 0;

ros::Publisher sector_publisher;

int getSector(double latitude, double longitude) {
    double res1, res2;
    
    if(latitude == 0 || longitude == 0)
        return 0;

    // Line AB
    res1 = ((LONG_A - LONG_B) / (LAT_A - LAT_B)) * (latitude - LAT_A) + LONG_A - longitude;

    // Perpendicular line through C
    res2 = -1 * ((LAT_A - LAT_B) / (LONG_A - LONG_B)) * (latitude - LAT_C) + LONG_C - longitude;

    if (res1 >= 0) {
        return 1; // Red, Sector 1
    } else if (res2 >= 0) {
        return 3; // Yellow, Sector 3
    } else {
        return 2; // Blue, Sector 2
    }
}

void callback(const geometry_msgs::PointStampedConstPtr& speedsteer_msg, const sensor_msgs::NavSatFixConstPtr& gps_msg){

    int curr_sector = getSector(gps_msg->latitude, gps_msg->longitude);
    
    if(curr_sector == 0)
        curr_sector = prev_sector;

    if(curr_sector != prev_sector){
        prev_sector = curr_sector;
        sector_start_time = speedsteer_msg->header.stamp;
        num_messages = 0;
        avg_velocities = 0;
    }
    
    double curr_sector_time = (speedsteer_msg->header.stamp - sector_start_time).toSec();
    
    avg_velocities = (avg_velocities * num_messages + speedsteer_msg->point.y) / (num_messages + 1);
    num_messages++;
    
    first_project::sector_times custom_msg;
    
    custom_msg.current_sector = curr_sector;
    custom_msg.current_sector_time = curr_sector_time;
    custom_msg.current_sector_mean_speed = avg_velocities;
    
    sector_publisher.publish(custom_msg);
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sector_times");

  ros::NodeHandle n;
  
  ros::NodeHandle private_nh("~");
  
  private_nh.param("LAT_A", LAT_A, 45.616398);
  private_nh.param("LONG_A", LONG_A, 9.280886);
  private_nh.param("LAT_B", LAT_B, 45.630021);
  private_nh.param("LONG_B", LONG_B, 9.288187);
  private_nh.param("LAT_C", LAT_C, 45.623928);
  private_nh.param("LONG_C", LONG_C, 9.287867);
  
  sector_publisher = n.advertise<first_project::sector_times>("/sector_times", 10);

  message_filters::Subscriber<geometry_msgs::PointStamped> sub1(n, "/speedsteer", 100);
  message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(n, "/swiftnav/front/gps_pose", 100);
  
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
