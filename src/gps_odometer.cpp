#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <deque>

class GpsOdometer {
public:
    GpsOdometer() : nh_("~"), initialized_(false), has_first_gps_(false) {
        nh_.param("lat_r", lat_r_, 0.0);
        nh_.param("lon_r", lon_r_, 0.0);
        nh_.param("alt_r", alt_r_, 0.0);
        nh_.param("heading_window_size", heading_window_size_, 5);
        nh_.param("max_acceptable_distance", max_acceptable_distance_, 1000.0); // meters
        nh_.param("max_acceptable_velocity", max_acceptable_velocity_, 500.0); // m/s

        // Convert reference point to ECEF
        llhToEcef(lat_r_, lon_r_, alt_r_, x_r_, y_r_, z_r_);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10);
        gps_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 10, &GpsOdometer::gpsCallback, this);

        ROS_INFO("GPS odometer initialized with reference point: lat=%.9f, lon=%.9f, alt=%.3f", 
                lat_r_, lon_r_, alt_r_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber gps_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double lat_r_, lon_r_, alt_r_;
    double x_r_, y_r_, z_r_;
    bool initialized_;
    int heading_window_size_;
    double max_acceptable_distance_;
    double max_acceptable_velocity_;

    // Velocity estimation
    double last_velocity_x_{0.0};
    double last_velocity_y_{0.0};
    ros::Time last_good_gps_time_;
    bool estimating_{false};

    //to store the first gps value
    double first_lat_, first_lon_, first_alt_;
    bool has_first_gps_;

    //to calculate angular velocity
    double prev_heading_{0.0};
    double last_angular_velocity_{0.0};
    
    std::deque<double> heading_history_;
    ros::Time prev_time_;
    double prev_x_, prev_y_, prev_z_{0.0};

    void llhToEcef(double lat, double lon, double alt, double &x, double &y, double &z) {
        // WGS84 parameters
        const double a = 6378137.0;              // semi-major axis
        const double e_sq = 6.69437999014e-3;     // first eccentricity squared

        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

        x = (N + alt) * cos(lat_rad) * cos(lon_rad);
        y = (N + alt) * cos(lat_rad) * sin(lon_rad);
        z = (N * (1 - e_sq) + alt) * sin(lat_rad);
    }

    void ecefToEnu(double x, double y, double z, double &enu_x, double &enu_y, double &enu_z) {
        // Convert from ECEF to ENU coordinates
        double dx = x - x_r_;
        double dy = y - y_r_;
        double dz = z - z_r_;

        double phi = lat_r_ * M_PI / 180.0;
        double lambda = lon_r_ * M_PI / 180.0;

        enu_x = -sin(lambda) * dx + cos(lambda) * dy;
        enu_y = -sin(phi) * cos(lambda) * dx - sin(phi) * sin(lambda) * dy + cos(phi) * dz;
        enu_z = cos(phi) * cos(lambda) * dx + cos(phi) * sin(lambda) * dy + sin(phi) * dz;
    }

    double calculateHeading(double x, double y, const ros::Time& current_time) {
        if (!initialized_) {
            return 0.0;
        }

        double dt = (current_time - prev_time_).toSec();
        if (dt <= 0) {
            dt = 0.01;  // avoid division by zero
        }

        double dx = x - prev_x_;
        double dy = y - prev_y_;
        
        // Only calculate heading if we've moved significantly
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < 0.1) {  // 10 cm threshold
            if (!heading_history_.empty()) {
                return heading_history_.back();
            }
            return 0.0;
        }

        double current_heading = atan2(dy, dx);
        
        // Apply moving average filter
        heading_history_.push_back(current_heading);
        if (heading_history_.size() > heading_window_size_) {
            heading_history_.pop_front();
        }

        double sum_sin = 0.0;
        double sum_cos = 0.0;
        for (double h : heading_history_) {
            sum_sin += sin(h);
            sum_cos += cos(h);
        }

        return atan2(sum_sin, sum_cos);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Skip if no fix
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) { 
            ROS_WARN_THROTTLE(1.0, "No GPS fix. Skipping update.");
            return;
        }

        // The front GPS sometimes reports lat = lon = 0 while still claiming a fix
        bool dropout = (msg->latitude == 0.0 && msg->longitude == 0.0);
        if (dropout && !initialized_) {
            return;
        }

        // Store the first GPS reading
        if (!has_first_gps_) {
            first_lat_ = msg->latitude;
            first_lon_ = msg->longitude;
            first_alt_ = msg->altitude;
            has_first_gps_ = true;
            
            ROS_INFO("First GPS position recorded: lat=%.9f, lon=%.9f, alt=%.3f",
                    first_lat_, first_lon_, first_alt_);
        }

        // Convert current position to ECEF
        double x, y, z;
        llhToEcef(msg->latitude, msg->longitude, msg->altitude, x, y, z);

        // Convert to ENU coordinates
        double enu_x, enu_y, enu_z;
        ecefToEnu(x, y, z, enu_x, enu_y, enu_z);

        // Use the measurement time, not wall-clock time, so dt stays correct at any bag play rate
        ros::Time current_time = msg->header.stamp;
        bool use_estimation = dropout;

        // Check if we should use estimation instead of raw GPS
        if (initialized_ && !dropout) {
            double dt = (current_time - prev_time_).toSec();
            if (dt > 0) {
                double dx = enu_x - prev_x_;
                double dy = enu_y - prev_y_;
                double distance = sqrt(dx*dx + dy*dy);
                double velocity = distance / dt;

                if (distance > max_acceptable_distance_ || velocity > max_acceptable_velocity_) {
                    use_estimation = true;
                    ROS_WARN("GPS jump detected! Distance: %.2fm, Velocity: %.2fm/s. Using estimation.", 
                            distance, velocity);
                }
            }
        }

        // If GPS data is bad, estimate position based on last good velocity
        if (use_estimation) {
            double dt = (current_time - prev_time_).toSec();
            enu_x = prev_x_ + last_velocity_x_ * dt;
            enu_y = prev_y_ + last_velocity_y_ * dt;
            enu_z = prev_z_;
        } else {
            // Update last good GPS time
            last_good_gps_time_ = current_time;
        }

        // On the first good fix after estimating, the position snaps back to the real one;
        // keep the previous velocity instead of turning that correction into a speed spike
        bool recovering = estimating_ && !use_estimation;
        estimating_ = use_estimation;

        // Calculate heading
        double heading = calculateHeading(enu_x, enu_y, current_time);

        // Update velocity estimation
        if (initialized_ && !recovering) {
            double dt = (current_time - prev_time_).toSec();
            if (dt > 0) {
                last_velocity_x_ = (enu_x - prev_x_) / dt;
                last_velocity_y_ = (enu_y - prev_y_) / dt;
            }
        }
        // Calculate angular velocity
        if (initialized_) {
            double dt = (current_time - prev_time_).toSec();
            if (dt > 0) {
                // Handle angle wrapping
                double heading_change = heading - prev_heading_;
                while (heading_change > M_PI) heading_change -= 2*M_PI;
                while (heading_change < -M_PI) heading_change += 2*M_PI;
                
                last_angular_velocity_ = heading_change / dt;
            }
        }

        // Create odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";
        odom.child_frame_id = "odom-gps";

        odom.pose.pose.position.x = enu_x;
        odom.pose.pose.position.y = enu_y;
        odom.pose.pose.position.z = enu_z;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = last_angular_velocity_;

        // Set orientation from heading
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Set velocity
        odom.twist.twist.linear.x = last_velocity_x_;
        odom.twist.twist.linear.y = last_velocity_y_;

        odom_pub_.publish(odom);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = odom.header.stamp;
        transform.header.frame_id = "world";
        transform.child_frame_id = "odom-gps";
        transform.transform.translation.x = enu_x;
        transform.transform.translation.y = enu_y;
        transform.transform.translation.z = enu_z;
        transform.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(transform);
    

        // Update previous values
        prev_heading_ = heading;
        prev_x_ = enu_x;
        prev_y_ = enu_y;
        prev_z_ = enu_z;
        prev_time_ = current_time;
        initialized_ = true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer gpsOdometer;
    ros::spin();
    return 0;
}
