#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

float gx = 0.0, gy = 0.0, gz = 0.0;
float ax = 0.0, ay = 0.0, az = 0.0;

tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
double dt = 0.01;

void GyroCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Access the x, y, and z components of the vector
    gx = msg->x;
    gy = msg->y;
    gz = msg->z;

    // Print the values to the console or process them as needed
}

void AccCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Access the x, y, and z components of the vector
    ax = msg->x;
    ay = msg->y;
    az = msg->z;

    // Print the values to the console or process them as needed
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "Imu");

    // Create a node handle
    ros::NodeHandle nh;

    // Subscribe to the Vector3 topic
    ros::Subscriber gyro_sub = nh.subscribe("/gyro_data", 10, GyroCallback);

    ros::Subscriber acc_sub = nh.subscribe("/acc_data", 10, AccCallback);

    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/Imu", 10);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(100);

     while (ros::ok())
    {
        // Create a message object
        sensor_msgs::Imu imu;
        
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";

        // Compute orientation based on accelerometer data
        tf::Vector3 accel(ax, ay, az);
        tf::Vector3 ref(0, 0, 1);
        tf::Vector3 acceln = accel.normalized();
        tf::Vector3 axis = acceln.cross(ref);
        double angle = std::acos(acceln.dot(ref));
        tf::Quaternion orientation = tf::Quaternion(axis, angle);

        // Set the orientation in the IMU message
        imu.orientation.x = orientation.x();
        imu.orientation.y = orientation.y();
        imu.orientation.z = orientation.z();
        imu.orientation.w = orientation.w();

        // Fill in the message data
        imu.angular_velocity.x = gx;
        imu.angular_velocity.y = gy;
        imu.angular_velocity.z = gz;

        imu.linear_acceleration.x = ax;
        imu.linear_acceleration.y = ay;
        imu.linear_acceleration.z = az;
 
        // Publish the message
        pub_imu.publish(imu);

        // Broadcast the transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "plane"; // Matching the Python version's frame
        transformStamped.child_frame_id = "imu_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = imu.orientation.x;
        transformStamped.transform.rotation.y = imu.orientation.y;
        transformStamped.transform.rotation.z = imu.orientation.z;
        transformStamped.transform.rotation.w = imu.orientation.w;

        odom_broadcaster.sendTransform(transformStamped);
   
        // Spin once to allow callbacks
        ros::spinOnce();

        // Sleep for the remainder of the loop cycle
        loop_rate.sleep();
    }


    // Spin to process incoming messages
    ros::spin();

    return 0;
}


