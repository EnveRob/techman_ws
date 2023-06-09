#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <chrono>

// Callback function for force_data subscriber
void forceDataCallback(const geometry_msgs::Vector3 &msg, std::ofstream &file,
                       const std::chrono::high_resolution_clock::time_point &start_time)
{
    // Extract force data
    double force_x = msg.x;
    double force_y = msg.y;
    double force_z = msg.z;

    // Get current timestamp relative to start time
    std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - start_time;

    // Write force data and timestamp to CSV file
    file << elapsed_time.count() << "," << force_x << "," << force_y << "," << force_z << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_data_logger");
    ros::NodeHandle nh;

    // Open the CSV file for writing
    std::ofstream file("force_data.csv");

    // Check if the file is opened successfully
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open the CSV file");
        return -1;
    }

    // Write the header row to the CSV file
    file << "Time(sec),Force_X,Force_Y,Force_Z" << std::endl;

    // Start the timer
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

    // Create a subscriber for force_data topic
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Vector3>("force_data", 1000,
                                                               [&](const boost::shared_ptr<const geometry_msgs::Vector3> &msg)
                                                               {
                                                                   forceDataCallback(*msg, file, start_time);
                                                               });

    // Spin the node
    ros::spin();

    // Close the CSV file
    file.close();

    return 0;
}
