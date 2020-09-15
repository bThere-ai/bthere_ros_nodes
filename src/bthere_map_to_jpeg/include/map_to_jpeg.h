#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>

namespace bthere_map_to_jpeg
{
    /**
    @class bthere_map_to_jpeg
    @brief Subscribes to a OccupancyGrid topic, converts it to jpeg and publishes it on a topic 
    **/

    class MapToJpeg
    {
    private:
        // Subscriber for the map topic
        ros::Subscriber subscriber_;

        // Publisher for the image
        ros::Publisher image_pub_;

        // Publisher for the jpeg compressed image
        ros::Publisher compressed_image_pub_;

        // ROS node handle
        ros::NodeHandle n_;

        // The occupancy grid topic
        std::string occupancy_grid_topic_;

        // The image topic
        std::string image_topic_namespace_;

        // Sequence counter
        int seq_cnt_;

        // Buffer & TransformListener for receiving the robot transform
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener *tfListener_;

    public:
    // Map origin
        float map_origin_x;
        float map_origin_y;

        // Occupancy grid resolution
        float og_res_;

        // Region of interest
        cv::Rect roi_;

        // Image to publish
        cv::Mat image_;

        /**
        @brief Default constructor
        @param argc [int] Number of input arguments
        @param argv [char **] Input arguments
        @return void
        **/
        MapToJpeg(int argc, char **argv);

        /**
        @brief Default destructor
        @return void
        **/
        ~MapToJpeg(void);

        

        cv::Mat convertOccupancyGridToMat(const nav_msgs::OccupancyGrid &occupancy_grid);

        /**
        @brief Extract the region of interest from the input
        @param input [cv::Mat] An openCV matrix representing an occupancy grid as an image
        @return [cv::Rect] An openCV Rect that specifies the region of interest
        **/ 
        cv::Rect getROI(cv::Mat input);

    private:
        /**
        @brief Callback for the subscriber
        @param msg [const nav_msgs::OccupancyGrid&] The new occupancy grid message
        @return void
        **/
        void callback(const nav_msgs::OccupancyGrid& msg);

        static void processOccupancyGrid(MapToJpeg* instance, const nav_msgs::OccupancyGrid &grid);

        cv::Mat overlayRobotLocation(cv::Mat input, cv::Rect roi = cv::Rect());
    };
} // namespace bthere_map_to_jpeg