#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
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

        // Image to publish
        cv::Mat image_;

    public:
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

    private:
        /**
        @brief Callback for the subscriber
        @param msg [const nav_msgs::OccupancyGrid&] The new occupancy grid message
        @return void
        **/
        void callback(const nav_msgs::OccupancyGrid& msg);

        cv::Mat getROI(cv::Mat input);
    };
} // namespace bthere_map_to_jpeg