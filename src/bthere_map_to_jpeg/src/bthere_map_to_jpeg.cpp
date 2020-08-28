#include "map_to_jpeg.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/CompressedImage.h>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "Image window";

namespace bthere_map_to_jpeg
{

    std::string type2str(int type)
    {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth)
        {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
        }

        r += "C";
        r += (chans + '0');

        return r;
    }

    MapToJpeg::MapToJpeg(int argc, char **argv)
    {

        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        sensor_msgs::CompressedImage c_img_msg;

        if (argc != 3)
        {
            ROS_ERROR("Usage: bthere_map_to_jpeg bthere_map_to_jpeg_node <occupancy_grid_topic> <image_topic_namespace>");
            exit(0);
        }

        std::string image_topic, compressed_image_topic;

        occupancy_grid_topic_ = std::string(argv[1]);
        image_topic_namespace_ = std::string(argv[2]);
        image_topic = image_topic_namespace_ + std::string("/image_raw");
        compressed_image_topic = image_topic + std::string("/compressed");

        ROS_INFO("occupancy_grid_topic_ : %s", occupancy_grid_topic_.c_str());
        ROS_INFO("image_topic_ : %s", image_topic.c_str());
        ROS_INFO("compressed_image_topic : %s", compressed_image_topic.c_str());

        subscriber_ = n_.subscribe(
            occupancy_grid_topic_.c_str(),
            1,
            &MapToJpeg::callback,
            this);

        image_pub_ = n_.advertise<sensor_msgs::Image>(image_topic.c_str(), 1);
        compressed_image_pub_ = n_.advertise<sensor_msgs::CompressedImage>(compressed_image_topic.c_str(), 1);

        seq_cnt_ = 0;
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            if (!image_.empty())
            {
                // ROS_INFO("Creating image message");
                std_msgs::Header header;
                header.seq = seq_cnt_++;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_);
                img_bridge.toImageMsg(img_msg);

                // ROS_INFO("Creating compressed image message");
                img_bridge.toCompressedImageMsg(c_img_msg, cv_bridge::Format::JPEG);

                // ROS_INFO("Publishing image message");
                image_pub_.publish(img_msg);
                // ROS_INFO("Publishing compressed image message");
                compressed_image_pub_.publish(c_img_msg);
            }

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    MapToJpeg::~MapToJpeg(void)
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void MapToJpeg::callback(const nav_msgs::OccupancyGrid &msg)
    {

        ROS_INFO("\nReceived an OccupancyGrid message");

        const nav_msgs::OccupancyGrid &occupancy_grid = (nav_msgs::OccupancyGrid &)msg;

        ROS_INFO("Width: %d, Heigth: %d", occupancy_grid.info.width, occupancy_grid.info.height);
        ROS_INFO("Size of data : %ld", occupancy_grid.data.size());
        ROS_INFO("Origin (x: %f, y: %f, z: %f)",
            (occupancy_grid.info.width / 2) + occupancy_grid.info.origin.position.x,
            (occupancy_grid.info.height / 2) + occupancy_grid.info.origin.position.y,
            occupancy_grid.info.origin.position.z
            );

        ROS_INFO("Converting OccupancyGrid to Mat");
        int row, col, value;
        int array_index = 0;
        cv::Mat single_channel_image(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);
        for(std::vector<int8_t>::const_iterator it = occupancy_grid.data.begin(); it != occupancy_grid.data.end(); ++it, array_index++) {
            row = (int)array_index/single_channel_image.cols;
            col = array_index%single_channel_image.cols;
            if ((int)*it == -1) {
                value = 125;
            } else if ((int)*it == 100) {
                value = 0;
            } else if ((int)*it == 0) {
                value = 255;
            } else {
                ROS_WARN("Unsupported value in Occupancy Grid");
                value == 125;
            }
            single_channel_image.at<uchar>(row, col) = (uchar)value;
        }
        ROS_INFO("Done!");

        // cv::Mat roi = getROI(mapped_og);

        // ROS_INFO("Writing image to file");
        // cv::imwrite("image.jpeg", roi);

        // ROS_INFO("Showing the image");
        // cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
        // cv::imshow(OPENCV_WINDOW, roi);
        // cv::waitKey(1000);

        // #ifdef false
        // ROS_INFO("Converting OccupancyGrid to GridMap");
        // grid_map::GridMap map;
        // // if (grid_map::GridMapRosConverter::fromOccupancyGrid((nav_msgs::OccupancyGrid &)msg, "elevation", map))
        // if (grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "elevation", map))
        // {
        //     ROS_INFO("Success!");
        //     std::cout << "GridMap size : " << map.getSize() << std::endl;
        // }
        // else
        // {
        //     ROS_WARN("Failure");
        // }

        // ROS_INFO("Converting GridMap to Image");
        // cv::Mat single_channel_image;
        // cv::Mat image;
        // if (grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, 100, 0, single_channel_image))
        // {
        //     ROS_INFO("Success!");
        //     cv::Mat roi = getROI(single_channel_image);
        //     cv::cvtColor(roi, image, CV_GRAY2BGR);
        //     ROS_INFO("Number of channels: %d", image.channels());
        //     ROS_INFO("Columns: %d, Rows: %d", image.cols, image.rows);
        //     // std::cout << "image: " << single_channel_image.row(0) << std::endl;
        // }
        // else
        // {
        //     ROS_WARN("Failure");
        // }

        if (single_channel_image.empty())
        {
            ROS_WARN("Failure: image empty");
            return;
        }
        else
        {
            cv::Mat image, flipped_image;
            
            // Get the region of interest
            cv::Mat roi = getROI(single_channel_image);

            // Convert single_channel_image to 3 channels
            cv::cvtColor(roi, image, CV_GRAY2BGR);

            // Flip image about the horizontal axis
            cv::flip(image, flipped_image, 0);

            // ROS_INFO("Writing image to file");
            cv::imwrite("image.jpeg", flipped_image);
            flipped_image.copyTo(image_);

            ROS_INFO("Showing the image");
            cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
            cv::imshow(OPENCV_WINDOW, flipped_image);
            cv::waitKey(1000);
        }
        // #endif
    }

    cv::Mat MapToJpeg::getROI(cv::Mat input)
    {
        int threshold = 250;
        int offset = 100;
        int min_count = 2;
        int scan_offset = 10;

        int top, bottom, left, right;
        bool found;
        int count;

        std::string ty = type2str(input.type());
        printf("Matrix: %s %dx%d \n", ty.c_str(), input.cols, input.rows);

        if (input.empty()) {
            ROS_WARN("Image is empty!");
        }

        // Get the first row that contains data
        found = false;
        count = 0;
        for (int row = scan_offset; row < input.rows; row++)
        {
            for (int col = scan_offset; col < input.cols; col++)
            {
                // std::cout << "row: " << row << ", col: " << col << std::endl;
                if (input.at<uchar>(row, col) > threshold)
                {
                    std::cout << "top: " << row << ", " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                    if (count++ >= min_count)
                    {
                        top = row;
                        found = true;
                        break;
                    }
                }
            }
            if (found)
                break;
        }
        // std::cout << "Finished finding first row!" << std::endl;

        // Get the first column that contains data
        found = false;
        count = 0;
        for (int col = scan_offset; col < input.cols; col++)
        {
            for (int row = scan_offset; row < input.rows; row++)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    // std::cout << "left: " << row << ", " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                    if (count++ >= min_count)
                    {
                        left = col;
                        found = true;
                        break;
                    }
                }
            }
            if (found)
                break;
        }
        // std::cout << "Finished finding first colum!" << std::endl;

        // Get the last row that contains data
        found = false;
        count = 0;
        for (int row = input.rows - scan_offset; row >= 0; row--)
        {
            for (int col = input.cols - scan_offset; col >= 0; col--)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    // std::cout << "bottom: " << row << ", " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                    if (count++ >= min_count)
                    {
                        bottom = row;
                        found = true;
                        break;
                    }
                }
            }
            if (found)
                break;
        }
        // std::cout << "Finished finding last row!" << std::endl;

        // Get the last column that contains data
        found = false;
        count = 0;
        for (int col = input.cols - scan_offset; col >= 0; col--)
        {
            for (int row = input.rows - scan_offset; row >= 0; row--)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    // std::cout << "right: " << row << ", " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                    if (count++ >= min_count)
                    {
                        right = col;
                        found = true;
                        break;
                    }
                }
            }
            if (found)
                break;
        }
        // std::cout << "Finished finding last column!" << std::endl;

        top = (top - offset) > 0 ? top - offset : top;
        left = (left - offset) > 0 ? left - offset : left;
        bottom = (bottom + offset) > input.rows ? bottom : bottom + offset;
        right = (right + offset) > input.cols ? right : right + offset;

        input.row(top).setTo(cv::Scalar(255));
        input.col(left).setTo(cv::Scalar(255));
        input.row(bottom).setTo(cv::Scalar(255));
        input.col(right).setTo(cv::Scalar(255));
        ROS_INFO("ROI: %d, %d, %d, %d", top, left, bottom, right);

        int width = right - left;
        // ROS_INFO("width : %d", width);
        int height = bottom - top;
        // ROS_INFO("height : %d", height);
        n_.setParam(image_topic_namespace_ + std::string("/image_width"), width);
        n_.setParam(image_topic_namespace_ + std::string("/image_height"), height);

        // ROS_INFO("Cropping to RECT(%d, %d, %d, %d)", left, top, width, height);
        cv::Rect R(left, top, width, height);
        return input(R);
    }

} // namespace bthere_map_to_jpeg