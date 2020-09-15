#include "map_to_jpeg.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

static const std::string OPENCV_WINDOW = "Image window";
static const float RADIUS_OF_ROBOT = 0.2;


namespace bthere_map_to_jpeg
{
    /**
     * Helper function for obtaining the string format of a cv::Mat element type
     * @param type type of a matrix element
     */
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


    /**
     * ROS node converting a nav_msgs/OccupancyGrid.msg topic into a sensor_msgs/Image.msg topic
     */
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

        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

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
                cv::Mat flipped_image, image_with_overlay, flipped_image_with_overlay;
                
                // Overlay the robot location on the occupancy grid image
                image_with_overlay = overlayRobotLocation(image_, roi_);

                // Before publishing the image flip it about the horizontal axis
                cv::flip(image_with_overlay, flipped_image_with_overlay, 0);

                // ROS_INFO("Creating image message");
                std_msgs::Header header;
                header.seq = seq_cnt_++;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, flipped_image_with_overlay);
                img_bridge.toImageMsg(img_msg);

                // ROS_INFO("Creating compressed image message");
                img_bridge.toCompressedImageMsg(c_img_msg, cv_bridge::Format::JPEG);

                // ROS_INFO("Publishing image message");
                image_pub_.publish(img_msg);

                // ROS_INFO("Publishing compressed image message");
                compressed_image_pub_.publish(c_img_msg);

                // If running on robot with a GUI, then display the image
                cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
                cv::imshow(OPENCV_WINDOW, flipped_image_with_overlay);
                cv::waitKey(100);
            }

            ros::spinOnce();

            loop_rate.sleep();
        }
    }


    /**
     * Class destructor
     */
    MapToJpeg::~MapToJpeg(void)
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }


    /**
     * Coverts an occupany grid to an opencv image
     * @param grid A reference to a nav_msgs::OccupancyGrid
     * 
     * @return  An opencv greyscale image where the occupancy grid values (-1, 0, 100) have been converted to a
     *          grey, white, and black respectively.
     */
    cv::Mat MapToJpeg::convertOccupancyGridToMat(const nav_msgs::OccupancyGrid &grid)
    {
        int row, col, value;
        int array_index = 0;
        cv::Mat image(grid.info.width, grid.info.height, CV_8UC1);

        for(std::vector<int8_t>::const_iterator it = grid.data.begin(); it != grid.data.end(); ++it, array_index++) {
            row = (int)array_index/image.cols;
            col = array_index%image.cols;
            if ((int)*it == -1) {
                value = 125;    // grey
            } else if ((int)*it == 100) {
                value = 0;      // black
            } else if ((int)*it == 0) {
                value = 255;    // white
            } else {
                ROS_WARN("Unsupported value in Occupancy Grid");
                value == 125;
            }
            image.at<uchar>(row, col) = (uchar)value;
        }
        return image;
    }


    /**
     * Callback function that is called each time a new map is published
     * @param msg   A message containing the occupancy grid information
     */
    void MapToJpeg::callback(const nav_msgs::OccupancyGrid &msg)
    {
        printf("\n");
        ROS_INFO("Received an OccupancyGrid message");

        // Since processing the occupancy grid information can take some time, let's do this in another thread
        new std::thread(processOccupancyGrid, this, msg);
    }


    /**
     * Processes the incoming occupancy grid data by converting it to an opencv image and cropping the image to
     * an region of interest.
     * @param instance  The class instance associated with the callback
     * @param grid      The occupancy grid data
     */
    void MapToJpeg::processOccupancyGrid(MapToJpeg *instance, const nav_msgs::OccupancyGrid &grid)
    {
        instance->map_origin_x = grid.info.width / 2;
        instance->map_origin_y = grid.info.height / 2;
        instance->og_res_ = grid.info.resolution;

        ROS_INFO("Width: %d, Heigth: %d", grid.info.width, grid.info.height);
        ROS_INFO("Size of data : %ld", grid.data.size());
        ROS_INFO("Origin (x: %f, y: %f, z: %f)",
            instance->map_origin_x,
            instance->map_origin_y,
            grid.info.origin.position.z
            );
        ROS_INFO("Resolution : %f", instance->og_res_);

        // Convert the occupancy grid to an OpenCV matrix
        ROS_INFO("Converting OccupancyGrid to Mat");
        cv::Mat single_channel_image = instance->convertOccupancyGridToMat(grid);
        ROS_INFO("Done!");

        if (single_channel_image.empty())
        {
            ROS_WARN("Failure: image empty");
            return;
        }
        else
        {
            cv::Mat multi_channel_image;
            
            // Get the region of interest
            instance->roi_ = instance->getROI(single_channel_image);

            // Update the image parameters
            instance->n_.setParam(instance->image_topic_namespace_ + std::string("/image_width"), instance->roi_.width);
            instance->n_.setParam(instance->image_topic_namespace_ + std::string("/image_height"), instance->roi_.height);

            // Convert single_channel_image to 3 channels
            cv::cvtColor(single_channel_image, multi_channel_image, CV_GRAY2BGR);

            // Crop the image to the ROI
            cv::Mat image_to_display = multi_channel_image(instance->roi_);

            // Make the image available to the main loop
            image_to_display.copyTo(instance->image_);
        }
    }


    /**
     * Get a bounding rectangle that represents the region of interest within a single channel image.
     * Since the occupancy grid is mainly filled with -1 values representing 'unknown' occupancy, getROI
     * will look for cells that contain other values to determine a bounding rectangle around the region.
     * @param input The image on which to find the region of interest
     * 
     * @return  A bounding rectangle representing the region of interest
     */
    cv::Rect MapToJpeg::getROI(cv::Mat input)
    {
        int threshold = 250;
        int offset = 10;
        int min_count = 2;
        int scan_offset = 10;

        int top, bottom, left, right;
        bool found;
        int count;

        std::string ty = type2str(input.type());
        ROS_INFO("Matrix: %s %dx%d", ty.c_str(), input.cols, input.rows);

        if (input.empty()) {
            ROS_WARN("Image is empty!");
        }

        // Get the first row (top) that contains data
        found = false;
        count = 0;
        int row, col;
        for (row = scan_offset; row < input.rows; row++)
        {
            for (col = scan_offset; col < input.cols; col++)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    if (count++ >= min_count)
                    {
                        top = row;
                        found = true;
                        break;
                    }
                }
            }
            if (found) {
                // std::cout << "top: " << row << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                break;
            }
        }
        // std::cout << "Finished finding first row!" << std::endl;

        // Get the first column (left) that contains data
        found = false;
        count = 0;
        for (col = scan_offset; col < input.cols; col++)
        {
            for (row = scan_offset; row < input.rows; row++)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    if (count++ >= min_count)
                    {
                        left = col;
                        found = true;
                        break;
                    }
                }
            }
            if (found) {
                // std::cout << "left: " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                break;
            }
        }
        // std::cout << "Finished finding first colum!" << std::endl;

        // Get the last row (bottom) that contains data
        found = false;
        count = 0;
        for (row = input.rows - scan_offset; row >= 0; row--)
        {
            for (col = input.cols - scan_offset; col >= 0; col--)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    if (count++ >= min_count)
                    {
                        bottom = row;
                        found = true;
                        break;
                    }
                }
            }
            if (found) {
                // std::cout << "bottom: " << row << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                break;
            }
        }
        // std::cout << "Finished finding last row!" << std::endl;

        // Get the last column (right) that contains data
        found = false;
        count = 0;
        for (col = input.cols - scan_offset; col >= 0; col--)
        {
            for (row = input.rows - scan_offset; row >= 0; row--)
            {
                if (input.at<uchar>(row, col) > threshold)
                {
                    if (count++ >= min_count)
                    {
                        right = col;
                        found = true;
                        break;
                    }
                }
            }
            if (found) {
                // std::cout << "right: " << col << " [" << (int)input.at<uchar>(row, col) << "]" << std::endl;
                break;
            }
        }
        // std::cout << "Finished finding last column!" << std::endl;

        top = (top - offset) > 0 ? top - offset : top;
        left = (left - offset) > 0 ? left - offset : left;
        bottom = (bottom + offset) > input.rows ? bottom : bottom + offset;
        right = (right + offset) > input.cols ? right : right + offset;
        ROS_INFO("ROI: %d, %d, %d, %d", left, top, right, bottom);

        cv::Rect R(left, top, right - left, bottom - top);
        return R;
    }


    /**
     * Overlays the robot's location over the input image of the occupancy grid cropped to the region of interest.
     * @param input cv::Mat representation of the occupancy grid cropped to the region of interest
     * @param roi   cv::Rect the region of interest that was used to crop the image
     */
    cv::Mat MapToJpeg::overlayRobotLocation(cv::Mat input, cv::Rect roi)
    {
        cv::Mat output(input.size().width, input.size().height, input.type());
        input.copyTo(output);
        
        // Get the robot pose
        geometry_msgs::TransformStamped transformStamped;
        try {
            // Get the current tranform of the robot
            transformStamped = tfBuffer_.lookupTransform("world", "robot0", ros::Time(0));
            // ROS_INFO("transform (x: %f, y: %f, z: %f)",
            //     transformStamped.transform.translation.x / og_res_,
            //     transformStamped.transform.translation.y / og_res_,
            //     transformStamped.transform.translation.z / og_res_
            //     );

            // Find the location of the robot
            cv::Point robot_center;
            robot_center.x = map_origin_x - roi.x + (transformStamped.transform.translation.x / og_res_);;
            robot_center.y = map_origin_y - roi.y + (transformStamped.transform.translation.y / og_res_);;
            // ROS_INFO("Robot center x: %d, y: %d", robot_center.x, robot_center.y);

            // Draw a circle to represent the robot
            circle(output, robot_center, RADIUS_OF_ROBOT/og_res_, cv::Scalar(0, 0, 255), -1);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        return output;
    }

} // namespace bthere_map_to_jpeg