#include "map_to_jpeg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bthere_map_to_jpeg", ros::init_options::AnonymousName);
    bthere_map_to_jpeg::MapToJpeg obj(argc, argv);
    ros::spin();

    return 0;
}