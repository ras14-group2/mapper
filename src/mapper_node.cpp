#include <ros/ros.h>

#include "OGMapper.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "mapper");

    mappers::OGMapper mapper;

    ros::spin();
	return 0;
}
