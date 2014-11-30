#include <ros/ros.h>

#include "OGMapper.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "mapper");

    mappers::OGMapper mapper;

    ros::Rate loop_rate(20);

    while(ros::ok()){
        ROS_INFO("loop");
        ros::spinOnce();
        mapper.update();
        loop_rate.sleep();
    }
	return 0;
}
