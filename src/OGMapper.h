#ifndef OGMAPPER_H
#define OGMAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <ir_reader/distance_readings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>

#define CELLS_PER_METER 100
#define MAX_SENSOR_DISTANCE 0.25

namespace mappers{

class OGMapper{

public:
    OGMapper();
    void update();

    //typedefs
    typedef geometry_msgs::TwistStamped posemsg;
    typedef ir_reader::distance_readings irmsg;
    typedef message_filters::sync_policies::ApproximateTime<OGMapper::posemsg, OGMapper::irmsg> poseIrPolicy;

    //subscriber callback functions
    void poseCallback(const OGMapper::posemsg::ConstPtr &msg);
    void irCallback(const OGMapper::irmsg::ConstPtr &msg);
    void poseIrCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::irmsg::ConstPtr &irMsg);



private:
    //expresses the position of a point in either global or robot space
    struct position{
        double x;
        double y;

        position(){
            this->x = 0;
            this->y = 0;
        }

        position(double x, double y){
            this->x = x;
            this->y = y;
        }
    };

    //expresses the position of a cell in the occupancy grid
    struct cell{
        int x;
        int y;

        cell(){
            this->x = 0;
            this->y = 0;
        }

        cell(int x, int y){
            this->x = x;
            this->y = y;
        }

        friend bool operator<(const cell& lhs, const cell& rhs){
            return (lhs.y < rhs.y ||(lhs.y == rhs.y && lhs.x < rhs.x));
        }
    };

    //subscribers for robot pose and ir-sensor values
//    ros::Subscriber poseSub;
//    ros::Subscriber irSub;

    //filters
    message_filters::Subscriber<OGMapper::posemsg> *pose_sub_Ptr;
    message_filters::Subscriber<OGMapper::irmsg> *ir_sub_Ptr;
    message_filters::Synchronizer<poseIrPolicy> *synchronizerPtr;


    //a new message was received
    bool dataAvailable;

    //publisher for the occupancy grid messages
    ros::Publisher gridPub;

    //enable or disable generation of occupancy grid message
    bool visualize;

    //internal grid representation
    std::map<cell, int8_t> map;

    //values to simplify creation of the grid message
    int maxXVal;
    int minXVal;
    int maxYVal;
    int minYVal;

    //holds the values of the side ir-sensors
    std::vector<double> sideSensorReadings;

    //positions of the side ir-sensors in robot space (clockwise, beginning with front right)
    std::vector<position> sideSensorPositions;

    //sign of x-direction of the sensors in robot space (clockwise, beginning with front right)
    std::vector<int> sideSensorOrientations;

    //points to express some lines inside the robot to set the cells covered by the robot to free
    std::vector<std::vector<position> > insideRoboLines;

    //the robot position in global space (from odometry)
    position roboPosition;

    //the robot orientation (from odometry)
    double roboOrientation;



    //computes the global position from a position in robot space according to the robot's position and orientation
    position computeGlobalPosition(position relativePosition);

    //computes the corresponding cell in the occupancy grid from a global position
    cell computeGridCell(position globalPosition);

    //computes all cells touched by a ray from sensorPosition to point (ordered)
    std::vector<cell> computeTouchedGridCells(position sensorPosition, position point);

    //computes the corner of the grid cell that is needed to decide which cell is touched next
    position computeCellCornerPosition(cell gridCell, double xSign, double ySign);

    //sets the touched cells of all lines in insideRoboLines to free
    void setCellsInsideRobotFree();

    //sets the given cell to free (increases probability for free)
    void setFree(cell gridCell);

    //sets the given cell to occupied (increases probability for occupied)
    void setOccupied(cell gridCell);

    //adapts the global maximum and minimum values of the grid
    void adaptMaxValues(cell gridCell);

    //generates a nav_msgs::OccupancyGrid message from internal grid representation
    void visualizeGrid();

};

}//namespace mappers

#endif //OGMAPPER_H
