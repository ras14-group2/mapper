#ifndef OGMAPPER_H
#define OGMAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <ir_reader/distance_readings.h>
#include <object_finder/WallPoints.h>
#include <recognition_controller/ObjectPosition.h>
#include <visualization_msgs/Marker.h>
#include <mapper/WallInFront.h>
#include <mapper/PathToObject.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/OccupancyGrid.h>

#define CELLS_PER_METER 50
#define MAX_SENSOR_DISTANCE 0.25
#define GRID_SIDE_LENGTH_M 10

namespace mappers{

class OGMapper{

public:
    OGMapper();

    //typedefs
    typedef geometry_msgs::TwistStamped posemsg;
    typedef ir_reader::distance_readings irmsg;
    typedef object_finder::WallPoints pcmsg;
    typedef message_filters::sync_policies::ApproximateTime<OGMapper::posemsg, OGMapper::irmsg> poseIrPolicy;
    typedef message_filters::sync_policies::ApproximateTime<OGMapper::posemsg, OGMapper::pcmsg> posePcPolicy;

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

    //saves a known object
    struct object{
        position globalPosition;
        std::string name;

        object(){
            this->globalPosition = position();
            this->name = "";
        }

        object(position globalPosition, std::string name){
            this->globalPosition = globalPosition;
            this->name = name;
        }
    };
    
    struct mapNode {
    	position pos;
    	bool active;
    	
    	struct edge {
    		int from, to;
    		double dist;
    		edge() {}
    		edge(int f, int t, double d) {
    			from = f;
    			to = t;
    			dist = d;
    		}
    	};
    	
    	edge edges[4];	//0 east, 1 north, 2 west, 3 south
    	
    	mapNode(double x, double y) : pos(x, y) {
    		for (int i = 0;i<4;++i) {
    			edges[i].to = -1;
    		}
    		active = true;
    	}
    };
		
		//node creation request subscriber
		ros::Subscriber nodeCreationSub;
		
    //subscriber + publisher for objects
    ros::Subscriber objectSub;
    ros::Publisher markerPub;

    //service server
    ros::ServiceServer service;
    ros::ServiceServer pathService;

    //list of all known objects
    std::list<object> knownObjects;

    //id for markers
    int objectID;

    //filters
    message_filters::Subscriber<OGMapper::posemsg> *pose_sub_Ptr;
    message_filters::Subscriber<OGMapper::irmsg> *ir_sub_Ptr;
    message_filters::Subscriber<OGMapper::pcmsg> *pc_sub_Ptr;
    message_filters::Synchronizer<poseIrPolicy> *ir_synchronizerPtr;
    message_filters::Synchronizer<posePcPolicy> *pc_synchronizerPtr;

    //counter to limit visualization frequency
    int updateCounter;

    //publisher for the occupancy grid messages
    ros::Publisher gridPub;

    //enable or disable generation of occupancy grid message
    bool visualize;

    //internal grid representation
    //std::vector<std::vector<int8_t> > map;
    nav_msgs::OccupancyGrid map;
    size_t gridHeight;
    size_t gridWidth;
    int xOffset;
    int yOffset;
		
		//Graph of the topological map
		std::vector<mapNode> nodes;

    //holds the values of the side ir-sensors
    std::vector<double> sideSensorReadings;

    //positions of the side ir-sensors in robot space (clockwise, beginning with front right)
    std::vector<position> sideSensorPositions;

    //sign of x-direction of the sensors in robot space (clockwise, beginning with front right)
    std::vector<int> sideSensorOrientations;

    //wallpoints from pointcloud
    std::vector<position> wallPoints;

    //points to express some lines inside the robot to set the cells covered by the robot to free
    std::vector<std::vector<position> > insideRoboLines;

    //the initial position of the robot, origin of the occupancy grid
    position initialPosition;

    //the robot position and orientation in global space (from odometry) for ir sensor data
    position irRoboPosition;
    double irRoboOrientation;

    //robot position and orientation in global space for pointcloud data
    position pcRoboPosition;
    double pcRoboOrientation;
		
		//find the path from one node to another
		bool findPath(mapper::PathToObject::Request &req, mapper::PathToObject::Response &res);
		
		//function to add a node to the topological map
		//Return true if inserted as a new node, false if merged with other node(s)
		bool addNode(double x, double y);
		
		//Visualize the topological map nodes in rviz
		void plotNodes();
		
    //subscriber callback functions
    void objectCallback(const recognition_controller::ObjectPosition::ConstPtr &msg);
    void poseCallback(const OGMapper::posemsg::ConstPtr &msg);
    void irCallback(const OGMapper::irmsg::ConstPtr &msg);
    void poseIrCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::irmsg::ConstPtr &irMsg);
    void posePcCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::pcmsg::ConstPtr &pcMsg);
    void nodeCreationCallback(const geometry_msgs::Point::ConstPtr &msg);
    
    //service functions
    bool wallInFrontService(mapper::WallInFront::Request &req, mapper::WallInFront::Response &res);

    //insert the information from the ir sensors into the map
    void processIrData();

    //computes the global position from a position in robot space according to the robot's position and orientation
    position computeGlobalPosition(position relativePosition, position roboPosition, double roboOrientation);

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

    //returns the given cell's value
    int8_t getCellValue(cell gridCell);

    //generates a nav_msgs::OccupancyGrid message from internal grid representation
    void visualizeGrid();

    //send a Marker to show the position of the detected object in the map
    void sendMarker(position globalPosition, std::string objectName);


};

}//namespace mappers

#endif //OGMAPPER_H
