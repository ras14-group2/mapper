#include "OGMapper.h"

#include <cmath>
#include <queue>
#include <map>

#include <mapper/PathToUnknown.h>

namespace mappers{

OGMapper::OGMapper(){
    ros::NodeHandle nh;

    //TODO: not working yet, fix
    nh.param<bool>("visualize_grid", visualize, false);
    visualize = true;

    if(visualize){
        ROS_INFO("running with visualization");
    }
    else{
        ROS_INFO("running without visualization");
    }
		
		nodeCreationSub = nh.subscribe("/node_creation", 100, &OGMapper::nodeCreationCallback, this);
		
    objectSub = nh.subscribe("/recognition_controller/identified_objects", 1, &OGMapper::objectCallback, this);
    markerPub = nh.advertise<visualization_msgs::Marker>("/mapper/object_marker", 1);
    //irSub = nh.subscribe("/ir_reader_node/cdistance", 1, &OGMapper::irCallback, this);

    service = nh.advertiseService("/wall_in_front", &OGMapper::wallInFrontService, this);
    pathService = nh.advertiseService("/find_path", &OGMapper::findPath, this);

    knownObjects = std::list<object>();
    objectID = 0;

    pose_sub_Ptr = new message_filters::Subscriber<OGMapper::posemsg>(nh, "/posori/Twist", 1);
    pose_sub_Ptr->registerCallback(&OGMapper::poseCallback, this);
    ir_sub_Ptr = new message_filters::Subscriber<OGMapper::irmsg>(nh, "/ir_reader_node/cdistance", 1);
    ir_sub_Ptr->registerCallback(&OGMapper::irCallback, this);
    pc_sub_Ptr = new message_filters::Subscriber<OGMapper::pcmsg>(nh, "/object_finder/wallpoints", 1);
    poseIrPolicy irPol = poseIrPolicy(20);
    irPol.setMaxIntervalDuration(ros::Duration(0.05));
    const poseIrPolicy constIrPol = irPol;
    ir_synchronizerPtr = new message_filters::Synchronizer<poseIrPolicy>(constIrPol, *pose_sub_Ptr, *ir_sub_Ptr);
    ir_synchronizerPtr->registerCallback(&OGMapper::poseIrCallback, this);
    posePcPolicy pcPol = posePcPolicy(20);
    pcPol.setMaxIntervalDuration(ros::Duration(0.05));
    const posePcPolicy constPcPol = pcPol;
    pc_synchronizerPtr = new message_filters::Synchronizer<posePcPolicy>(constPcPol, *pose_sub_Ptr, *pc_sub_Ptr);
    pc_synchronizerPtr->registerCallback(&OGMapper::posePcCallback, this);

    gridPub = nh.advertise<nav_msgs::OccupancyGrid>("/mapper/grid", 1);
    pathToUnknownPub = nh.advertise<mapper::PathToUnknown>("/mapper/pathToUnknown", 1);

    //sensor positions relative to robot origin
    sideSensorPositions = std::vector<position>(4);
    sideSensorPositions[0] = position(0.055, 0.09);
    sideSensorPositions[1] = position(0.055, -0.1);
    sideSensorPositions[2] = position(-0.055, -0.1);
    sideSensorPositions[3] = position(-0.055, 0.09);

    sideSensorOrientations = std::vector<int>(4);
    sideSensorOrientations[0] = 1;
    sideSensorOrientations[1] = 1;
    sideSensorOrientations[2] = -1;
    sideSensorOrientations[3] = -1;

    sideSensorReadings = std::vector<double>(4, -1);

    fbSensorPositions = std::vector<position>(2);
    fbSensorPositions[0] = position(0, 0.09);
    fbSensorPositions[1] = position(0, -0.1);

    fbSensorOrientations = std::vector<int>(2);
    fbSensorOrientations[0] = 1;
    fbSensorOrientations[1] = -1;

    fbSensorReadings = std::vector<double>(2, -1);

    insideRoboLines = std::vector<std::vector<position> >(8, std::vector<position>(2));
    insideRoboLines[0][0] = position(0.08, 0.09);
    insideRoboLines[0][1] = position(-0.08, 0.09);
    insideRoboLines[1][0] = position(-0.08, 0.09);
    insideRoboLines[1][1] = position(-0.08, -0.09);
    insideRoboLines[2][0] = position(-0.08, -0.09);
    insideRoboLines[2][1] = position(0.08, -0.09);
    insideRoboLines[3][0] = position(0.08, -0.09);
    insideRoboLines[3][1] = position(0.08, 0.09);
    insideRoboLines[4][0] = position(0.11, 0.05);
    insideRoboLines[4][1] = position(0.11, -0.05);
    insideRoboLines[5][0] = position(0.11, -0.05);
    insideRoboLines[5][1] = position(-0.11, -0.05);
    insideRoboLines[6][0] = position(-0.11, -0.05);
    insideRoboLines[6][1] = position(-0.11, 0.05);
    insideRoboLines[7][0] = position(-0.11, 0.05);
    insideRoboLines[7][1] = position(0.11, 0.05);

    //initialize map
    //map = std::vector<std::vector<int8_t> >(GRID_SIDE_LENGTH_M * CELLS_PER_METER, std::vector<int8_t>(GRID_SIDE_LENGTH_M*CELLS_PER_METER, -1));
    map = nav_msgs::OccupancyGrid();
    map.header.frame_id = "/map";
    map.info.resolution = 1.0d/CELLS_PER_METER;
    gridHeight = GRID_SIDE_LENGTH_M * CELLS_PER_METER;
    gridWidth = GRID_SIDE_LENGTH_M * CELLS_PER_METER;
    map.info.width = gridWidth;
    map.info.height = gridHeight;
    map.data.resize(gridHeight * gridWidth);
    map.info.origin.position.x = - (double) GRID_SIDE_LENGTH_M / 2.0d;
    map.info.origin.position.y = - (double) GRID_SIDE_LENGTH_M / 2.0d;
    map.info.origin.position.z = 0;

    for(size_t i = 0; i < map.data.size(); i++){
        map.data[i] = -1;
    }

    grownMap = map;

    xOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;
    yOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;

    irRoboPosition = position(0, 0);
    irRoboOrientation = 0;

    updateCounter = 0;


    mazeExplored = false;
    explorationTarget = cell(-100, -100);

    return;
}

bool OGMapper::addNode(double x, double y) {
	const double mergeLimit = 0.1;
	mapNode n(x, y);

	if (nodes.empty()) {
		nodes.reserve(1000u);	//Should be plenty enough
		nodes.push_back(n);
		return true;
	}
	
	//Draw an edge from the previous node
	int dir = -1;	//0 east, 1 north, 2 west, 3 south
	mapNode& prevn = nodes.back();
	double xdiff = n.pos.x - prevn.pos.x;
	double ydiff = n.pos.y - prevn.pos.y;
	if (fabs(xdiff) > fabs(ydiff)) {
		if (xdiff > mergeLimit) {
			dir = 0;	//east
		} else if (xdiff < -mergeLimit) {
			dir = 2;	//west
		} else {
			//Too close to the last one
			return false;
		}
	} else {
		if (ydiff > mergeLimit) {
			dir = 1;	//north
		} else if (ydiff < -mergeLimit) {
			dir = 3;	//south
		} else {
			//Too close to the last one
			return false;
		}
	}
	int oppDir = (dir+2)%4;	//Opposite direction of dir
	
	//See if we can merge the node with a currently existing one.
	for (int i = 0;i<nodes.size();++i) {
		mapNode& other = nodes[i];
		if (fabs(other.pos.x - n.pos.x) < mergeLimit && fabs(other.pos.y - n.pos.y) < mergeLimit) {
			//Sufficiently close; merge!
			prevn.edges[dir] = mapNode::edge(nodes.size()-2, i, 1);
			other.edges[oppDir] = mapNode::edge(i, nodes.size()-2, 1);
			return false;
		}
	}
	
	prevn.edges[dir] = mapNode::edge(nodes.size()-2, nodes.size()-1, 1);
	n.edges[oppDir] = mapNode::edge(nodes.size()-1, nodes.size()-2, 1);
	nodes.push_back(n);
	return true;
}

void OGMapper::plotNodes() {

	for (int i = 0;i<nodes.size();++i) {
		
		mapNode& n = nodes[i];
		
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "topological map nodes";
		marker.id = i;

		// Set the marker type.
		marker.type = visualization_msgs::Marker::SPHERE;

		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = n.pos.x;
		marker.pose.position.y = n.pos.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();
		
		markerPub.publish(marker);
	}
}

void OGMapper::nodeCreationCallback(const geometry_msgs::Point::ConstPtr &msg) {
	ROS_INFO("Received node creation request at position (%lf, %lf)", msg->x, msg->y);
	if (true) {	//Perhaps some condition, but probably not
		addNode((double)(msg->x), (double)(msg->y));
	}
	plotNodes();
}

void OGMapper::objectCallback(const recognition_controller::ObjectPosition::ConstPtr &msg){

    ROS_INFO("received object position: %s at (%f, %f)", msg->name.c_str(), msg->position.x, msg->position.y);

    object newObject;
    newObject.globalPosition.x = msg->position.x;
    newObject.globalPosition.y = msg->position.y;
    newObject.name = msg->name;

    knownObjects.push_back(newObject);

    sendMarker(newObject.globalPosition, msg->name);
    return;
}

void OGMapper::poseCallback(const OGMapper::posemsg::ConstPtr &msg){
    return;
}

void OGMapper::irCallback(const OGMapper::irmsg::ConstPtr &msg){
    return;
}

void OGMapper::poseIrCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::irmsg::ConstPtr &irMsg){

    irRoboPosition.x = poseMsg->twist.linear.x;
    irRoboPosition.y = poseMsg->twist.linear.y;
    irRoboOrientation = poseMsg->twist.angular.z;

    sideSensorReadings[0] = irMsg->front_right / 100.0d;
    sideSensorReadings[1] = irMsg->back_right / 100.0d;
    sideSensorReadings[2] = irMsg->back_left / 100.0d;
    sideSensorReadings[3] = irMsg->front_left / 100.0d;

    fbSensorReadings[0] = irMsg->front_center / 100.0d;
    fbSensorReadings[1] = irMsg->back_center / 100.0d;

    if(mazeExplored){
        return;
    }

    processIrData();

    //set cells covered by the robot to free
    setCellsInsideRobotFree();

//    if(updateCounter > 50){
//        visualizeGrid();
//        updateCounter = 0;
//    }
//    updateCounter++;
    visualizeGrid();

    if(explorationTarget.x != -100 && explorationTarget.y != -100 && pathFree(currentPath)){
        //currently following a path
        return;
    }
    //if the maze is not completely explored yet, check if loop closure

    //check if position is known
    position left = position(-0.08, 0.09 + 1.0d/CELLS_PER_METER);
    position right = position(0.08, 0.09 + 1.0d/CELLS_PER_METER);

    position globalLeft = computeGlobalPosition(left, irRoboPosition, irRoboOrientation);
    position globalRight = computeGlobalPosition(right, irRoboPosition, irRoboOrientation);
    std::vector<cell> frontCells = computeTouchedGridCells(globalLeft, globalRight);

    int nOfVisitedCells = 0;
    for(size_t i = 0; i < frontCells.size(); i++){
        if(grownMap.data[frontCells[i].y*gridWidth + frontCells[i].x] == 0){
            nOfVisitedCells++;
        }
    }

    if(nOfVisitedCells >= 2){
        //we have been here, find the closest unknown cell
        ROS_INFO("reached known point, search for unknown cells");
        std::list<cell> path;
        cell pos = computeGridCell(irRoboPosition);
        if(findClosestUnknown(pos, path)){
            //send path to maze_navigator
            ROS_INFO("unknown cell found, send path for following (size %lu)", path.size());

            mapper::PathToUnknown msg;

            geometry_msgs::Point startPoint;
            startPoint.x = irRoboPosition.x;
            startPoint.y = irRoboPosition.y;
            msg.points.push_back(startPoint);

            currentPath.clear();
            currentPath.push_back(irRoboPosition);

            std::list<cell>::const_iterator nextCell = path.begin();
            nextCell++;
            nextCell++;
            for(std::list<cell>::const_iterator end = path.end(); nextCell != end; nextCell++){
                 std::list<cell>::const_iterator currentCell = nextCell;
                 currentCell--;
                  std::list<cell>::const_iterator lastCell = currentCell;
                  lastCell--;

                //compute last direction
                int lastXDir = currentCell->x - lastCell->x;
                int lastYDir = currentCell->y - lastCell->y;
                int newXDir = nextCell->x - currentCell->x;
                int newYDir = nextCell->y - currentCell->y;

                ROS_INFO("diffs: lastX: %d, newX: %d, lastY: %d, newY: %d", lastXDir, newXDir, lastYDir, newYDir);

                if(newXDir != lastXDir || newYDir != lastYDir){
                    //turn in path, add node
                    geometry_msgs::Point pt;
                    position pos = computePositionFromGridCell(*currentCell);                    
                    pt.x = pos.x;
                    pt.y = pos.y;
                    ROS_INFO("node in path: (%f, %f)", pt.x, pt.y);
                    msg.points.push_back(pt);
                    currentPath.push_back(pos);

                }
            }
            position lastPos = computePositionFromGridCell(path.back());
            explorationTarget = computeGridCell(lastPos);
            geometry_msgs::Point pt;
            pt.x = lastPos.x;
            pt.y = lastPos.y;
            msg.points.push_back(pt);
            currentPath.push_back(lastPos);
            ROS_INFO("added target point: (%f, %f)", pt.x, pt.y);

            pathToUnknownPub.publish(msg);

            //visualize path
            visualization_msgs::Marker pathMsg;
            pathMsg.header.frame_id = "/map";
            pathMsg.header.stamp = ros::Time::now();
            pathMsg.type = visualization_msgs::Marker::LINE_STRIP;
            pathMsg.color.r = 1.0;
            pathMsg.color.a = 1.0;
            pathMsg.ns = "basic_shapes";
            pathMsg.action = visualization_msgs::Marker::ADD;
            pathMsg.id = 0;
            pathMsg.scale.x = 0.02;
            for(size_t i = 0; i < msg.points.size(); i++){
                pathMsg.points.push_back(msg.points[i]);
            }
            markerPub.publish(pathMsg);

        }
        else{
            //no unknown cells reachable, go back to start
            ROS_INFO("no unknown cells found, go back to start");
            mazeExplored = true;
            //TODO
        }
    }

    return;
}

void OGMapper::posePcCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::pcmsg::ConstPtr &pcMsg){

    if(mazeExplored){
        return;
    }

//		ROS_INFO("received cloud with timestamp [%d.%d]. (current time: [%d.%d])", pcMsg->header.stamp.sec, pcMsg->header.stamp.nsec, ros::Time::now().sec, ros::Time::now().nsec);
    pcRoboPosition.x = poseMsg->twist.linear.x;
    pcRoboPosition.y = poseMsg->twist.linear.y;
    pcRoboOrientation = poseMsg->twist.angular.z;

//    int nOfPoints = std::min((int) pcMsg->points.size(), 50);

//    wallPoints = std::vector<position>(pcMsg->points.size());
    for(size_t i = 0; i < pcMsg->points.size(); i++){
        position wallPoint;

        wallPoint.x = pcMsg->points[i].x;
        wallPoint.y = pcMsg->points[i].y;

        position globalPointPosition = computeGlobalPosition(wallPoint, pcRoboPosition, pcRoboOrientation);
        setOccupied(computeGridCell(globalPointPosition));
    }

    //set cells covered by the robot to free
    setCellsInsideRobotFree();

//    if(updateCounter > 50){
//        visualizeGrid();
//        updateCounter = 0;
//    }
//    updateCounter++;
    visualizeGrid();

    return;
}

bool OGMapper::findPath(mapper::PathToObject::Request &req, mapper::PathToObject::Response &res) {
	
	//find the shortest path
	//TODO
	
	//Lookup where the desired object is
	//TODO
	
	//Return some dummy path
	res.path[0] = req.start;
//	res.length = 1;
	
	return true;
}

bool OGMapper::wallInFrontService(mapper::WallInFront::Request &req, mapper::WallInFront::Response &res){

    ros::Time now = ros::Time::now();
//    ROS_INFO("received wall in front request at %d.%d", now.sec, now.nsec);
    //depth of observed box
    //double depth = 0.1;

    //back corners of observed box
    position bl(-0.12, 0.12);
    position br(0.12, 0.12);
    position tip(0, 0.17);

    double depth = tip.y - bl.y;

    //position of the robot
    position roboPosition(req.position.x, req.position.y);
    double roboOrientation = req.angle;

    int occupiedCells = 0;

    int nOfLines = depth * CELLS_PER_METER + 1;

    for(size_t i = 0; i < nOfLines; i++){
        //compute ends of line to check
        position leftEnd(((nOfLines-i)*bl.x + i*tip.x) /(double) nOfLines, bl.y + ((double) i)/ (double)CELLS_PER_METER);
        position rightEnd(((nOfLines-i)*br.x + i*tip.x) /(double) nOfLines, br.y + ((double) i)/(double)CELLS_PER_METER);
        
        ROS_INFO("checking line from (%f, %f) to (%f, %f)", leftEnd.x, leftEnd.y, rightEnd.x, rightEnd.y);

        position globalLeftEnd = computeGlobalPosition(leftEnd, roboPosition, roboOrientation);
        position globalRightEnd = computeGlobalPosition(rightEnd, roboPosition, roboOrientation);

        std::vector<cell> lineCells = computeTouchedGridCells(globalLeftEnd, globalRightEnd);

        for(size_t j = 0; j < lineCells.size(); j++){
            if(getCellValue(lineCells[j]) > 80){
                occupiedCells++;
                if(occupiedCells > 5){
                    res.wallInFront = 1;
                    now = ros::Time::now();
                    ROS_INFO("responded WALL at %d.%d, position (%lu, %lu)", now.sec, now.nsec, j, i);
                    return true;
                }
            }
        }
    }
    res.wallInFront = 0;

    now = ros::Time::now();
    ROS_INFO("responded free at %d.%d", now.sec, now.nsec);
    return true;
}

void OGMapper::processIrData(){

    for(size_t i = 0; i < 4; i++){
//        ROS_INFO("sensor %lu value: %f", i, sideSensorReadings[i]);
        if(sideSensorReadings[i] != -1 && sideSensorReadings[i] < MAX_SENSOR_DISTANCE){
            //wall seen set cell to occupied and in-between cells to free

            //compute relative posiiton of measured point
            position measuredPoint;
            measuredPoint.x = sideSensorPositions[i].x + sideSensorOrientations[i]*sideSensorReadings[i];
            measuredPoint.y = sideSensorPositions[i].y;

            //compute global positions
            position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i], irRoboPosition, irRoboOrientation);
            position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

//            ROS_INFO("global sensor position: (%f, %f)", globalSensorPosition.x, globalSensorPosition.y);
//            ROS_INFO("global point position: (%f, %f)", globalPointPosition.x, globalPointPosition.y);

            //set cells
            std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
            const size_t nOfFreeCells = touchedCells.size() - 1;
            for(size_t j = 0; j < nOfFreeCells; j++){
                setFree(touchedCells[j]);
            }
            setOccupied(computeGridCell(globalPointPosition));
        }
        else if(sideSensorReadings[i] != -1){
            //no wall seen, set close cells to free

            //compute relative posiiton of measured point
            position measuredPoint;
            measuredPoint.x = sideSensorPositions[i].x + sideSensorOrientations[i]*MAX_SENSOR_DISTANCE;
            measuredPoint.y = sideSensorPositions[i].y;

            //compute global positions
            position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i], irRoboPosition, irRoboOrientation);
            position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

            //set cells
            std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
            const size_t nOfFreeCells = touchedCells.size() - 1;
            for(size_t j = 0; j < nOfFreeCells; j++){
                setFree(touchedCells[j]);
            }
        }
    }

    //process front + back sensor
    for(size_t i = 0; i < 2; i++){
        if(fbSensorReadings[i] < MAX_LONG_RANGE_SENSOR_DISTANCE){
            //wall seen set cell to occupied and in-between cells to free

            //compute relative posiiton of measured point
            position measuredPoint;
            measuredPoint.x = fbSensorPositions[i].x;
            measuredPoint.y = fbSensorPositions[i].y + fbSensorReadings[i]* fbSensorOrientations[i];

            //compute global positions
            position globalSensorPosition = computeGlobalPosition(fbSensorPositions[i], irRoboPosition, irRoboOrientation);
            position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

            //set cells
            std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
            const size_t nOfFreeCells = touchedCells.size() - 1;
            for(size_t j = 0; j < nOfFreeCells; j++){
                setFree(touchedCells[j]);
            }
            setOccupied(computeGridCell(globalPointPosition));
        }
        else if(sideSensorReadings[i] != -1){
            //no wall seen, set close cells to free

            //compute relative posiiton of measured point
            position measuredPoint;
            measuredPoint.x = fbSensorPositions[i].x;
            measuredPoint.y = fbSensorPositions[i].y + fbSensorOrientations[i] * MAX_LONG_RANGE_SENSOR_DISTANCE;

            //compute global positions
            position globalSensorPosition = computeGlobalPosition(fbSensorPositions[i], irRoboPosition, irRoboOrientation);
            position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

            //set cells
            std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
            const size_t nOfFreeCells = touchedCells.size() - 1;
            for(size_t j = 0; j < nOfFreeCells; j++){
                setFree(touchedCells[j]);
            }
        }
    }
    return;
}

bool OGMapper::pathFree(std::list<OGMapper::position> path){
    std::list<position>::const_iterator nextTurn = path.begin();
    nextTurn++;
    for(std::list<position>::const_iterator end = path.end(); nextTurn != end; nextTurn++){
        std::list<position>::const_iterator lastTurn = nextTurn;
        lastTurn--;

        //get touched grid cells
        std::vector<cell> pathCells = computeTouchedGridCells(*lastTurn, *nextTurn);
        for(size_t i = 0; i < pathCells.size(); i++){
            if(grownMap.data[pathCells[i].y*gridWidth + pathCells[i].x] == 100){
                return false;
            }
        }
    }
    return true;
}

OGMapper::position OGMapper::computeGlobalPosition(position relativePosition, position roboPosition, double roboOrientation){
    position globalPosition;
    globalPosition.x = roboPosition.x - (std::sin(roboOrientation)*relativePosition.x) - (std::cos(roboOrientation)*relativePosition.y);
    globalPosition.y = roboPosition.y + (std::cos(roboOrientation)*relativePosition.x) - (std::sin(roboOrientation)*relativePosition.y);
    return globalPosition;
}

OGMapper::cell OGMapper::computeGridCell(position globalPosition){
    return cell(round(globalPosition.x * CELLS_PER_METER) + xOffset, round(globalPosition.y * CELLS_PER_METER) + yOffset);
}

OGMapper::position OGMapper::computePositionFromGridCell(cell gridCell){
    double x = ((double) (gridCell.x - xOffset)) / ((double) CELLS_PER_METER);
    double y = ((double) (gridCell.y - yOffset)) / ((double) CELLS_PER_METER);
    return position(x, y);
}

std::vector<OGMapper::cell> OGMapper::computeTouchedGridCells(position origin, position point){
    double xDiff = point.x - origin.x;
    double yDiff = point.y - origin.y;

    double xSign = xDiff > 0 ? 1 : -1;
    double ySign = yDiff > 0 ? 1 : -1;

    std::vector<cell> cells(0);

    if(xDiff == 0){
        cell nextCell = computeGridCell(origin);
        position cornerPosition;
        do{
            cells.push_back(nextCell);
            cornerPosition = computeCellCornerPosition(nextCell, 1, yDiff);
//            ROS_INFO("corner position: (%f, %f)", cornerPosition.x, cornerPosition.y);
            nextCell.y += ySign;
        }
        while((point.y - cornerPosition.y)*ySign > 0);
    }
    else{
        double m = yDiff / xDiff;

//        position originOffset = position(origin.x - (originCell.x / CELLS_PER_METER), origin.y - (originCell.y / CELLS_PER_METER));
        cell nextCell = computeGridCell(origin);
        position cornerPosition;

        do{
            cells.push_back(nextCell);
            cornerPosition = computeCellCornerPosition(nextCell, xSign, ySign);

            //decide on next cell
//            if((origin.y + m*xSign*(cornerPosition.x - origin.x)) * ySign > cornerPosition.y){
            if((origin.y + m*(cornerPosition.x - origin.x)) * ySign > cornerPosition.y * ySign){
                nextCell.y += ySign;
            }
            else{
                nextCell.x += xSign;
            }
        }
        while((point.x - cornerPosition.x)*xSign > 0 || (point.y - cornerPosition.y)*ySign > 0);
    }

    return cells;
}

OGMapper::position OGMapper::computeCellCornerPosition(cell gridCell, double xSign, double ySign){
    position corner;
    corner.x = ((double) (gridCell.x - xOffset) / CELLS_PER_METER) + (xSign * 1.0/(2.0 * CELLS_PER_METER));
    corner.y = ((double) (gridCell.y - yOffset) / CELLS_PER_METER) + (ySign * 1.0/(2.0 * CELLS_PER_METER));
    return corner;
}

void OGMapper::setCellsInsideRobotFree(){
    for(size_t i = 0; i < insideRoboLines.size(); i++){
        std::vector<cell> insideLine = computeTouchedGridCells(computeGlobalPosition(insideRoboLines[i][0], irRoboPosition, irRoboOrientation), computeGlobalPosition(insideRoboLines[i][1], irRoboPosition, irRoboOrientation));
        for(size_t j = 0; j < insideLine.size(); j++){
            setFree(insideLine[j]);
        }
//        setStrictFree(computeGridCell(irRoboPosition));
    }
    cell centerCell = computeGridCell(irRoboPosition);
    std::vector<cell> cellDiffs(8);
    cellDiffs[0] = cell(-1, -1);
    cellDiffs[1] = cell(0, -1);
    cellDiffs[2] = cell(1, -1);
    cellDiffs[3] = cell(-1, 0);
    cellDiffs[4] = cell(1, 0);
    cellDiffs[5] = cell(-1, 1);
    cellDiffs[6] = cell(0, 1);
    cellDiffs[7] = cell(1, 1);

    for(size_t i = 0; i < cellDiffs.size(); i++){
        cell tcell = cell(centerCell.x + cellDiffs[i].x, centerCell.y + cellDiffs[i].y);
        if(insideMap(tcell)){
            grownMap.data[tcell.y*gridWidth + tcell.x] = 0;
        }
    }
    return;
}

void OGMapper::setFree(cell gridCell){    
//    ROS_INFO("access gridCell[%d][%d] (set free)", gridCell.y, gridCell.x);
    int8_t oldVal = map.data[gridCell.y*gridWidth + gridCell.x];
    if(oldVal == -1){
        //unknown cell
        map.data[gridCell.y*gridWidth + gridCell.x] = 30;
        if(grownMap.data[gridCell.y*gridWidth + gridCell.x] == -1){
            growFree(gridCell);
        }
    }
    else{
        //cell known, adapt value
        map.data[gridCell.y*gridWidth + gridCell.x] = oldVal <= 20 ? 0 :  oldVal - 20;
    }
    return;
}

void OGMapper::setStrictFree(cell gridCell){
    map.data[gridCell.y*gridWidth + gridCell.x] = 0;
    grownMap.data[gridCell.y*gridWidth + gridCell.x] = 0;
    return;
}

void OGMapper::setOccupied(cell gridCell){
//    ROS_INFO("access gridCell[%d][%d] (set occupied)", gridCell.y, gridCell.x);
    int8_t oldVal = map.data[gridCell.y*gridWidth + gridCell.x];
    if(oldVal == -1){
        //unknown cell
        map.data[gridCell.y*gridWidth + gridCell.x] = 70;
    }
    else{
        //cell known, adapt value
        map.data[gridCell.y*gridWidth + gridCell.x] = oldVal >= 80 ? 100 :  oldVal + 20;
        if(map.data[gridCell.y*gridWidth + gridCell.x] >= 80){// && isEnvironmentOccupied(gridCell)){
//            ROS_INFO("cell value is 100, start environment check");
            if(isEnvironmentOccupied(gridCell)){
//                ROS_INFO("occupied cells in environment found, start growing");
                growRegion(gridCell);
            }

        }
    }
    return;
}

int8_t OGMapper::getCellValue(cell gridCell){
    return map.data[gridCell.y*gridWidth + gridCell.x];
}

bool OGMapper::isEnvironmentOccupied(cell gridCell){
    std::vector<cell> cellDiffs(20);
    cellDiffs[0] = cell(-1, -1);
    cellDiffs[1] = cell(0, -1);
    cellDiffs[2] = cell(1, -1);
    cellDiffs[3] = cell(-1, 0);
    cellDiffs[4] = cell(1, 0);
    cellDiffs[5] = cell(-1, 1);
    cellDiffs[6] = cell(0, 1);
    cellDiffs[7] = cell(1, 1);
    cellDiffs[8] = cell(-1, -2);
    cellDiffs[9] = cell(0, -2);
    cellDiffs[10] = cell(1, -2);
    cellDiffs[11] = cell(-1, 2);
    cellDiffs[12] = cell(0, 2);
    cellDiffs[13] = cell(1, 2);
    cellDiffs[14] = cell(-2, -1);
    cellDiffs[15] = cell(-2, 0);
    cellDiffs[16] = cell(-2, 1);
    cellDiffs[17] = cell(2, -1);
    cellDiffs[18] = cell(2, 0);
    cellDiffs[19] = cell(2, 1);

    size_t occupiedCells = 0;

    for(size_t i = 0; i < cellDiffs.size(); i++){
        cell tcell(gridCell.x + cellDiffs[i].x, gridCell.y + cellDiffs[i].y);
        if(insideMap(tcell) && getCellValue(tcell) > 80){
            occupiedCells++;
        }
    }
    return occupiedCells >= 2;
}

bool OGMapper::insideMap(cell gridCell){
//    ROS_INFO("cell (%d, %d) inside map?", gridCell.x, gridCell.y);
    return gridCell.x >= 0 && gridCell.y >= 0 && gridCell.x < GRID_SIDE_LENGTH_M*CELLS_PER_METER && gridCell.y < GRID_SIDE_LENGTH_M*CELLS_PER_METER;
}

void OGMapper::growFree(cell gridCell){
    std::vector<cell> cellDiffs(9);
    cellDiffs[0] = cell(-1, -1);
    cellDiffs[1] = cell(0, -1);
    cellDiffs[2] = cell(1, -1);
    cellDiffs[3] = cell(-1, 0);
    cellDiffs[4] = cell(1, 0);
    cellDiffs[5] = cell(-1, 1);
    cellDiffs[6] = cell(0, 1);
    cellDiffs[7] = cell(1, 1);
    cellDiffs[8] = cell(0, 0);

    for(size_t i = 0; i < cellDiffs.size(); i++){
        cell tcell = cell(gridCell.x + cellDiffs[i].x, gridCell.y + cellDiffs[i].y);
        if(insideMap(tcell) && grownMap.data[tcell.y*gridWidth + tcell.x] == -1){
            grownMap.data[tcell.y*gridWidth + tcell.x] = 1;
            if(tcell.x = explorationTarget.x && tcell.y == explorationTarget.y){
                explorationTarget = cell(-100, -100);
            }
        }
    }
    return;
}

void OGMapper::growRegion(cell gridCell){
    std::vector<int> lineWidths(7);
    lineWidths[0] = 2;
    lineWidths[1] = 4;
    lineWidths[2] = 5;
    lineWidths[3] = 5;
    lineWidths[4] = 6;
    lineWidths[5] = 6;
    lineWidths[6] = 6;

//    for(int line = gridCell.y - lineWidths.size() + 1; line < gridCell.y + lineWidths.size(); line++){
//        for(int column = gridCell.x - lineWidths[line]; column <= gridCell.x + lineWidths[line]; column++){
//            cell tcell(column, line);
//            if(insideMap(tcell)){
//                grownMap.data[tcell.y*gridWidth + tcell.x] = 100;
//            }
//        }
//    }
    for(size_t i = 0; i  < lineWidths.size(); i++){
        int line = gridCell.y - lineWidths.size() + 1 + i;
        for(int column = gridCell.x - lineWidths[i]; column <= gridCell .x + lineWidths[i]; column++){
            cell tcell(column, line);
            if(insideMap(tcell) && grownMap.data[tcell.y*gridWidth + tcell.x] != 0){
                grownMap.data[tcell.y*gridWidth + tcell.x] = 100;
                if(tcell.x = explorationTarget.x && tcell.y == explorationTarget.y){
                    explorationTarget = cell(-100, -100);
                }
            }
        }
    }
    for(size_t i = 0; i < lineWidths.size() - 1; i++){
        int line = gridCell.y + lineWidths.size() - 1 - i;
        for(int column = gridCell.x - lineWidths[i]; column <= gridCell .x + lineWidths[i]; column++){
            cell tcell(column, line);
            if(insideMap(tcell) && grownMap.data[tcell.y*gridWidth + tcell.x] != 0){
                grownMap.data[tcell.y*gridWidth + tcell.x] = 100;
                if(tcell.x = explorationTarget.x && tcell.y == explorationTarget.y){
                    explorationTarget = cell(-100, -100);
                }
            }
        }
    }

}

bool OGMapper::findClosestUnknown(cell startCell, std::list<cell> &path){
    std::priority_queue<searchCell> nextSearchCells;
    std::map<cell, cell> predecessors;

    searchCell start;
    start.currentCell = startCell;
    start.cost = 0;

    nextSearchCells.push(start);

    bool foundCell = false;

    while(!nextSearchCells.empty() && !foundCell){
    ROS_INFO("heap size: %lu", nextSearchCells.size());
        searchCell currCell = nextSearchCells.top();
        nextSearchCells.pop();

        if(predecessors.find(currCell.currentCell) != predecessors.end()){
            continue;
        }

        predecessors.insert(std::make_pair(currCell.currentCell, currCell.lastCell));

        if(grownMap.data[currCell.currentCell.y*gridWidth + currCell.currentCell.x] == -1){
            //found cell, find way back
            path = std::list<cell>();
            cell tcell = currCell.currentCell;
            path.push_front(tcell);
            predecessors.insert(std::make_pair(startCell, cell(0, 0)));
            do{
                tcell = predecessors.at(tcell);
                path.push_front(tcell);
            }
            while(tcell.x != startCell.x || tcell.y != startCell.y);
            foundCell = true;
        }
        else{
            //not found yet, keep searching

            //get last direction
            int xDir = currCell.currentCell.x - currCell.lastCell.x;
            int yDir = currCell.currentCell.y - currCell.lastCell.y;

            std::vector<cell> cellDiffs(4);
            cellDiffs[0] = cell(-1, 0);
            cellDiffs[1] = cell(0, 1);
            cellDiffs[2] = cell(1, 0);
            cellDiffs[3] = cell(0, -1);

            for(size_t i = 0; i < 4; i++){
                cell nextCell = cell(currCell.currentCell.x + cellDiffs[i].x, currCell.currentCell.y + cellDiffs[i].y);
                if(grownMap.data[nextCell.y*gridWidth + nextCell.x] != 100){
                    if(cellDiffs[i].x == xDir && cellDiffs[i].y == yDir){
                        //go in same direction
                        searchCell sc(nextCell, currCell.currentCell, currCell.cost+1);
                        nextSearchCells.push(sc);
                    }
                    else if(cellDiffs[i].x == -xDir && cellDiffs[i].y == -yDir){
                        //don't go back, do nothing
                    }
                    else{
                        //turn
                        searchCell sc(nextCell, currCell.currentCell, currCell.cost+50);
                        nextSearchCells.push(sc);
                    }
                }
            }
        }
    }

    return foundCell;
}

void OGMapper::visualizeGrid(){
    map.header.stamp = ros::Time::now();
    gridPub.publish(grownMap);
    return;
}

void OGMapper::sendMarker(position globalPosition, std::string objectName){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = objectID++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = globalPosition.x;
    marker.pose.position.y = globalPosition.y;
    marker.pose.position.z = 0.01;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text = objectName;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    markerPub.publish(marker);

    return;
}

} //namespace mappers
