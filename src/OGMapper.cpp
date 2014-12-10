#include "OGMapper.h"

#include <cmath>

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

    insideRoboLines = std::vector<std::vector<position> >(6, std::vector<position>(2));
    insideRoboLines[0][0] = sideSensorPositions[0];
    insideRoboLines[0][1] = sideSensorPositions[1];
    insideRoboLines[1][0] = sideSensorPositions[1];
    insideRoboLines[1][1] = sideSensorPositions[2];
    insideRoboLines[2][0] = sideSensorPositions[2];
    insideRoboLines[2][1] = sideSensorPositions[3];
    insideRoboLines[3][0] = sideSensorPositions[3];
    insideRoboLines[3][1] = sideSensorPositions[0];
    insideRoboLines[4][0] = sideSensorPositions[0];
    insideRoboLines[4][1] = sideSensorPositions[2];
    insideRoboLines[5][0] = sideSensorPositions[1];
    insideRoboLines[5][1] = sideSensorPositions[3];

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

    xOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;
    yOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;

    irRoboPosition = position(0, 0);
    irRoboOrientation = 0;

    updateCounter = 0;
    
    nodes.reserve(1000u);	//Should be plenty enough
    nodes_file = fopen(file_path, "r");
		if (nodes_file == NULL) {
			//File not found. This means that there are no stored nodes from a previous run. Act accordingly.
			//TODO
		} else {
			nodes.clear();
			mapNode n;
			while (fscanf(nodes_file, "%lf %lf\n", &(n.pos.x), &(n.pos.y)) != EOF) {
				fprintf(stderr, "Read: %lf %lf %lf\n", i, n.pos.x, n.pos.y);
				for (int j = 0;j < 4;++j) {
					fscanf(nodes_file, "%d %lf\n", &(n.edges[j].to), &(n.edges[j].dist));
					n.edges[j].from = id;
					fprintf(stderr, "%d %lf\n", n.edges[j].to, n.edges[j].dist);
				}
				fclose(nodes_file);
			}
		}

    return;
}

OGMapper::~OGMapper(){
	//Store the map in a file
	if (nodes_file != NULL) {
		nodes_file = fopen(file_path, "w");
		for (int i = 0;i<nodes.size();++i) {
			const mapNode& n = nodes[i];
			fprintf(nodes_file, "%lf %lf %lf\n", i, n.pos.x, n.pos.y);
			for (int j = 0;j < 4;++j) {
				fprintf(nodes_file, "%d %lf\n", n.edges[j].to, n.edges[j].dist);
			}
			fclose(nodes_file);
		}
	}
}

bool OGMapper::addNode(double x, double y) {
	const double mergeLimit = 0.1;
	mapNode n(x, y);

	if (nodes.empty()) {
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

    processIrData();

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

void OGMapper::posePcCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::pcmsg::ConstPtr &pcMsg){

		ROS_INFO("received cloud with timestamp [%d.%d]. (current time: [%d.%d])", pcMsg->header.stamp.sec, pcMsg->header.stamp.nsec, ros::Time::now().sec, ros::Time::now().nsec);
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

	//Lookup where the desired object is
	OGMapper::position destPos;
	bool found = false;
	for (std::list<object>::iterator it = knownObjects.begin(); it != knownObjects.end();++it) {
		if (it->name == req.object.data) {
			destPos.x = it->globalPosition.x;
			destPos.y = it->globalPosition.y;
			found = true;
			break;
		}
	}
	
	if (!found) {
		//The object is not found. Act accordingly.
		//TODO
		return false;
	}
	
	int start = -1;
	int dest = -1;
	for (int i = 0;i<nodes.size();++i) {
		if (fabs(nodes[i].pos.x - destPos.x) <= 0.1 && fabs(nodes[i].pos.y - destPos.y) <= 0.1) {
			//Destination node found
			dest = i;
		}
		if (fabs(nodes[i].pos.x - req.start.x) <= 0.1 && fabs(nodes[i].pos.y - req.start.y) <= 0.1) {
			//Starting node found
			start = i;
		}
	}
	if (dest == -1) {
		//Destination node not found, act accordingly.
		//TODO
	}
	if (start = -1) {
		//Starting node not found, act accordingly.
		//TODO
	}
	
	//find the shortest path
	const double nodePenalty = 0.0;	//The amount of nodes should affect the path weight
	const int UNDEF = -1;	//Some random value
	double* dists = new double[nodes.size()];
	double* prev = new double[nodes.size()];
	int u, v;
	std::vector<int> q;
	
	for (int i = 0;i<nodes.size();++i) {
		q.push_back(i);
		prev[i] = UNDEF;
		if (i == start) {
			dists[i] = 0.0;
		} else {
			dists[i] = UNDEF;
		}
	}
	
	while (!q.empty()) {
		double minD = UNDEF;
		u = UNDEF;
		for (int i = 0;i<q.size();++i) {
			if (minD == UNDEF || dists[i] > minD) {
				minD = dists[i];
				u = i;
			}
		}
		for (int i = 0;i<4;++i) {
			v = nodes[u].edges[i].to;
			double alt = dists[u] + nodes[u].edges[i].dist + nodePenalty;
			if (alt < dists[v]) {
				dists[v] = alt;
				prev[v] = u;
			}
		}
	}
	
	std::vector<int> revPath;
	int p = dest;
	while (p != start) {
		revPath.push_back(p);
		p = prev[p];
	}
	revPath.push_back(start);
	res.length.data = revPath.size();
	for (int i = 0;i<revPath.size();++i) {
		res.path[i].x = nodes[revPath[revPath.size()-1-i]].pos.x;
		res.path[i].y = nodes[revPath[revPath.size()-1-i]].pos.y;
	}
	
	return true;
}

bool OGMapper::wallInFrontService(mapper::WallInFront::Request &req, mapper::WallInFront::Response &res){

    ros::Time now = ros::Time::now();
    ROS_INFO("received wall in front request at %d.%d", now.sec, now.nsec);
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
                if(occupiedCells > 2){
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
    return;
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
    }
    return;
}

void OGMapper::setFree(cell gridCell){    
//    ROS_INFO("access gridCell[%d][%d] (set free)", gridCell.y, gridCell.x);
    int8_t oldVal = map.data[gridCell.y*gridWidth + gridCell.x];
    if(oldVal == -1){
        //unknown cell
        map.data[gridCell.y*gridWidth + gridCell.x] = 30;
    }
    else{
        //cell known, adapt value
        map.data[gridCell.y*gridWidth + gridCell.x] = oldVal <= 20 ? 0 :  oldVal - 20;
    }
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
    }
    return;
}

int8_t OGMapper::getCellValue(cell gridCell){
    return map.data[gridCell.y*gridWidth + gridCell.x];
}

void OGMapper::visualizeGrid(){

//    size_t gridWidth = map[0].size();
//    size_t gridHeight = map.size();

//    size_t gridSize = gridWidth * gridHeight;

//    ROS_INFO("grid size: %lu, %lu", gridWidth, gridHeight);

//    nav_msgs::OccupancyGrid og;
//    og.header.frame_id = "/map";
//    og.header.stamp = ros::Time::now();
//    og.info.resolution = 1.0d / CELLS_PER_METER;
//    og.info.width = gridWidth;
//    og.info.height = gridHeight;
//    og.data.resize(gridSize);
//    og.info.origin.position.x = 0;//-initialPosition.x; //- (double) gridWidth / (2*CELLS_PER_METER);
//    og.info.origin.position.y = 0;//-initialPosition.y;// - (double) gridHeight / (2*CELLS_PER_METER);
//    og.info.origin.position.z = 0;

//    // fill the occupancy grid
//    for(size_t i = 0; i < gridHeight; i++){
//        size_t offset = i * gridWidth;
//        for(size_t j = 0; j < gridWidth; j++){
//            og.data[offset + j] = map[i][j];
//        }
//    }
    map.header.stamp = ros::Time::now();
    gridPub.publish(map);
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
