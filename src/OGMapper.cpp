#include "OGMapper.h"

#include <nav_msgs/OccupancyGrid.h>

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

//    poseSub = nh.subscribe("/posori/Twist", 1, &OGMapper::poseCallback, this);
//    irSub = nh.subscribe("/ir_reader_node/cdistance", 1, &OGMapper::irCallback, this);

    pose_sub_Ptr = new message_filters::Subscriber<OGMapper::posemsg>(nh, "/posori/Twist", 1);
    pose_sub_Ptr->registerCallback(&OGMapper::poseCallback, this);
    ir_sub_Ptr = new message_filters::Subscriber<OGMapper::irmsg>(nh, "/ir_reader_node/cdistance", 1);
    ir_sub_Ptr->registerCallback(&OGMapper::irCallback, this);
//    poseIrPolicy pol = poseIrPolicy(10);
//    pol.setMaxIntervalDuration(ros::Duration(0.1));
    synchronizerPtr = new message_filters::Synchronizer<poseIrPolicy>(poseIrPolicy(10), *pose_sub_Ptr, *ir_sub_Ptr);
    synchronizerPtr->registerCallback(&OGMapper::poseIrCallback, this);

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

    //intitialize with dummy values to determine first message
//    initialPosition = position(M_PI, M_PI);

    roboPosition = position(0, 0);
    roboOrientation = 0;

    maxXVal = 0;
    minXVal = 0;
    maxYVal = 0;
    minYVal = 0;

    dataAvailable = false;

    updateCounter = 0;

    return;
}

void OGMapper::update(){

    //for each sensor:
    //compute global sensor position + global measurement point position
    //compute touched cells + set to free
    //compute measurement point position + set to occupied
    if(!dataAvailable){
        ROS_ERROR("no new data available");
        if(updateCounter >= 50){
            visualizeGrid();
            updateCounter = 0;
        }
        updateCounter++;
        return;
    }
    updateCounter++;

    ROS_INFO("robo position: (%f, %f)", roboPosition.x, roboPosition.y);
    ROS_INFO("robo orientation: %f", roboOrientation);

    for(size_t i = 0; i < 4; i++){
        ROS_INFO("sensor %lu value: %f", i, sideSensorReadings[i]);
        if(sideSensorReadings[i] != -1 && sideSensorReadings[i] < MAX_SENSOR_DISTANCE){
            //wall seen set cell to occupied and in-between cells to free

            //compute relative posiiton of measured point
            position measuredPoint;
            measuredPoint.x = sideSensorPositions[i].x + sideSensorOrientations[i]*sideSensorReadings[i];
            measuredPoint.y = sideSensorPositions[i].y;

            //compute global positions
            position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i]);
            position globalPointPosition = computeGlobalPosition(measuredPoint);

            ROS_INFO("global sensor position: (%f, %f)", globalSensorPosition.x, globalSensorPosition.y);
            ROS_INFO("global point position: (%f, %f)", globalPointPosition.x, globalPointPosition.y);

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
            position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i]);
            position globalPointPosition = computeGlobalPosition(measuredPoint);

            //set cells
            std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
            const size_t nOfFreeCells = touchedCells.size();
            for(size_t j = 0; j < nOfFreeCells; j++){
                setFree(touchedCells[j]);
            }
        }
        sideSensorReadings[i] = -1;
    }

    //set cells covered by the robot to free
    setCellsInsideRobotFree();

    dataAvailable = false;

    //visualize
//    if(visualize && (updateCounter == 50)){
////        ROS_INFO("visualizing grid");
//        visualizeGrid();
//        updateCounter = 0;
////        ROS_INFO("returned from visualizeGrid()");
////        sleep(2);
//    }

//    ROS_INFO("returning from update()");

    return;
}

void OGMapper::poseCallback(const OGMapper::posemsg::ConstPtr &msg){
    return;
}

void OGMapper::irCallback(const OGMapper::irmsg::ConstPtr &msg){
    return;
}

void OGMapper::poseIrCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::irmsg::ConstPtr &irMsg){
    roboPosition.x = poseMsg->twist.linear.x;
    roboPosition.y = poseMsg->twist.linear.y;
    roboOrientation = poseMsg->twist.angular.z;

//    sideSensorReadings[0] = irMsg->front_right / 100.0d;
//    sideSensorReadings[1] = irMsg->back_right / 100.0d;
//    sideSensorReadings[2] = irMsg->back_left / 100.0d;
//    sideSensorReadings[3] = irMsg->front_left / 100.0d;

    sideSensorReadings[0] = irMsg->front_left / 100.0d;
    sideSensorReadings[1] = irMsg->back_left / 100.0d;
    sideSensorReadings[2] = irMsg->back_right / 100.0d;
    sideSensorReadings[3] = irMsg->front_right / 100.0d;

//    if(initialPosition.x == M_PI && initialPosition.y == M_PI){
//        initialPosition = roboPosition;
//    }

    dataAvailable = true;

    return;
}


OGMapper::position OGMapper::computeGlobalPosition(position relativePosition){
    position globalPosition;
    globalPosition.x = roboPosition.x + (std::sin(roboOrientation)*relativePosition.x) + (std::cos(roboOrientation)*relativePosition.y);
    globalPosition.y = roboPosition.y - (std::cos(roboOrientation)*relativePosition.x) + (std::sin(roboOrientation)*relativePosition.y);
    return globalPosition;
}

OGMapper::cell OGMapper::computeGridCell(position globalPosition){
    return cell(round(globalPosition.x * CELLS_PER_METER), round(globalPosition.y * CELLS_PER_METER));
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
    corner.x = ((double) gridCell.x / CELLS_PER_METER) + (xSign * 1/(2 * CELLS_PER_METER));
    corner.y = ((double) gridCell.y / CELLS_PER_METER) + (ySign * 1/(2 * CELLS_PER_METER));
    return corner;
}

void OGMapper::setCellsInsideRobotFree(){
    for(size_t i = 0; i < insideRoboLines.size(); i++){
        std::vector<cell> insideLine = computeTouchedGridCells(computeGlobalPosition(insideRoboLines[i][0]), computeGlobalPosition(insideRoboLines[i][1]));
        for(size_t j = 0; j < insideLine.size(); j++){
            setFree(insideLine[j]);
        }
    }
    return;
}

void OGMapper::setFree(cell gridCell){
//    ROS_INFO("adding free cell: (%d, %d)", gridCell.x, gridCell.y);
    std::map<cell, int8_t>::iterator it = map.find(gridCell);
    if(it == map.end()){
        map.insert(std::make_pair(gridCell, 40));
    }
    else{
        it->second = it->second <= 10 ? 0 : it->second - 10;
    }
    map.insert(std::make_pair(gridCell, 0));
    adaptMaxValues(gridCell);
    return;
}

void OGMapper::setOccupied(cell gridCell){
//    ROS_INFO("adding occupied cell: (%d, %d)", gridCell.x, gridCell.y);
    std::map<cell, int8_t>::iterator it = map.find(gridCell);
    if(it == map.end()){
        map.insert(std::make_pair(gridCell, 60));
    }
    else{
        it->second = it->second >= 90 ? 100 : it->second + 10;
    }
    adaptMaxValues(gridCell);
    return;
}

void OGMapper::adaptMaxValues(cell gridCell){
    if(gridCell.x > maxXVal){
        maxXVal = gridCell.x;
    }
    else if(gridCell.x < minXVal){
        minXVal = gridCell.x;
    }

    if(gridCell.y > maxYVal){
        maxYVal = gridCell.y;
    }
    else if(gridCell.y < minYVal){
        minYVal = gridCell.y;
    }
    return;
}

void OGMapper::visualizeGrid(){

    ROS_INFO("maxXVal: %d", maxXVal);
    ROS_INFO("minXVal: %d", minXVal);
    ROS_INFO("maxYVal: %d", maxYVal);
    ROS_INFO("minYVal: %d", minYVal);

    int gridWidth = maxXVal - minXVal;
    if(maxXVal >= 0 && minXVal <= 0){
        gridWidth++;
    }
    int gridHeight = maxYVal - minYVal;
    if(maxYVal >= 0 && minYVal <= 0){
        gridHeight++;
    }
    int gridSize = gridWidth * gridHeight;

    ROS_INFO("grid size: %d, %d", gridWidth, gridHeight);

    nav_msgs::OccupancyGrid og;
    og.header.frame_id = "/map";
    og.header.stamp = ros::Time::now();
    og.info.resolution = 1.0d / CELLS_PER_METER;
    og.info.width = gridWidth;
    og.info.height = gridHeight;
    og.data.resize(gridSize);
    og.info.origin.position.x = 0;//-initialPosition.x; //- (double) gridWidth / (2*CELLS_PER_METER);
    og.info.origin.position.y = 0;//-initialPosition.y;// - (double) gridHeight / (2*CELLS_PER_METER);
    og.info.origin.position.z = 0;

    size_t foundCells = 0;

    // fill the occupancy grid
    for(size_t i = 0; i < og.data.size(); i++){
        og.data[i] = -1;
    }

    for(std::map<cell, int8_t>::const_iterator it = map.begin(), end = map.end(); it != end; it++){
        int index = gridWidth * (it->first.y - minYVal) + it->first.x - minXVal;
//        ROS_INFO("grid index for cell (%d, %d): %d", it->first.x, it->first.y, index);
        og.data[index] = it->second;
        foundCells++;
    }

    ROS_INFO("cells in map: %lu", map.size());
    ROS_INFO("found cells in map: %lu", foundCells);
    gridPub.publish(og);
    ROS_INFO("gridmessage published");
    return;
}

} //namespace mappers
