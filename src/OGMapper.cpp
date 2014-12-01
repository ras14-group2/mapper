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
    pc_sub_Ptr = new message_filters::Subscriber<OGMapper::pcmsg>(nh, "/object_finder/wallpoints", 1);
//    poseIrPolicy pol = poseIrPolicy(10);
//    pol.setMaxIntervalDuration(ros::Duration(0.1));
    ir_synchronizerPtr = new message_filters::Synchronizer<poseIrPolicy>(poseIrPolicy(1000), *pose_sub_Ptr, *ir_sub_Ptr);
    ir_synchronizerPtr->registerCallback(&OGMapper::poseIrCallback, this);
    pc_synchronizerPtr = new message_filters::Synchronizer<posePcPolicy>(posePcPolicy(1000), *pose_sub_Ptr, *pc_sub_Ptr);
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

    //intitialize with dummy values to determine first message
//    initialPosition = position(M_PI, M_PI);

    //initialize map
    map = std::vector<std::vector<int8_t> >(GRID_SIDE_LENGTH_M * CELLS_PER_METER, std::vector<int8_t>(GRID_SIDE_LENGTH_M*CELLS_PER_METER, -1));
    xOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;
    yOffset = GRID_SIDE_LENGTH_M*CELLS_PER_METER / 2;

    irRoboPosition = position(0, 0);
    irRoboOrientation = 0;

//    maxXVal = 0;
//    minXVal = 0;
//    maxYVal = 0;
//    minYVal = 0;

    irDataAvailable = false;
    pcDataAvailable = false;

    updateCounter = 0;

    return;
}

void OGMapper::update(){

    //for each sensor:
    //compute global sensor position + global measurement point position
    //compute touched cells + set to free
    //compute measurement point position + set to occupied
    if(!irDataAvailable && !pcDataAvailable){
        ROS_ERROR("no new data available");
        if(updateCounter >= 50){
            visualizeGrid();
            updateCounter = 0;
        }
        updateCounter++;
        return;
    }
    updateCounter++;

    ROS_INFO("robo position: (%f, %f)", irRoboPosition.x, irRoboPosition.y);
    ROS_INFO("robo orientation: %f", irRoboOrientation);

    if(irDataAvailable){

        for(size_t i = 0; i < 4; i++){
            ROS_INFO("sensor %lu value: %f", i, sideSensorReadings[i]);
            if(sideSensorReadings[i] != -1 && sideSensorReadings[i] < MAX_SENSOR_DISTANCE){
                //wall seen set cell to occupied and in-between cells to free

                //compute relative posiiton of measured point
                position measuredPoint;
                measuredPoint.x = sideSensorPositions[i].x + sideSensorOrientations[i]*sideSensorReadings[i];
                measuredPoint.y = sideSensorPositions[i].y;

                //compute global positions
                position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i], irRoboPosition, irRoboOrientation);
                position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

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
                position globalSensorPosition = computeGlobalPosition(sideSensorPositions[i], irRoboPosition, irRoboOrientation);
                position globalPointPosition = computeGlobalPosition(measuredPoint, irRoboPosition, irRoboOrientation);

                //set cells
                std::vector<cell> touchedCells = computeTouchedGridCells(globalSensorPosition, globalPointPosition);
                const size_t nOfFreeCells = touchedCells.size();
                for(size_t j = 0; j < nOfFreeCells; j++){
                    setFree(touchedCells[j]);
                }
            }
            sideSensorReadings[i] = -1;
        }
        irDataAvailable = false;
    }

    if(pcDataAvailable){

        for(size_t i = 0; i < wallPoints.size(); i++){
            setOccupied(computeGridCell(computeGlobalPosition(wallPoints[i], pcRoboPosition, pcRoboOrientation)));
        }
        pcDataAvailable = false;
    }

    //set cells covered by the robot to free
    setCellsInsideRobotFree();

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

//    sideSensorReadings[0] = irMsg->front_right / 100.0d;
//    sideSensorReadings[1] = irMsg->back_right / 100.0d;
//    sideSensorReadings[2] = irMsg->back_left / 100.0d;
//    sideSensorReadings[3] = irMsg->front_left / 100.0d;

    sideSensorReadings[0] = irMsg->front_left / 100.0d;
    sideSensorReadings[1] = irMsg->back_left / 100.0d;
    sideSensorReadings[2] = irMsg->back_right / 100.0d;
    sideSensorReadings[3] = irMsg->front_right / 100.0d;

//    if(initialPosition.x == M_PI && initialPosition.y == M_PI){
//        initialPosition = irRoboPosition;
//    }

    irDataAvailable = true;

    return;
}

void OGMapper::posePcCallback(const OGMapper::posemsg::ConstPtr &poseMsg, const OGMapper::pcmsg::ConstPtr &pcMsg){

    pcRoboPosition.x = poseMsg->twist.linear.x;
    pcRoboPosition.y = poseMsg->twist.linear.y;
    pcRoboOrientation = poseMsg->twist.angular.z;

    wallPoints = std::vector<position>(pcMsg->points.size());
    for(size_t i = 0; i < pcMsg->points.size(); i++){
        wallPoints[i].x = pcMsg->points[i].x;
        wallPoints[i].y = pcMsg->points[i].y;
    }
    pcDataAvailable = true;
    return;
}

OGMapper::position OGMapper::computeGlobalPosition(position relativePosition, position roboPosition, double roboOrientation){
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
        std::vector<cell> insideLine = computeTouchedGridCells(computeGlobalPosition(insideRoboLines[i][0], irRoboPosition, irRoboOrientation), computeGlobalPosition(insideRoboLines[i][1], irRoboPosition, irRoboOrientation));
        for(size_t j = 0; j < insideLine.size(); j++){
            setFree(insideLine[j]);
        }
    }
    return;
}

void OGMapper::setFree(cell gridCell){    
    int8_t oldVal = map[gridCell.y + yOffset][gridCell.x + xOffset];
    if(oldVal == -1){
        //unknown cell
        map[gridCell.y + yOffset][gridCell.x + xOffset] = 40;
    }
    else{
        //cell known, adapt value
        map[gridCell.y + yOffset][gridCell.x + xOffset] = oldVal <= 10 ? 0 :  oldVal - 10;
    }
    return;
}

void OGMapper::setOccupied(cell gridCell){
    int8_t oldVal = map[gridCell.y + yOffset][gridCell.x + xOffset];
    if(oldVal == -1){
        //unknown cell
        map[gridCell.y + yOffset][gridCell.x + xOffset] = 60;
    }
    else{
        //cell known, adapt value
        map[gridCell.y + yOffset][gridCell.x + xOffset] = oldVal >= 90 ? 0 :  oldVal + 10;
    }
    return;
}

void OGMapper::visualizeGrid(){

    size_t gridWidth = map[0].size();
    size_t gridHeight = map.size();

    size_t gridSize = gridWidth * gridHeight;

    ROS_INFO("grid size: %lu, %lu", gridWidth, gridHeight);

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

    // fill the occupancy grid
    for(size_t i = 0; i < gridHeight; i++){
        size_t offset = i * gridWidth;
        for(size_t j = 0; j < gridWidth; j++){
            og.data[offset + j] = map[i][j];
        }
    }
    gridPub.publish(og);
    return;
}

} //namespace mappers
