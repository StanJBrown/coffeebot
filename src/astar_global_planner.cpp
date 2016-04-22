#include <pluginlib/class_list_macros.h>
#include "astar_global_planner.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include "std_msgs/Header.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/publisher.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_cb_global_planner::AStar_CB_GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

ofstream MyExcelFile ("CB_INFO.xlsx", ios::trunc);
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
static const int OBSTACLE_THRESHOLD = 2;
//static const float VISITED_THRESHOLD = 0.01; // The threshold that states the cell has been visited and is now in the closed set so cant visit it again
static const float SLEEP_TIME = 0.01; // sleep for 1 second
bool debug=0; // flag used for debugging
bool* visited; // same size as cost map, stores cost of cell to indicate visited
int* cameFromCellArray;
float infinity = std::numeric_limits< float >::infinity();
unsigned int CellMapSize;
vector<int> finalPath;
multiset<cellInfo,comp> frontier;
ros::Time astar_runtime;

// Publish visualization markers
ros::NodeHandle n;
ros::Publisher plan_pub;

//Default Constructor
namespace astar_cb_global_planner {

    // Constructor
    AStar_CB_GlobalPlanner::AStar_CB_GlobalPlanner(){
    }

    // Constructor
    AStar_CB_GlobalPlanner::AStar_CB_GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_INFO(" ##### Constructor called");
        initialize(name, costmap_ros);
    }

    // ***** Checked *****
    // Initialize the global planner
    void AStar_CB_GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_INFO(" ##### Initializing AStar_CB_GlobalPlanner");
        if(!CMinitialized_){
            // Get Cost Map
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            // Get Specifications of cost map
            CMoriginX = costmap_->getOriginX(); // costmap origin wrt world
            CMoriginY = costmap_->getOriginY();
            CMresolution = costmap_->getResolution(); //meters/cell
            CMwidth = costmap_->getSizeInCellsX();
          	CMheight = costmap_->getSizeInCellsY();
            cellArrayWidth = CMwidth;
            cellArrayHeight = CMheight;
          	CellMapSize = cellArrayWidth*cellArrayHeight;

            plan_pub = n.advertise<nav_msgs::Path>("planB", 1);

            cout << "CMwidth: " << CMwidth << " CMheight: " << CMheight << endl;
            cout << "CellArraywidth: " << cellArrayWidth << " CellArrayheight: " << cellArrayHeight << endl;
            cout << "originX: " << CMoriginX << " originY: " << CMoriginY <<endl;
            cout << "resolution: " << CMresolution;
            if(debug){
                letsSleep();
            }
            ros::Time begin = ros::Time::now();
            int total = 0;
            // Create visited array and cameFromCellArray
            visited = new bool[CellMapSize];
            cameFromCellArray = new int[CellMapSize];
            for(unsigned int iy=0; iy<cellArrayHeight; iy++){
                for(unsigned int ix=0; ix<cellArrayWidth; ix++){
                    //static_cast<int>(costmap_->getCost(ix,iy));
                    visited[iy*cellArrayWidth+ix] = 0;
                    cameFromCellArray[iy*cellArrayWidth+ix] = -1; // set to -1 since cell index of cost map starts at 0.
                    total++;
                }
            }
            cout << "total cells " << total << endl;
            cout << "Time takes: " << begin-ros::Time::now() << endl;
            // Write to Excel file
            MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;
            ROS_INFO(" ##### AStar CB planner initialized successfully");

            //plan_pub = n.advertise<nav_msgs::Path>("visualize_path", 10);
            //ros::Rate r(30);
            // Indicate finished initialization
            CMinitialized_ = true;
        }
        else{
            ROS_WARN(" ##### This planner has already been initialized... doing nothing");
        }
    }


    // Make a plan
    bool AStar_CB_GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan ){
        ROS_INFO(" ##### makePlan called");
        // Check if cost map is initialized
        if (!CMinitialized_){
            ROS_ERROR(" ##### Astar CB planner hasnt been initialized");
            return false;
        }

        // Send debug message of start point
        ROS_DEBUG(" ##### Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

        // Start a new plan
        plan.clear();
        // Initialize came from and visited arrays
        for(unsigned int iy=0; iy<cellArrayHeight; iy++){
            for(unsigned int ix=0; ix<cellArrayWidth; ix++){
                visited[iy*cellArrayWidth+ix] = 0;
                cameFromCellArray[iy*cellArrayWidth+ix] = -1; // set to -1 since cell index of cost map starts at 0.
            }
        }
        // Clear final Path reversed
        finalPath.clear();

        // Clear frontier
        frontier.clear();

        // Check if the frame ids of the cost map is same as the goal frame id
        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR(" ##### This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        // Store start and goal in variables (world coordinate frames)
        float WstartX = start.pose.position.x;
        float WstartY = start.pose.position.y;
        float WgoalX = goal.pose.position.x;
        float WgoalY = goal.pose.position.y;

        // Unnecessary variables used for better readability (WCM represents the conversion from world to cost map. The function WCoordToCMCoord
        // passes variables by reference)
        float WCMstartX = WstartX;
        float WCMstartY = WstartY;
        float WCMgoalX = WgoalX;
        float WCMgoalY = WgoalY;

        if(debug){
            cout << " WStartX " << WstartX << " WstartY "
            << WstartY << " WGoalX " << WgoalX << " WGoaly " << WgoalY << endl;
            letsSleep();
        }

        // startX,startY,goalX,goalY represent cell locations in costmap frame
        WCoordToCMCoord(WCMstartX,WCMstartY);
        WCoordToCMCoord(WCMgoalX,WCMgoalY);

        if(debug){
            cout << " StartX " << WCMstartX << " startY "
            << WCMstartY << " GoalX " << WCMgoalX << " Goaly " << WCMgoalY << endl;
            letsSleep();
        }

        // Get the cellIndex
        int startCellIdx = CMCoordToCellIdx(WCMstartX, WCMstartY);;
        int goalCellIdx = CMCoordToCellIdx(WCMgoalX,  WCMgoalY);
        if(debug){
            cout << "Start idx " << startCellIdx << " Goal idx " << goalCellIdx << endl;
        }

        if(isCellValid(startCellIdx) && isCellValid(goalCellIdx)){
            // pass the AstarPlanner
            cout << "Start: ";
            displayRowCol(startCellIdx);
            cout << "Goal: ";
            displayRowCol(goalCellIdx);
            //letsSleep();
            //letsSleep();
            astar_runtime = ros::Time::now();
            clock_t startTime;
            startTime = clock();
            double elapsed;
            if(AStarPlanner(startCellIdx,goalCellIdx)){ // Call Astar here. If Astar returns true we have a plan
                //cout << "Astar found a path in " << ros::Time::now()-astar_runtime << endl;
                ROS_INFO("##### Astar found a path");
                //plan = finalPath;
                float x,y;
                for(int i=0; i<finalPath.size(); i++){
                    geometry_msgs::PoseStamped new_goal = goal;
                    CellIdxToCMCoord(finalPath[i], x, y);
                    CMCoordToWCoord(x, y);
                    if(debug)
                        cout << "x,y " << x << " " << y << endl;
                    new_goal.pose.position.x = x;
                    new_goal.pose.position.y = y;
                    new_goal.pose.orientation.x = 0;
                    new_goal.pose.orientation.y = 0;
                    new_goal.pose.orientation.z = 0;
                    new_goal.pose.orientation.w = 1;
                    plan.push_back(new_goal);
                }
                //letsSleep();
                //letsSleep();
                elapsed = (clock() - startTime) / (double)(CLOCKS_PER_SEC / 1000);
                cout << "Astar with clock timing " << elapsed << " seconds" << endl;
                cout << "Astar found a path in " << ros::Time::now()-astar_runtime << " seconds"<< endl;
                nav_msgs::Path rvizPath;
                rvizPath.poses.resize(plan.size());
                rvizPath.header.frame_id = plan[0].header.frame_id;
                rvizPath.header.stamp = plan[0].header.stamp;
                for(unsigned int i=0; i < plan.size(); i++)
                {
                    rvizPath.poses[i] = plan[i];
                }
                plan_pub.publish(rvizPath);
                return true;
            }
            else{
                ROS_INFO("##### Could not find a path");
                return false;
            }
        }
        else{
            ROS_WARN(" ##### The start or goal is not valid");
            return false;
        }
    }

    // ***** Checked *****
    // A* planning
    bool AStar_CB_GlobalPlanner::AStarPlanner(int startCellIdx, int goalCellIdx){
        ROS_INFO("##### Performing Astar search");
        cellInfo node;
        // Setup start node
        node.fval = (0.1+heuristicCost(startCellIdx,goalCellIdx));
        node.gval = 0.1;
        node.current = startCellIdx;
        node.cameFrom = startCellIdx;
        float curr_total_cost,curr_cost;
        int curr_cellIdx,curr_cameFrom,curr_rowID,curr_colID;
        // Insert start node into frontier
        frontier.insert(node);
        while(!frontier.empty()){
            curr_total_cost = frontier.begin()->fval; // total_cost
            curr_cost = frontier.begin()->gval; // cost
            curr_cellIdx = frontier.begin()->current; // pos
            curr_cameFrom = frontier.begin()->cameFrom; // previous
            curr_rowID = CellIdxToCellRowID(curr_cellIdx);
            curr_colID = CellIdxToCellColID(curr_cellIdx);
            if(debug){
                cout << "--------------------------------------------" << endl;
                cout << "Curr row col " << curr_rowID << " " << curr_colID << endl;
                cout << "Curr cell idx " << curr_cellIdx << endl;
                cout << "visited ?: " << visited[curr_cellIdx] << endl;
                cout << "Frontier Size " << frontier.size() << endl;
            }
            frontier.erase(frontier.begin());
            if(visited[curr_cellIdx]!=0){
                if(debug){
                    cout << "Visited was not zero "<< endl;
                    letsSleep();
                }
                continue;
            }
            visited[curr_cellIdx] = 1;
            cameFromCellArray[curr_cellIdx] = curr_cameFrom;
            if(curr_cellIdx == goalCellIdx){
                cout << "Distance to reach goal " << curr_cost*CMresolution  << " meters"<< endl;
                break;
            }
            for(int i=-1; i<=1; i++){
                for(int j=-1; j<=1; j++){
                    int neighbor_RowID = curr_rowID+i;
                    int neighbor_ColID = curr_colID+j;
                    if(!((neighbor_RowID>=0) && (neighbor_ColID>=0) && (neighbor_RowID<cellArrayHeight) && (neighbor_ColID<cellArrayWidth)))
                        continue;
                    int neighbor_cellIdx = CellRowColToCellIdx(neighbor_ColID,neighbor_RowID);
                    int neighbor_obst_cost = static_cast<int>(costmap_->getCost(neighbor_ColID,neighbor_RowID));
                    if(neighbor_obst_cost<OBSTACLE_THRESHOLD && visited[neighbor_cellIdx]==0){
                        cellInfo neighbor_node;
                        neighbor_node.gval=curr_cost+moveCost(curr_cellIdx,neighbor_cellIdx)+neighbor_obst_cost;
                        neighbor_node.fval=neighbor_node.gval+heuristicCost(neighbor_cellIdx,goalCellIdx);
                        neighbor_node.current=neighbor_cellIdx;
                        neighbor_node.cameFrom=curr_cellIdx;
                        if(debug){
                            cout << "Adding cell row col to neighbor " << neighbor_RowID << " " << neighbor_ColID << endl;
                            letsSleep();
                        }
                        frontier.insert(neighbor_node);
                    }
                }
            }
            //cout << "Frontier Size " << frontier.size() << endl;
        }
        if(curr_cellIdx==goalCellIdx){
            ROS_INFO("##### I found the goal :D");
            int pathIdx = goalCellIdx;
            while(pathIdx!=startCellIdx){
                finalPath.push_back(pathIdx);
                pathIdx = cameFromCellArray[pathIdx];
            }
            finalPath.push_back(pathIdx);
            std::reverse(finalPath.begin(),finalPath.end());
            if(debug){
                for(int i=0; i<finalPath.size(); i++)
                    displayRowCol(finalPath[i]);
                cout << endl;
            }
            return true;
        }
        if(frontier.empty())
            ROS_INFO("##### My frontier is empty :S");
        return false;
    }

    // Converts World coordinates into Cost Map coordinates
    void AStar_CB_GlobalPlanner::WCoordToCMCoord(float& wcmx, float& wcmy){
        wcmx = wcmx - CMoriginX;
        wcmy = wcmy - CMoriginY;
    }


    // Converts cell coordinates into cell index
    int AStar_CB_GlobalPlanner::CMCoordToCellIdx(float cmx, float cmy){
        int cellIndex;
        float col = cmx / CMresolution;
        float row = cmy / CMresolution;
        cellIndex = CellRowColToCellIdx(col,row);
        return cellIndex;
    }

    // Converts Cell Index to CostMap coordinates
    void AStar_CB_GlobalPlanner::CellIdxToCMCoord(int index, float& cmx, float& cmy){
        cmx = CellIdxToCellColID(index) * CMresolution;
        cmy = CellIdxToCellRowID(index) * CMresolution;
    }

    // Converts CostMap Coordinates to World Coordinates
    void AStar_CB_GlobalPlanner::CMCoordToWCoord(float& cmwx, float& cmwy){
        cmwx = cmwx + CMoriginX;
        cmwy = cmwy + CMoriginY;
    }

    // Valid:
    // not obstacle,
    // not already visited,
    // cost not -1 (questionable) <- not included
    // check bounds
    bool AStar_CB_GlobalPlanner::isCellValid(int cellIndex){
        bool valid = true;
        int rowID = CellIdxToCellRowID(cellIndex);
        int colID = CellIdxToCellColID(cellIndex);
        unsigned int cost = static_cast<int>(costmap_->getCost(colID,rowID));
        float cmx, cmy;
        CellIdxToCMCoord(cellIndex,cmx,cmy);
        if(cost>OBSTACLE_THRESHOLD){
            valid = false;
            if(debug)
                ROS_INFO("##### cost is greater than obstacle threshold");
            return valid;
        }
        if(visited[cellIndex]){
            valid = false;
            if(debug)
                ROS_INFO("##### Cell it has already been visited");
            return valid;
        }
        if(!isCMCoordInsideMap(cmx,cmy)){
            valid = false;
            if(debug)
                ROS_INFO("##### Cell is outside the map");
            return valid;
        }
        return valid;
    }


    // Check if cost map coordinates are inside cost map
    bool AStar_CB_GlobalPlanner::isCMCoordInsideMap(float CMx, float CMy){
        bool valid = true;
        if (CMx > CMwidth || CMy > CMheight)
            valid = false;
        return valid;
    }

    void AStar_CB_GlobalPlanner::letsSleep(){
        ros::Duration(SLEEP_TIME).sleep();
    }
};
