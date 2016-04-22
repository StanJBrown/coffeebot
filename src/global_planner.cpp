#include <pluginlib/class_list_macros.h>
#include <cmath>
#include "global_planner.h"
#include <pcl_ros/publisher.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

    // Map size in cells
    unsigned int mapSizeCX;
    unsigned int mapSizeCY;

    // Map size in meters
    double mapSizeMX;
    double mapSizeMY;
    double mapOriginX;
    double mapOriginY;
    double mapResolution;

    const unsigned int SAMPLE_SIZE = 10000;

    // Each cell of the cost map is a value from 0 to 255
    // Anything above this threshold is consisered occupied for path making
    const unsigned char COST_THRESHOLD = 2;
    const unsigned char GOAL_COST_THRESHOLD = 50;

    // The number of nearest vertices to attempt to connect to when building network
    const unsigned int N_NEAREST = 20;

    // PRM nodes and edges
    Vertex vertices[SAMPLE_SIZE];
    vector<Vertex> freeVertices;
    vector<Edge> edges;

    costmap_2d::Costmap2D* globalCostmap;

    ros::Publisher plan_pub;
    ros::Publisher viz_pub;

    GlobalPlanner::GlobalPlanner (){
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    //private refinePath

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // Get the map dimensions in terms of cells
        globalCostmap = costmap_ros->getCostmap();

        mapSizeCX = globalCostmap->getSizeInCellsX();
        mapSizeCY = globalCostmap->getSizeInCellsY();
        mapSizeMX = globalCostmap->getSizeInMetersX();
        mapSizeMY = globalCostmap->getSizeInMetersY();
        mapOriginX = globalCostmap->getOriginX();
        mapOriginY = globalCostmap->getOriginY();

        ROS_INFO("## - Initializing global planner - ##");
        ROS_INFO("## - Costmap origin: %f, %f", mapOriginX, mapOriginY);


        ros::NodeHandle n;

        // Parameters
        bool test;
        n.param("test_param", test, false);
        if (test)
        {
            ROS_INFO("## test_param : true ##");
        }

        // Publishers
        plan_pub = n.advertise<nav_msgs::Path>("planB", 1);
        viz_pub = n.advertise<visualization_msgs::Marker>("PRM_Network", 1);
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan )
    {
        // Start timer
        clock_t startTime;
        startTime = clock();
        double elapsed;

        // Clear out all global containers
        freeVertices.clear();
        edges.clear();

        // First add the start and goal poses as vertices
        vertices[0].xPosition = start.pose.position.x;
        vertices[0].yPosition = start.pose.position.y;
        vertices[0].inFreeSpace = true;
        vertices[1].xPosition = goal.pose.position.x;
        vertices[1].yPosition = goal.pose.position.y;
        vertices[1].inFreeSpace = true;

        // Check if goal location is in free space
        if (isOccupied(goal.pose.position.x, goal.pose.position.y, GOAL_COST_THRESHOLD))
        {
            ROS_INFO("Goal is placed on unreachable cell! No path generated");
            return (false);
        }
        else
        {
            // Push on goal and start position to freeVertices
            freeVertices.push_back(vertices[0]);
            freeVertices.push_back(vertices[1]);
        }

        // TODO: Check if straight line is feasible first

        ROS_INFO("Start position: %f, %f", vertices[0].xPosition, vertices[0].yPosition);
        ROS_INFO("Goal position: %f, %f", vertices[1].xPosition, vertices[1].yPosition);

        // Generate some random points on the map and check for obstacles
        srand(2);
        unsigned int freeVertexCount = 2;
        for (int i = 2; i < SAMPLE_SIZE; i++)
        {
            // Create random point
            double randX = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/mapSizeMX)) + mapOriginX;
            double randY = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/mapSizeMY)) + mapOriginY;
            vertices[i].xPosition = randX;
            vertices[i].yPosition = randY;

            // Check if it is a free cell
            if (!isOccupied(randX, randY, COST_THRESHOLD))
            {
                vertices[i].inFreeSpace = true;
                freeVertexCount++;
                freeVertices.push_back(vertices[i]);
            }
        }

        elapsed = (clock() - startTime) / (double)(CLOCKS_PER_SEC / 1000);
        double nextTime = clock();
        ROS_INFO("%d free vertices generated in %f ms", freeVertexCount, elapsed);

        // Check through all free vertices and generate a network
        for (int i = 0; i < freeVertexCount; i++)
        {
            vector<double> distances(freeVertexCount, 0.0);

            // Find nearest vertices
            for (int j = 0; j < freeVertexCount; j++)
            {
                distances[j] = distanceBetweenVertices(freeVertices[i], freeVertices[j]);
            }

            // sort by distance
            vector<int> indices = sortIndexes(distances);

            //ROS_INFO("--Nearest distances for node: %d", i);
            int nClosest = N_NEAREST;
            if (nClosest > freeVertexCount)
            {
                nClosest = freeVertexCount;
            }
            //for (int j = 0; j < nClosest; j++)
            //{
                //ROS_INFO("Nearest distance: %f, to node: %d", distances[j], j);
            //}

            // Attempt to create edges to the nearest vertices starting at closest vertex
            Vertex v1 = freeVertices[i];
            // Loop through the list of nearest vertices
            for (int j = 0; j < nClosest; j++)
            {
                Vertex v2 = freeVertices[indices[j]];
                // Skip the same vertex
                if (i == indices[j])
                {
                    continue;
                }

                if (!isLineOccupied(v1, v2))
                {
                    // Create edge
                    Edge newEdge;
                    newEdge.vertex1 = v1;
                    newEdge.vertex2 = v2;
                    newEdge.length = distances[indices[j]];
                    edges.push_back(newEdge);

                    // Add neighbour nodes
                    freeVertices[i].neighbourIndices.push_back(indices[j]);
                    freeVertices[i].neighbourDistances.push_back(distances[indices[j]]);

                    freeVertices[indices[j]].neighbourIndices.push_back(i);
                    freeVertices[indices[j]].neighbourDistances.push_back(distances[indices[j]]);
                }
            }
        }

        elapsed = (clock() - nextTime) / (double)(CLOCKS_PER_SEC / 1000);
        nextTime = clock();
        ROS_INFO("Network generated in %f ms", elapsed);

        // Check if start and finish nodes have no neighbours
        if (freeVertices[0].neighbourIndices.size() == 0 || freeVertices[1].neighbourIndices.size() == 0)
        {
            ROS_INFO("No connections could be made to the start or goal node!");
            return (false);
        }

        /*
        // display all vertex connections
        for (int i = 0; i < freeVertexCount; i++)
        {
            ROS_INFO("Vertex %d is connected to:", i);
            for (int j = 0; j < freeVertices[i].neighbourIndices.size(); j++)
            {
                ROS_INFO(" - %d with dist: %f", freeVertices[i].neighbourIndices[j], freeVertices[i].neighbourDistances[j]);
            }
        }*/

        // Search network using A*
        vector<int> pathFromGoal = aStarSearch(0, 1);

        // Refine the path
        if (pathFromGoal.size() > 2)
        {
            for (int i = 0; i < pathFromGoal.size() - 2; i++)
            {
                // Check if we can skip some nodes
                Vertex currentVertex = freeVertices[pathFromGoal[i]];
                Vertex nextVertex = freeVertices[pathFromGoal[i + 2]];

                while (!isLineOccupied(currentVertex, nextVertex))
                {
                    // Remove node in between
                    pathFromGoal.erase(pathFromGoal.begin() + i + 1);
                    if ((pathFromGoal.size() - 1) <= i + 1)
                    {
                        break;
                    }
                    else
                    {
                        nextVertex = freeVertices[pathFromGoal[i + 2]];
                    }
                }
            }
        }

        elapsed = (clock() - nextTime) / (double)(CLOCKS_PER_SEC / 1000);
        nextTime = clock();
        ROS_INFO("Path found and refined in %f ms", elapsed);

        // Finally create nav path from the A* path
        double path_length = 0.0;
        for (int i = pathFromGoal.size()-1; i >= 0; i--)
        {
            Vertex v = freeVertices[pathFromGoal[i]];

            // calc length of path
            if (i < pathFromGoal.size()-1)
            {
                double pathLen = distanceBetweenFreeVertices(pathFromGoal[i], pathFromGoal[i+1]);
                path_length += pathLen;
            }

            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
            new_goal.pose.position.x = v.xPosition;
            new_goal.pose.position.y = v.yPosition;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }

        elapsed = (clock() - startTime) / (double)(CLOCKS_PER_SEC / 1000);
        ROS_INFO("#### - New global path served fresh in %f ms - ####", elapsed);
        ROS_INFO("#### - Path length is: %f m - ####", path_length);

        // Publish data for visualization
        visualization_msgs::Marker edgeLines;
        edgeLines.header.frame_id = plan[0].header.frame_id;
        edgeLines.header.stamp = plan[0].header.stamp;
        edgeLines.ns = "Edges_cool";
        edgeLines.action = visualization_msgs::Marker::ADD;
        edgeLines.pose.orientation.w = 1.0;
        edgeLines.id = 0;
        edgeLines.type = visualization_msgs::Marker::LINE_LIST;
        edgeLines.scale.x = 0.01;
        edgeLines.color.r = 1.0;
        edgeLines.color.a = 0.6;

        // Display all edges
        for (int i = 0; i < edges.size(); i++)
        {
            // Create two points to define the line to be drawn
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            p1.x = edges[i].vertex1.xPosition;
            p1.y = edges[i].vertex1.yPosition;
            p2.x = edges[i].vertex2.xPosition;
            p2.y = edges[i].vertex2.yPosition;
            p1.z = 0.0;
            p2.z = 0.0;

            edgeLines.points.push_back(p1);
            edgeLines.points.push_back(p2);
        }

        nav_msgs::Path rvizPath;
        rvizPath.poses.resize(plan.size());

        rvizPath.header.frame_id = plan[0].header.frame_id;
        rvizPath.header.stamp = plan[0].header.stamp;

        for(unsigned int i=0; i < plan.size(); i++)
        {
            rvizPath.poses[i] = plan[i];
        }

        // Publish all network edges and then the path found
        viz_pub.publish(edgeLines);
        plan_pub.publish(rvizPath);

        return (true);
    }





    // Checks if the costmap cell is considered occupied
    // Input is in cell coordinates
    bool isOccupied(int x, int y, unsigned char threshold)
    {
        unsigned char cost = globalCostmap->getCost(x, y);
        if (cost >= threshold)
        {
            return (true);
        }
        return (false);
    }

    // Checks if the costmap cell is considered occupied
    // Input is in world coordinates
    bool isOccupied(double x, double y, unsigned char threshold)
    {
        int cX;
        int cY;
        globalCostmap->worldToMapEnforceBounds(x, y, cX, cY);
        unsigned char cost = globalCostmap->getCost(cX, cY);
        if (cost >= threshold)
        {
            return (true);
        }
        return (false);
    }

    // Uses the bresenham line algorithm to check if a line intersects with occupied cells.
    bool isLineOccupied(int x1, int y1, int const x2, int const y2)
    {
        int delta_x(x2 - x1);
        // if x1 == x2, then it does not matter what we set here
        signed char const ix((delta_x > 0) - (delta_x < 0));
        delta_x = std::abs(delta_x) << 1;

        int delta_y(y2 - y1);
        // if y1 == y2, then it does not matter what we set here
        signed char const iy((delta_y > 0) - (delta_y < 0));
        delta_y = std::abs(delta_y) << 1;

        if (isOccupied(x1, y1, COST_THRESHOLD)) { return (true); }

        if (delta_x >= delta_y)
        {
            // error may go below zero
            int error(delta_y - (delta_x >> 1));

            while (x1 != x2)
            {
                if ((error >= 0) && (error || (ix > 0)))
                {
                    error -= delta_x;
                    y1 += iy;
                }

                error += delta_y;
                x1 += ix;

                if (isOccupied(x1, y1, COST_THRESHOLD)) { return (true); }
            }
        }
        else
        {
            // error may go below zero
            int error(delta_x - (delta_y >> 1));

            while (y1 != y2)
            {
                if ((error >= 0) && (error || (iy > 0)))
                {
                    error -= delta_y;
                    x1 += ix;
                }

                error += delta_x;
                y1 += iy;

                if (isOccupied(x1, y1, COST_THRESHOLD)) { return (true); }
            }
        }
        return (false);
    }

    // Overload to allow passing in vertices
    bool isLineOccupied(Vertex pt1, Vertex pt2)
    {
        int x1, y1, x2, y2;
        globalCostmap->worldToMapEnforceBounds(pt1.xPosition, pt1.yPosition, x1, y1);
        globalCostmap->worldToMapEnforceBounds(pt2.xPosition, pt2.yPosition, x2, y2);

        return (isLineOccupied(x1, y1, x2, y2));
    }

    template <typename T>
    vector<int> sortIndexes(const vector<T> &v) {

        // initialize original index locations
        vector<int> idx(v.size());
        for (int i = 0; i != idx.size(); ++i) idx[i] = i;

        // sort indexes based on comparing values in v
        sort(idx.begin(), idx.end(),
        [&v](int i1, int i2) {return v[i1] < v[i2];});

        return idx;
    }

    // Calculates distance between two given vertices
    double distanceBetweenVertices(Vertex pt1, Vertex pt2)
    {
        double xDiff = pt2.xPosition - pt1.xPosition;
        double yDiff = pt2.yPosition - pt1.yPosition;

        double distance = sqrt(xDiff * xDiff + yDiff * yDiff);
        return (distance);
    }

    // Calculated distance between two free vertices identified by their array index
    double distanceBetweenFreeVertices(int ind1, int ind2)
    {
        double xDiff = freeVertices[ind2].xPosition - freeVertices[ind1].xPosition;
        double yDiff = freeVertices[ind2].yPosition - freeVertices[ind1].yPosition;

        double distance = sqrt(xDiff * xDiff + yDiff * yDiff);
        return (distance);
    }

    // Checks the given set for the given Vertex, Vertex is identified by index within freeVertices
    bool setContains(vector<GraphNode> set, int index)
    {
        for (int i = 0; i < set.size(); i++)
        {
            if (set[i].index == index)
            {
                return (true);
            }
        }
        return (false);
    }

    // Checks the given set and returns the index within the set if found
    // Input index is the index of the Vertex within freeVertices
    int getNodeIndex(vector<GraphNode> set, int index)
    {
        for (int i = 0; i < set.size(); i++)
        {
            if (set[i].index == index)
            {
                return (i);
            }
        }
        return (-1);
    }

    // Searches the connections of freeVertices for a path to goal
    vector<int> aStarSearch(int startIndex, int goalIndex)
    {
        bool goalFound = false;
        vector<GraphNode> openSet;
        vector<GraphNode> closedSet;

        GraphNode start = {startIndex, startIndex, 0.0, distanceBetweenFreeVertices(startIndex, goalIndex)};
        openSet.push_back(start);

        GraphNode current;
        int currentIndex;
        ROS_INFO("Starting search from %d to %d", startIndex, goalIndex);
        while (openSet.size() > 0)
        {
            // Remove from open set
            current = openSet.back();
            openSet.pop_back();
            currentIndex = current.index;

            // Add to closed set
            closedSet.push_back(current);
            if (currentIndex == goalIndex)
            {
                goalFound = true;
                break;
            }

            int neighbourCount = freeVertices[currentIndex].neighbourIndices.size();
            //ROS_INFO("Checking %d neighbours of vertex %d", neighbourCount, currentIndex);

            // Loop through all neighbours
            for (int i = 0; i < neighbourCount; i++)
            {
                int nextIndex = freeVertices[currentIndex].neighbourIndices[i];
                double newCost = current.costToReach + freeVertices[currentIndex].neighbourDistances[i];

                //ROS_INFO("Neighbour %d (%d) of vertex %d has a new cost of %f", i, nextIndex, currentIndex, newCost);

                if (setContains(openSet, nextIndex))
                {
                    if (newCost < openSet[getNodeIndex(openSet, nextIndex)].costToReach)
                    {
                        // Remove from open set
                        openSet.erase(openSet.begin() + getNodeIndex(openSet, nextIndex));
                    }
                }
                if (setContains(closedSet, nextIndex))
                {
                    if (newCost < closedSet[getNodeIndex(closedSet, nextIndex)].costToReach)
                    {
                        // Remove from closed set
                        closedSet.erase(closedSet.begin() + getNodeIndex(closedSet, nextIndex));
                    }
                }
                if (!setContains(openSet, nextIndex) && !setContains(closedSet, nextIndex))
                {
                    // Add new Node to openSet
                    double distToGoal = distanceBetweenFreeVertices(nextIndex, goalIndex);
                    GraphNode next = {nextIndex, currentIndex, newCost, distToGoal};
                    openSet.push_back(next);
                    sort(openSet.begin(), openSet.end(), greater<GraphNode>());
                }
            }
        }

        int parentInd;
        bool notDone = true;
        vector<int> pathIndices;

        if (goalFound)
        {
            ROS_INFO("Path to goal found, now back tracking");
            parentInd = goalIndex;
        }
        else
        {
            ROS_INFO("No path to goal found, changing goal to nearest point");
            parentInd = currentIndex;
        }

        // Go back through the chain to find the path
        while (notDone)
        {
            pathIndices.push_back(parentInd);
            int i = getNodeIndex(closedSet, parentInd);
            if (i == -1)
            {
                ROS_INFO("Node %d could not be found!!", parentInd);
                break;
            }
            //ROS_INFO("At node %d, moving to node %d", parentInd, closedSet[i].parentIndex);
            parentInd = closedSet[i].parentIndex;
            if (parentInd == startIndex)
            {
                notDone = false;
            }
        }

        pathIndices.push_back(startIndex);
        return (pathIndices);
    }
};
