#include <pluginlib/class_list_macros.h>
#include <cmath>
#include "global_planner.h"
#include <pcl_ros/publisher.h>
#include <vector>
#include <iterator>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

struct Vertex
{
    double xPosition;
    double yPosition;
    bool inFreeSpace;
    vector<int> neighbourIndices;
    vector<double> neighbourDistances;
};

struct Edge
{
    Vertex vertex1;
    Vertex vertex2;
    double length;
};

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

    const unsigned int SAMPLE_SIZE = 500;

    // Each cell of the cost map is a value from 0 to 255
    // Anything above this threshold is not consisered for path making
    const unsigned char COST_THRESHOLD = 50;

    // The number of nearest vertices to attempt to connect to when building network
    const unsigned int N_NEAREST = 20;

    // PRM nodes and edges
    Vertex vertices[SAMPLE_SIZE];
    Vertex* freeVertices;
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

    bool checkCost(int x, int y)
    {
        unsigned char cost = globalCostmap->getCost(x, y);
        //ROS_INFO("cost at location: %d, %d: %d", x, y, cost);
        if (cost >= COST_THRESHOLD)
        {
            return (true);
        }
        return (false);
    }

    //
    bool Bresenham(int x1, int y1, int const x2, int const y2)
    {
        int delta_x(x2 - x1);
        // if x1 == x2, then it does not matter what we set here
        signed char const ix((delta_x > 0) - (delta_x < 0));
        delta_x = std::abs(delta_x) << 1;

        int delta_y(y2 - y1);
        // if y1 == y2, then it does not matter what we set here
        signed char const iy((delta_y > 0) - (delta_y < 0));
        delta_y = std::abs(delta_y) << 1;

        //plot(x1, y1);
        if (checkCost(x1, y1)) { return (true); }

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
                // else do nothing

                error += delta_y;
                x1 += ix;

                //plot(x1, y1);
                if (checkCost(x1, y1)) { return (true); }
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
                // else do nothing

                error += delta_x;
                y1 += iy;

                //plot(x1, y1);
                if (checkCost(x1, y1)) { return (true); }
            }
        }

        return (false);
    }



    // Returns true if a collision is found
    bool checkLineCollision(Vertex pt1, Vertex pt2)
    {
        int x1, y1, x2, y2;
        globalCostmap->worldToMapEnforceBounds(pt1.xPosition, pt1.yPosition, x1, y1);
        globalCostmap->worldToMapEnforceBounds(pt2.xPosition, pt2.yPosition, x2, y2);

        return (Bresenham(x1, y1, x2, y2));




    }

    double distanceBetweenVertices(Vertex pt1, Vertex pt2)
    {
        double xDiff =  pt2.xPosition - pt1.xPosition;
        double yDiff =  pt2.yPosition - pt1.yPosition;

        double distance = sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    double distanceBetweenFreeVertices(int ind1, int ind2)
    {
        double xDiff =  freeVertices[ind1].xPosition - freeVertices[ind1].xPosition;
        double yDiff =  freeVertices[ind2].yPosition - freeVertices[ind2].yPosition;

        double distance = sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    template <typename T>
    vector<int> sort_indexes(const vector<T> &v) {

        // initialize original index locations
        vector<size_t> idx(v.size());
        for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;

        // sort indexes based on comparing values in v
        sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

        return idx;
    }

    template <typename T>
    vector<int> sort_indexes2(const vector<T> &v) {

        // initialize original index locations
        vector<int> idx(v.size());
        for (int i = 0; i != idx.size(); ++i) idx[i] = i;

        // sort indexes based on comparing values in v
        sort(idx.begin(), idx.end(),
        [&v](int i1, int i2) {return v[i1] < v[i2];});

        return idx;
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // Get the map dimensions in terms of cells

        //globalCostmapROS = costmap_ros;
        globalCostmap = costmap_ros->getCostmap();

        mapSizeCX = globalCostmap->getSizeInCellsX();
        mapSizeCY = globalCostmap->getSizeInCellsY();
        mapSizeMX = globalCostmap->getSizeInMetersX();
        mapSizeMY = globalCostmap->getSizeInMetersY();
        mapOriginX = globalCostmap->getOriginX();
        mapOriginY = globalCostmap->getOriginY();

        ROS_INFO("## - Initializing global planner - ##");
        ROS_INFO("## - Costmap origin: %f, %f", mapOriginX, mapOriginY);

        // Publisher
        ros::NodeHandle n;
        //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        plan_pub = n.advertise<nav_msgs::Path>("planB", 1);
        viz_pub = n.advertise<visualization_msgs::Marker>("PRM_Network", 1);
    }





    class CompareDist
    {
    public:
        bool operator()(pair<int, double> n1,pair<int, double> n2)
        {
            return n1.second > n2.second;
        }
    };

    struct OpenNode
    {
        int index;
        int parentIndex;
        double costToReach;
        double costToGoal;

        bool operator > (const OpenNode& n) const
        {
            return ((costToReach + costToGoal) > (n.costToReach + n.costToGoal));
        }
    };

    class CompareNode
    {
    public:
        bool operator()(OpenNode n1, OpenNode n2)
        {
            return (n1.costToReach + n1.costToGoal) > (n2.costToReach + n2.costToGoal);
        }
    };

    bool setContains(vector<OpenNode> set, int index)
    {
        //priority_queue<OpenNode, vector<OpenNode>, CompareNode> setCopy = new
        for (int i = 0; i < set.size(); i++)
        {
            if (set[i].index == index)
            {
                return (true);
            }
        }
        return (false);
    }

    int getNodeIndex(vector<OpenNode> set, int index)
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


//vector<int>
    vector<int> aStarSearch(int startIndex, int goalIndex)
    {
        vector<OpenNode> openSet;
        vector<OpenNode> closedSet;

        OpenNode start = {startIndex, startIndex, 0.0, distanceBetweenFreeVertices(startIndex, goalIndex)};
        openSet.push_back(start);

        OpenNode current;
        int currentIndex;
        ROS_INFO("Starting search from %d to %d", startIndex, goalIndex);
        while (openSet.size() > 0)
        {
            current = openSet.back();
            openSet.pop_back();
            currentIndex = current.index;

            // Add to closed list
            closedSet.push_back(current);
            if (currentIndex == goalIndex)
            {
                break;
            }


            int neighbourCount = freeVertices[currentIndex].neighbourIndices.size();

            // Loop through all neighbours
            for (int i = 0; i < neighbourCount; i++)
            {
                int nextIndex = freeVertices[currentIndex].neighbourIndices[i];
                double newCost = current.costToReach + freeVertices[currentIndex].neighbourDistances[i];

                ROS_INFO("Neighbour %d (%d) of vertex %d has a new cost of %f", i, nextIndex, currentIndex, newCost);

                if (setContains(openSet, nextIndex))
                {
                    if (newCost < openSet[getNodeIndex(openSet, nextIndex)].costToReach)
                    {
                        // Remove from open set
                        openSet[getNodeIndex(openSet, nextIndex)].index = -1;
                    }
                }
                if (setContains(closedSet, nextIndex))
                {
                    if (newCost < closedSet[getNodeIndex(closedSet, nextIndex)].costToReach)
                    {
                        // Remove from open set
                        closedSet[getNodeIndex(closedSet, nextIndex)].index = -1;
                    }
                }
                if (!setContains(openSet, nextIndex) && !setContains(closedSet, nextIndex))
                {
                    // Add new Node to openSet
                    double distToGoal = distanceBetweenFreeVertices(nextIndex, goalIndex);
                    OpenNode next = {nextIndex, currentIndex, newCost, distToGoal};
                    openSet.push_back(next);
                    sort(openSet.begin(), openSet.end(), greater<OpenNode>());
                }
            }
        }

        ROS_INFO("Path found, now back tracking");
        // Go back through the chain to find the path

        bool notDone = true;
        vector<int> pathIndices;

        int parentInd = goalIndex;

        while (notDone)
        {
            pathIndices.push_back(parentInd);
            int i = getNodeIndex(closedSet, parentInd);
            if (i == -1)
            {
                ROS_INFO("Node %d could not be found!!", parentInd);
                break;
            }
            ROS_INFO("At node %d, moving to node %d", parentInd, closedSet[i].parentIndex);
            parentInd = closedSet[i].parentIndex;
            if (parentInd == startIndex)
            {
                notDone = false;
            }
        }

        pathIndices.push_back(startIndex);

        return (pathIndices);
    }

/*
    struct Vertex
    {
        double xPosition;
        double yPosition;
        bool inFreeSpace;
        vector<int> neighbourIndices;
        vector<double> neighbourDistances;
    };
*/


    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan )
    {
        // First add the start and goal poses as vertices
        vertices[0].xPosition = start.pose.position.x;
        vertices[0].yPosition = start.pose.position.y;
        vertices[0].inFreeSpace = true;

        vertices[1].xPosition = goal.pose.position.x;
        vertices[1].yPosition = goal.pose.position.y;
        vertices[1].inFreeSpace = true;

        ROS_INFO("Start position: %f, %f", vertices[0].xPosition, vertices[0].yPosition);
        ROS_INFO("Goal position: %f, %f", vertices[1].xPosition, vertices[1].yPosition);

        // Generate some random points on the map and check for obstacles
        srand(2);
        unsigned int freeVertexCount = 2;
        for (int i = 2; i < SAMPLE_SIZE; i++)
        {
            // Create random point
            //vertices[i].xPosition = rand() % mapSizeX;
            //vertices[i].yPosition = rand() % mapSizeY;
            double randX = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/mapSizeMX)) + mapOriginX;
            double randY = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/mapSizeMY)) + mapOriginY;
            vertices[i].xPosition = randX;
            vertices[i].yPosition = randY;

            // Grid cell that the point is in
            int cellX;
            int cellY;
            globalCostmap->worldToMapEnforceBounds(randX, randY, cellX, cellY);



            //ROS_INFO("## - New random point at: %f, %f", randX, randY);
            //ROS_INFO("## - New random point at: %d, %d", cellX, cellY);

            unsigned char cost = globalCostmap->getCost(cellX, cellY);

            //ROS_INFO("## - Cell cost: %d", cost);
            // Check if it is a free cell
            if (cost < COST_THRESHOLD)
            {
                vertices[i].inFreeSpace = true;
                freeVertexCount++;



                ROS_INFO("Node pushed: %f, %f", vertices[i].xPosition, vertices[i].yPosition);

                // Display stuff
                /*
                geometry_msgs::Point p;
                p.x = randX;
                p.y = randY;
                p.z = 2.0;
                drawPoints.points.push_back(p);
                */

            }
        }


        ROS_INFO("No of free Verts: %d", freeVertexCount);

        // Create a list of vertices in free space
        freeVertices = new Vertex[freeVertexCount];
        unsigned int count = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++)
        {
            if (vertices[i].inFreeSpace == true)
            {
                freeVertices[count] = vertices[i];
                count++;
            }
        }

        // Check through all free vertices and generate a network
        for (int i = 0; i < freeVertexCount; i++)
        {
            vector<double> distances(freeVertexCount, 0.0);

            // Find nearest vertices
            for (int j = i + 1; j < freeVertexCount; j++)
            {
                // Skip the same vertex
                //if (i == j)
                //{
                //    continue;
                //}
                distances[j] = distanceBetweenVertices(freeVertices[i], freeVertices[j]);
            }

            // sort by distance
            vector<int> indices = sort_indexes2(distances);


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
            for (int j = i + 1; j < nClosest; j++)
            {
                Vertex v2 = freeVertices[indices[j]];
                // Skip the same vertex
                if (i == indices[j])
                {
                    continue;
                }

                if (!checkLineCollision(v1, v2))
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

        // display all vertex connections
        for (int i = 0; i < freeVertexCount; i++)
        {
            ROS_INFO("Vertex %d is connected to:", i);
            for (int j = 0; j < freeVertices[i].neighbourIndices.size(); j++)
            {
                ROS_INFO(" - %d with dist: %f", freeVertices[i].neighbourIndices[j], freeVertices[i].neighbourDistances[j]);
            }
        }



        // Search network using A*

        vector<int> pathFromGoal = aStarSearch(0, 1);

        // Finally create nav path from the A* path
        for (int i = pathFromGoal.size()-1; i >= 0; i--)
        {
            Vertex v = freeVertices[pathFromGoal[i]];

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

        viz_pub.publish(edgeLines);
        plan_pub.publish(rvizPath);

        ROS_INFO("#### - New global path served fresh - ####");

        return true;
    }
};
