/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <iterator>

using std::string;
using namespace std;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {
	class GlobalPlanner : public nav_core::BaseGlobalPlanner {
		public:

		GlobalPlanner();
		GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan );


	};

	// --- Structs
	// Vertex of the PRM graph
	struct Vertex
	{
	    double xPosition;
	    double yPosition;
	    bool inFreeSpace;
	    vector<int> neighbourIndices;
	    vector<double> neighbourDistances;
	};

	// Edges of the PRM graph, currently only used for plotting
	struct Edge
	{
	    Vertex vertex1;
	    Vertex vertex2;
	    double length;
	};

	// Used for A* search only
	struct GraphNode
	{
		int index;
		int parentIndex;
		double costToReach;
		double costToGoal;

		bool operator > (const GraphNode& n) const
		{
			return ((costToReach + costToGoal) > (n.costToReach + n.costToGoal));
		}
	};

	// --- Helper methods
	bool isOccupied(double x, double y, unsigned char threshold);
	bool isOccupied(int x, int y, unsigned char threshold);
	bool isLineOccupied(int x1, int y1, int const x2, int const y2);
	bool isLineOccupied(Vertex pt1, Vertex pt2);

	template <typename T>
	vector<int> sortIndexes(const vector<T> &v);

	double distanceBetweenVertices(Vertex pt1, Vertex pt2);
	double distanceBetweenFreeVertices(int ind1, int ind2);

	bool setContains(vector<GraphNode> set, int index);
	int getNodeIndex(vector<GraphNode> set, int index);
	vector<int> aStarSearch(int startIndex, int goalIndex);
};
#endif
