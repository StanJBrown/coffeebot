/** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <vector>
 #include <set>

 using std::string;
 using namespace std;

 #ifndef ASTAR_GLOBAL_PLANNER_CPP
 #define ASTAR_GLOBAL_PLANNER_CPP

 // Structure to store a node in the frontier
struct cellInfo{
  float fval; // previous cost + cost to cell + obstacle cost + heuristic
  float gval; // previous cost + obstacle cost
  int current; // current cell index
  int cameFrom; // index of cell where we came from
};

// Comparison function for multisetstruct comp{
struct comp{
    inline bool operator()(const cellInfo& left,const cellInfo& right){
        return left.fval < right.fval;
    }
};

namespace astar_cb_global_planner {

   class AStar_CB_GlobalPlanner : public nav_core::BaseGlobalPlanner {

    public:
      bool  CMinitialized_;
      float CMoriginX;
      float CMoriginY;
      float CMresolution; // meters/cell 0.05 how many meters in a cell
      int   CMwidth; // number of cells in the cost map
      int   CMheight; // number of cells in the cost map
      int cellArrayWidth,cellArrayHeight;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;

      AStar_CB_GlobalPlanner();
      AStar_CB_GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /** overridden classes from interface nav_core::BaseGlobalPlanner **/
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
      bool isCMCoordInsideMap(float x, float y);
      bool isCellValid(int cellIndex);

      // Transformations
      void WCoordToCMCoord(float& x, float& y);
      int  CMCoordToCellIdx(float x, float y);
      int  CellIdxToCellRowID(int index){//get the row ID from cell index
           return index/cellArrayWidth;}
      int  CellIdxToCellColID(int index){//get colunm ID from cell index
           return index%cellArrayWidth;}

      // Dimension along x(col) first then y(row)
      int  CellRowColToCellIdx(int col,int row){
           return (row*cellArrayWidth)+col;}
      void CellIdxToCMCoord(int index, float& x, float& y);
      void CMCoordToWCoord(float& x, float& y);

      // Stores the plan cell indices
      bool AStarPlanner(int startCell, int goalCell);
      int AddFreeNeighbors(multiset<cellInfo,comp>& frontier, int goalCellIdx);

      // Print
      void displayCellIdx(int col, int row){
          cout << "Cell(row,col) (" << row << "," << col << ") " <<
          "Cell idx: " << CellRowColToCellIdx(col,row) << endl;
      }
      void displayRowCol(int cellIdx){
          cout << "Cell Idx: " << cellIdx << " Cell(row,col) (" <<
          CellIdxToCellRowID(cellIdx) << "," << CellIdxToCellColID(cellIdx)
          << ")" << endl;
      }

      // Move cost from one cell indedx to the other index
      // As of now same as the heuristic cost
      float moveCost(int fromIdx, int toIdx){
        int x1=CellIdxToCellRowID(toIdx);
        int y1=CellIdxToCellColID(toIdx);
        int x2=CellIdxToCellRowID(fromIdx);
        int y2=CellIdxToCellColID(fromIdx);
        return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
      }

      float heuristicCost(int cellIdx, int goalIdx){ // get h cost
        int x1=CellIdxToCellRowID(goalIdx);
        int y1=CellIdxToCellColID(goalIdx);
        int x2=CellIdxToCellRowID(cellIdx);
        int y2=CellIdxToCellColID(cellIdx);
        return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
      }

      void letsSleep();
  };
 };
 #endif
