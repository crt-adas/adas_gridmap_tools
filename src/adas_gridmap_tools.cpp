#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_utils.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <limits>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


boost::shared_ptr<visualization_msgs::MarkerArray> arrowMarkers(new visualization_msgs::MarkerArray());
int markerIndex = 0;

void getRecInfo(visualization_msgs::Marker& line_strip)
{
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.a = 1; line_strip.color.r = 0; line_strip.color.g = 1; line_strip.color.b = 1;
  line_strip.id = ++markerIndex;
  line_strip.header.frame_id = "map";
  line_strip.action =  visualization_msgs::Marker::ADD;
}

void sendArrow(const geometry_msgs::PoseStamped& arrowPose, bool init)
{
  double diameterArrow = 0.2;

  visualization_msgs::Marker nodeArrowMarker;

  nodeArrowMarker.header.frame_id = "map";
  nodeArrowMarker.header.stamp = ::ros::Time::now();
  nodeArrowMarker.action =  visualization_msgs::Marker::ADD;
  nodeArrowMarker.type = visualization_msgs::Marker::ARROW;

  nodeArrowMarker.color.a = 1.0;
  if(init)
  { nodeArrowMarker.color.b = 1.0; nodeArrowMarker.color.r = 0.0; nodeArrowMarker.color.g = 0.0; }
  if(!init)
  { nodeArrowMarker.color.b = 0.0; nodeArrowMarker.color.r = 0.0; nodeArrowMarker.color.g = 1.0; }

  nodeArrowMarker.id = ++markerIndex;
  
  nodeArrowMarker.scale.x = 1.0;
  nodeArrowMarker.scale.y = diameterArrow;
  nodeArrowMarker.scale.z = diameterArrow;

  nodeArrowMarker.pose.position = arrowPose.pose.position;
  nodeArrowMarker.pose.orientation = arrowPose.pose.orientation;

  arrowMarkers->markers.push_back(nodeArrowMarker);
  
}


// □□□□□□□▧▧▧▧▧▧▧
// □□□□□□□▧▧▧▧▧▧▧
// □□□□□□□▧▧▧▧▧▧▧
// ▧▧▧▧▧▧▧▧▧▧▧▧▧▧
// ▧▧▧▧▧▧▧▧▧▧▧▧▧▧
// ▧▧▧▧▧▧▧▧▧▧▧▧▧▧
/** Scan the gridmap to find biggest vertex of rectangle, check how big can the rectangle expand in direction of empty cubes in shape above.
*  @param[out] MaxVal: return value in res unit 
*  @param[in] init: pose of origin of rectangle (not center necceserly), 
*  @param[in] L: length of the line to scan along 
*  @param[in] newMap: map
*  @param[in] res: map resolution, or size of each step in scanning the map in meters 
*  @param[in] right: true = scan right side of pose, false = scan left side of pose.
*  @param[in] front: true = scan front side of pose, false = scan back side of pose.
*/
void pacManRect(int& MaxVal,const geometry_msgs::Pose2D& init, const int L, const grid_map::GridMap& newMap,const float res, bool right, bool front)
{
  int safetyDis = 2 ; //push back the rectangle from obstacle in res units for safety
  geometry_msgs::Point p;
  int r = right ? +1 : -1; //sign of direction toward right or left relative to pose
  int f = front ? +1 : -1; //sign of direction toward front or back relative to pose
  int j = 0;
  float lenX = (newMap.getLength().x() - 1)/2; //get cordinates of 
  float lenY = (newMap.getLength().y() - 1)/2;
  
  for(int i=0; i <= L; i++) //for the length between pose and intersection by res steps 
  {
    do{ // expand to right or left until obstacle or end of map, then break
    p.x = init.x + r*res*j*cos(init.theta + (M_PI/2)) + f*res*i*cos(init.theta);  //centerX + right + front
    p.y = init.y + r*res*j*sin(init.theta + (M_PI/2)) + f*res*i*sin(init.theta);  //centerY + right + front
    j++;
    if ((abs(p.x) > lenX) || (abs(p.y) > lenY)) // break if end of map
      break;
    }
    while(!newMap.atPosition("elevation", {p.x, p.y})); //while cell is obstacle free, false = free cell. 

    MaxVal = std::min(MaxVal,j); //keep the shortest distance in res units to obstacle in every step forward
    j = 0;
  }
  
  MaxVal = MaxVal - safetyDis;

} 


/** check if given point is inside any of three rectangle
*  @param[in] point: the given point to check
*  @param[in] center: a known point inside rectangle
*  @param[in] p1_p2: two points on the line
*/
bool checkLine(const geometry_msgs::PointStamped& point,const geometry_msgs::Pose2D& center,
  const geometry_msgs::Point& p1,const geometry_msgs::Point& p2)
{
  
  bool signCenterSide = std::signbit((p2.x - p1.x) * (center.y - p1.y) - (center.x - p1.x) * (p2.y - p1.y)); //true = negative
  bool signPointSide = std::signbit((p2.x - p1.x) * (point.point.y - p1.y) - (point.point.x - p1.x) * (p2.y - p1.y)); //true = negative 

  if (signCenterSide == signPointSide) // check if center and point are in same side of line
    return true;
  else
    return false;
    
}


/** check if given point is inside any of three rectangle
*  @param[in] point: the given point to check
*  @param[in] center: a known point inside rectangle
*  @param[in] p1_p4: cordinates four edges of rectangle
*/
bool CheckPoint(const geometry_msgs::PointStamped& point,const geometry_msgs::Pose2D& center,
  const geometry_msgs::Point& p1,const geometry_msgs::Point& p2,const geometry_msgs::Point& p3,const geometry_msgs::Point& p4)
{
  bool line1 = checkLine(point,center,p1,p2); //true = on the correct side 
  bool line2 = checkLine(point,center,p2,p4);
  bool line3 = checkLine(point,center,p4,p3);
  bool line4 = checkLine(point,center,p3,p1);
  if(line1 && line2 && line3 && line4)
    return true;
  else
    return false;
}














int main(int argc, char **argv)
{
  using namespace ast;
  using namespace ast::ros;
  using namespace geometry_msgs;
  using namespace nav_msgs;
  using namespace grid_map;
  using namespace visualization_msgs;
  
  ::ros::init(argc, argv, "adas_gridmap_tools");

  
  NodeHandle nh;
  auto occupancyGridIn = nh.Input<OccupancyGrid>("/occupancy_map_filtered");
  auto gridMapOut = nh.Output<grid_map_msgs::GridMap>("/grid_map");
  auto initPoseIn = nh.Input<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");

  auto pointIn = nh.Input<geometry_msgs::PointStamped>("/clicked_point");

  auto goalPoseIn = nh.Input<geometry_msgs::PoseStamped>("/move_base_simple/goal");
  auto MarkersOut = nh.Output<visualization_msgs::MarkerArray>("/gridmap_tools_marker_array");
  
  boost::shared_ptr<grid_map_msgs::GridMap> gridMapMsg(new grid_map_msgs::GridMap());
  GridMap gridMap, newMap;
  std::string gridMapLayerName = "elevation";
  

  bool gridMapUpdated = false;
  while(!gridMapUpdated)
  {
    ::ros::spinOnce();

    auto grid = occupancyGridIn();
    if(grid.info.height > 0 && grid.info.width > 0)
    {                   
        ROS_INFO_STREAM("Occupancy grid was received!");
        for(auto& cell : grid.data)
        {
          if(cell < 0)
            cell = 0;
        }

        ROS_INFO_STREAM("Unknowns converted to free");
        if(GridMapRosConverter::fromOccupancyGrid(grid, gridMapLayerName, gridMap))
        {                
            //GridMapCvProcessing::changeResolution(gridMap, newMap, 0.3);
            //newMap.setPosition(Position(0.0, 0.0));
            bool succ;
            newMap = gridMap.getSubmap(Position(0.0, 0.0), Length(80, 80), succ);
            GridMapRosConverter::toMessage(newMap, *gridMapMsg);
            gridMapOut(gridMapMsg);
            gridMapUpdated = true;
        }
    }
  }

    
  double oldInitx = 0.0;
  double oldGoalx = 0.0;
  double oldPointx = 0.0;

  PoseStamped poseArrowGoal, poseArrowInit;
  bool rectangleDone = false;
  bool initReceived = false;
  bool goalReceived = false;

  Pose2D init, goal, inter;
  Point p1o,p2o,p3o,p4o; // rectangle No.1 4points
  Point p1oo,p2oo,p3oo,p4oo; // rectangle No.2 4points
  Point p1ooo,p2ooo,p3ooo,p4ooo; // rectangle No.3 4points

  ::ros::Rate loop_rate(10);
  while (::ros::ok())
  {
    auto initPos = initPoseIn();
    auto goalPose = goalPoseIn();
    auto pointC = pointIn();
    bool ifInit;
    


    if(pointC.point.x != oldPointx)
    {
      if (!rectangleDone)
      {
        ROS_INFO_STREAM_ONCE("First enter the inial and final pose!");

      }else
      {
        oldPointx = pointC.point.x;
        bool inRecInit = CheckPoint(pointC,init,p1o,p2o,p3o,p4o); //true = inside the rectangle
        bool inRecGoal = CheckPoint(pointC,goal,p1oo,p2oo,p3oo,p4oo); //true = inside the rectangle
        bool inRecInter = CheckPoint(pointC,inter,p1ooo,p2ooo,p3ooo,p4ooo); //true = inside the rectangle
        ROS_INFO_STREAM("rec1: "<< inRecInit<<" ,rec2: " <<inRecGoal<<" ,rec3: "<< inRecInter<<"");
        if(inRecInit || inRecGoal || inRecInter)
          ROS_INFO_STREAM("Point("<< pointC.point.x <<","<< pointC.point.y <<") is: INSIDE");
        else
          ROS_INFO_STREAM("Point("<< pointC.point.x <<","<< pointC.point.y <<") is: OUTSIDE");
      }
      

    }



    if(initPos.pose.pose.position.x != oldInitx) // Check if new initial pose was received, publish the marker
    {      
      if(rectangleDone)  
      {
        rectangleDone = false;
      }
      ROS_INFO_STREAM("Initial Pose Received!");
      oldInitx = initPos.pose.pose.position.x;
      poseArrowInit.pose = initPos.pose.pose;
      ifInit = true;
      sendArrow(poseArrowInit, ifInit);
      initReceived = true;
      MarkersOut(arrowMarkers);
    }

    if(goalPose.pose.position.x != oldGoalx) // Check if new final pose was received, publish the marker
    {    
      if(rectangleDone)  
      {
        rectangleDone = false;
      }               
      ROS_INFO_STREAM("Goal Pose Received!");
      oldGoalx = goalPose.pose.position.x;
      poseArrowGoal = goalPose;
      ifInit = false;
      sendArrow(poseArrowGoal, ifInit);
      goalReceived = true;
      MarkersOut(arrowMarkers);

    }

    if( goalReceived && !initReceived) 
      ROS_INFO_STREAM_ONCE("Waiting for Inital pose... ");

    if( !goalReceived && initReceived)
      ROS_INFO_STREAM_ONCE("Waiting for Final pose... ");



    if(goalReceived && initReceived) //if both inital and final pose received
    {
      
      Marker line_strip1, line_strip2, line_strip3 ;
      getRecInfo(line_strip1);
      getRecInfo(line_strip2);
      getRecInfo(line_strip3);
      
      

      float m1, c1, m2, c2; //1 = inital, 2 = goal
      int j1Min = std::numeric_limits<int>::max(), j2Min = j1Min, j3Min = j1Min, j4Min = j1Min;
      int jj1Min = std::numeric_limits<int>::max(), jj2Min = jj1Min, jj3Min = jj1Min, jj4Min = jj1Min;
      int jjj1Min = std::numeric_limits<int>::max(), jjj2Min = jjj1Min, jjj3Min = jjj1Min, jjj4Min = jjj1Min;

      float resScale = 0.9; // between 0.0 -> 1.0 , not needed, but just to be sure all cells are checked.
      float res = newMap.getResolution()*resScale;
      

      float offsetFront = (5.9)/res; // in meters convert to res unit
      float offsetBack = (2)/res;           //in meters convert to res unit
      
      init.x = poseArrowInit.pose.position.x;
      init.y = poseArrowInit.pose.position.y;
      init.theta = tf::getYaw(poseArrowInit.pose.orientation);
      
      goal.x = poseArrowGoal.pose.position.x;
      goal.y = poseArrowGoal.pose.position.y;
      goal.theta = tf::getYaw(poseArrowGoal.pose.orientation);

      
      //calculating intersection between init and final pose
      m1 = tan(init.theta);
      c1 = init.y - ( m1 * init.x);
      m2 = tan(goal.theta);
      c2 = goal.y - ( m2 * goal.x);

      if((m1 - m2) == 0)
        ROS_INFO_STREAM("No Intersection between Inital and Final pose");
      else
      {
        inter.x = (c2 - c1) / (m1 - m2);
        inter.y = m1 * inter.x + c1;
        ROS_INFO_STREAM("Intersection at: " << inter.x << " ," << inter.y <<"");

        if(newMap.atPosition("elevation", {inter.x, inter.y}))
          ROS_INFO_STREAM("Intersection is not collision free!");
        else
        {
          int initL = sqrt(pow(inter.x - init.x, 2) +  pow(inter.y - init.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from inital pose to Intersection: " << initL*res << " m");
          int goalL = sqrt(pow(inter.x - goal.x, 2) +  pow(inter.y - goal.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from goal pose to Intersection: " << goalL*res << " m");
          
          
          // Inital rectangle
          pacManRect(j1Min, init, initL, newMap, res, true, true); //center + right + front
          pacManRect(j3Min, init, initL, newMap, res, false, true); //center + left + front
          pacManRect(j2Min, init, offsetBack, newMap, res, true, false); //center + right + back
          pacManRect(j4Min, init, offsetBack, newMap, res, false, false); //center + left + back

          p1o.x = init.x + res*std::min(j1Min,j2Min)*cos(init.theta + (M_PI/2)) + res*initL*cos(init.theta);  //centerX + right + front
          p1o.y = init.y + res*std::min(j1Min,j2Min)*sin(init.theta + (M_PI/2)) + res*initL*sin(init.theta);  //centerY + right + front

          p3o.x = init.x - res*std::min(j3Min,j4Min)*cos(init.theta + (M_PI/2)) + res*initL*cos(init.theta);  //centerX + left + front
          p3o.y = init.y - res*std::min(j3Min,j4Min)*sin(init.theta + (M_PI/2)) + res*initL*sin(init.theta);  //centerY + left + front

          p2o.x = init.x + res*std::min(j1Min,j2Min)*cos(init.theta + (M_PI/2)) - res*offsetBack*cos(init.theta);  //centerX + right + back 
          p2o.y = init.y + res*std::min(j1Min,j2Min)*sin(init.theta + (M_PI/2)) - res*offsetBack*sin(init.theta);  //centerY + right + back 
 
          p4o.x = init.x - res*std::min(j3Min,j4Min)*cos(init.theta + (M_PI/2)) - res*offsetBack*cos(init.theta);  //centerX + left + back 
          p4o.y = init.y - res*std::min(j3Min,j4Min)*sin(init.theta + (M_PI/2)) - res*offsetBack*sin(init.theta);  //centerY + left + back 

          line_strip1.points.push_back(p1o);
          line_strip1.points.push_back(p2o);
          line_strip1.points.push_back(p4o);
          line_strip1.points.push_back(p3o);
          line_strip1.points.push_back(p1o);
          arrowMarkers->markers.push_back(line_strip1);


          //final rectangle
          pacManRect(jj2Min, goal, goalL, newMap, res, true, false); //center + right + back
          pacManRect(jj4Min, goal, goalL, newMap, res, false, false); //center + left + back
          pacManRect(jj1Min, goal, offsetFront, newMap, res, true, true); //center + right + front
          pacManRect(jj3Min, goal, offsetFront, newMap, res, false, true); //center + left + front

          p2oo.x = goal.x + res*std::min(jj1Min,jj2Min)*cos(goal.theta + (M_PI/2)) - res*goalL*cos(goal.theta);  //centerX + right + back
          p2oo.y = goal.y + res*std::min(jj1Min,jj2Min)*sin(goal.theta + (M_PI/2)) - res*goalL*sin(goal.theta);  //centerY + right + back

          p4oo.x = goal.x - res*std::min(jj3Min,jj4Min)*cos(goal.theta + (M_PI/2)) - res*goalL*cos(goal.theta);  //centerX + left + back
          p4oo.y = goal.y - res*std::min(jj3Min,jj4Min)*sin(goal.theta + (M_PI/2)) - res*goalL*sin(goal.theta);  //centerY + left + back

          p1oo.x = goal.x + res*std::min(jj1Min,jj2Min)*cos(goal.theta + (M_PI/2)) + res*offsetFront*cos(goal.theta);  //centerX + right + front 
          p1oo.y = goal.y + res*std::min(jj1Min,jj2Min)*sin(goal.theta + (M_PI/2)) + res*offsetFront*sin(goal.theta);  //centerY + right + front 
 
          p3oo.x = goal.x - res*std::min(jj3Min,jj4Min)*cos(goal.theta + (M_PI/2)) + res*offsetFront*cos(goal.theta);  //centerX + left + front 
          p3oo.y = goal.y - res*std::min(jj3Min,jj4Min)*sin(goal.theta + (M_PI/2)) + res*offsetFront*sin(goal.theta);  //centerY + left + front 

          line_strip2.points.push_back(p1oo);
          line_strip2.points.push_back(p2oo);
          line_strip2.points.push_back(p4oo);
          line_strip2.points.push_back(p3oo);
          line_strip2.points.push_back(p1oo);
          arrowMarkers->markers.push_back(line_strip2);


          //Intersection rectangle
          inter.theta = ((goal.theta - init.theta)/2) + init.theta;
          int front3 = std::min(j3Min,jj2Min);
          int back3 = std::min(j1Min,jj4Min);

          pacManRect(jjj1Min, inter, front3, newMap, res, true, true); //center + right + front
          pacManRect(jjj3Min, inter, front3, newMap, res, false, true); //center + left + front
          pacManRect(jjj2Min, inter, back3, newMap, res, true, false); //center + right + back
          pacManRect(jjj4Min, inter, back3, newMap, res, false, false); //center + left + back

          p2ooo.x = inter.x + res*std::min(jjj1Min,jjj2Min)*cos(inter.theta + (M_PI/2)) - res*back3*cos(inter.theta);  //centerX + right + back
          p2ooo.y = inter.y + res*std::min(jjj1Min,jjj2Min)*sin(inter.theta + (M_PI/2)) - res*back3*sin(inter.theta);  //centerY + right + back

          p4ooo.x = inter.x - res*std::min(jjj3Min,jjj4Min)*cos(inter.theta + (M_PI/2)) - res*back3*cos(inter.theta);  //centerX + left + back
          p4ooo.y = inter.y - res*std::min(jjj3Min,jjj4Min)*sin(inter.theta + (M_PI/2)) - res*back3*sin(inter.theta);  //centerY + left + back

          p1ooo.x = inter.x + res*std::min(jjj1Min,jjj2Min)*cos(inter.theta + (M_PI/2)) + res*front3*cos(inter.theta) ;  //centerX + right + front
          p1ooo.y = inter.y + res*std::min(jjj1Min,jjj2Min)*sin(inter.theta + (M_PI/2)) + res*front3*sin(inter.theta) ;  //centerY + right + front
 
          p3ooo.x = inter.x - res*std::min(jjj3Min,jjj4Min)*cos(inter.theta + (M_PI/2)) + res*front3*cos(inter.theta) ;  //centerX + left + front
          p3ooo.y = inter.y - res*std::min(jjj3Min,jjj4Min)*sin(inter.theta + (M_PI/2)) + res*front3*sin(inter.theta) ;  //centerY + left + front

          line_strip3.points.push_back(p1ooo);
          line_strip3.points.push_back(p2ooo);
          line_strip3.points.push_back(p4ooo);
          line_strip3.points.push_back(p3ooo);
          line_strip3.points.push_back(p1ooo);
          arrowMarkers->markers.push_back(line_strip3);

          MarkersOut(arrowMarkers);









        }

      }
      
      goalReceived = false;
      initReceived = false;
      rectangleDone = true;

    }

    gridMapOut(gridMapMsg);
    ::ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}

