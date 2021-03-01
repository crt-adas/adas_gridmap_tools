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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



typedef unsigned long int uintl;




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
//               GridMapCvProcessing::changeResolution(gridMap, newMap, 0.3);
//                newMap.setPosition(Position(0.0, 0.0));
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
  bool rectangleDone = false;
  PoseStamped poseArrowGoal;
  PoseStamped poseArrowInit;
  bool initReceived = false;
  bool goalReceived = false;

  ::ros::Rate loop_rate(10);
  while (::ros::ok())
  {
    auto initPos = initPoseIn();
    auto goalPose = goalPoseIn();
    
    bool ifInit;
    
    

    if(initPos.pose.pose.position.x != oldInitx)
    {      
      if(rectangleDone)  
      {
        arrowMarkers->markers.clear();
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

    if(goalPose.pose.position.x != oldGoalx)
    {    
      if(rectangleDone)  
      {
        arrowMarkers->markers.clear();
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



    if(goalReceived && initReceived)
    {
      
      Marker line_strip1, line_strip2, line_strip3 ;
      getRecInfo(line_strip1);
      getRecInfo(line_strip2);
      getRecInfo(line_strip3);

      Pose2D init, goal, inter;
      Point p;
      std::vector<Point> rec1, rec2 ,rec3;
      Point p1o,p2o,p3o,p4o;
      Point p1oo,p2oo,p3oo,p4oo;
      Point p1ooo,p2ooo,p3ooo,p4ooo;

      float m1, c1, m2, c2; //1 = inital, 2 = goal
      
      int j = 0;
      int j1Min = std::numeric_limits<int>::max(), j2Min = j1Min, j3Min = j1Min, j4Min = j1Min;
      int jj1Min = std::numeric_limits<int>::max(), jj2Min = jj1Min, jj3Min = jj1Min, jj4Min = jj1Min;
      int jjj1Min = std::numeric_limits<int>::max(), jjj2Min = jjj1Min, jjj3Min = jjj1Min, jjj4Min = jjj1Min;

      float res = newMap.getResolution();
      float lenX = (newMap.getLength().x() - 1)/2;
      float lenY = (newMap.getLength().y() - 1)/2;

      init.x = poseArrowInit.pose.position.x;
      init.y = poseArrowInit.pose.position.y;
      init.theta = tf::getYaw(poseArrowInit.pose.orientation);
      
      goal.x = poseArrowGoal.pose.position.x;
      goal.y = poseArrowGoal.pose.position.y;
      goal.theta = tf::getYaw(poseArrowGoal.pose.orientation);

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
          uintl initL = sqrt(pow(inter.x - init.x, 2) +  pow(inter.y - init.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from inital pose to Intersection: " << initL*res << " m");
          uintl goalL = sqrt(pow(inter.x - goal.x, 2) +  pow(inter.y - goal.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from goal pose to Intersection: " << goalL*res << " m");


          for(int i=0; i <= initL; i++) 
          {
            do{
            p.x = init.x + res*j*cos(init.theta + (M_PI/2)) + res*i*cos(init.theta);  //centerX + right + front
            p.y = init.y + res*j*sin(init.theta + (M_PI/2)) + res*i*sin(init.theta);  //centerY + right + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell

            j1Min = std::min(j1Min,j);
            j = 0;
          }
          
          for(int i=0; i <= initL; i++) 
          {
            do{
            p.x = init.x - res*j*cos(init.theta + (M_PI/2)) + res*i*cos(init.theta);  //centerX + left + front
            p.y = init.y - res*j*sin(init.theta + (M_PI/2)) + res*i*sin(init.theta);  //centerY + left + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell

            j3Min = std::min(j3Min,j);
            j = 0;
          }

          
          p1o.x = init.x + res*j1Min*cos(init.theta + (M_PI/2)) + res*initL*cos(init.theta);  //centerX + right + front
          p1o.y = init.y + res*j1Min*sin(init.theta + (M_PI/2)) + res*initL*sin(init.theta);  //centerY + right + front

          p3o.x = init.x - res*j3Min*cos(init.theta + (M_PI/2)) + res*initL*cos(init.theta);  //centerX + left + front
          p3o.y = init.y - res*j3Min*sin(init.theta + (M_PI/2)) + res*initL*sin(init.theta);  //centerY + left + front

          p2o.x = init.x + res*j1Min*cos(init.theta + (M_PI/2)) ;  //centerX + right 
          p2o.y = init.y + res*j1Min*sin(init.theta + (M_PI/2)) ;  //centerY + right 

          p4o.x = init.x - res*j3Min*cos(init.theta + (M_PI/2)) ;  //centerX + left 
          p4o.y = init.y - res*j3Min*sin(init.theta + (M_PI/2)) ;  //centerY + left 

          line_strip1.points.push_back(p1o);
          line_strip1.points.push_back(p2o);
          line_strip1.points.push_back(p4o);
          line_strip1.points.push_back(p3o);
          line_strip1.points.push_back(p1o);
          arrowMarkers->markers.push_back(line_strip1);



          for(int i=0; i <= goalL; i++) 
          {
            
            do{
            p.x = goal.x + res*j*cos(goal.theta + (M_PI/2)) - res*i*cos(goal.theta);  //centerX + right + front
            p.y = goal.y + res*j*sin(goal.theta + (M_PI/2)) - res*i*sin(goal.theta);  //centerY + right + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell
            
            jj2Min = std::min(jj2Min,j);
            j = 0;
          }
          
          for(int i=0; i <= goalL; i++) 
          {
            do{
            p.x = goal.x - res*j*cos(goal.theta + (M_PI/2)) - res*i*cos(goal.theta);  //centerX + left + back
            p.y = goal.y - res*j*sin(goal.theta + (M_PI/2)) - res*i*sin(goal.theta);  //centerY + left + back
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell

            jj4Min = std::min(jj4Min,j);
            j = 0;
          }

          

          p2oo.x = goal.x + res*jj2Min*cos(goal.theta + (M_PI/2)) - res*goalL*cos(goal.theta);  //centerX + right + back
          p2oo.y = goal.y + res*jj2Min*sin(goal.theta + (M_PI/2)) - res*goalL*sin(goal.theta);  //centerY + right + back

          p4oo.x = goal.x - res*jj4Min*cos(goal.theta + (M_PI/2)) - res*goalL*cos(goal.theta);  //centerX + left + back
          p4oo.y = goal.y - res*jj4Min*sin(goal.theta + (M_PI/2)) - res*goalL*sin(goal.theta);  //centerY + left + back

          p1oo.x = goal.x + res*jj2Min*cos(goal.theta + (M_PI/2)) ;  //centerX + right 
          p1oo.y = goal.y + res*jj2Min*sin(goal.theta + (M_PI/2)) ;  //centerY + right 

          p3oo.x = goal.x - res*jj4Min*cos(goal.theta + (M_PI/2)) ;  //centerX + left 
          p3oo.y = goal.y - res*jj4Min*sin(goal.theta + (M_PI/2)) ;  //centerY + left 



          line_strip2.points.push_back(p1oo);
          line_strip2.points.push_back(p2oo);
          line_strip2.points.push_back(p4oo);
          line_strip2.points.push_back(p3oo);
          line_strip2.points.push_back(p1oo);
          arrowMarkers->markers.push_back(line_strip2);


          inter.theta = ((goal.theta - init.theta)/2) + init.theta;

          uintl front3 = std::min(j3Min,jj2Min);
          uintl back3 = std::min(j1Min,jj4Min);

          

          for(int i =0; i <= front3; i++) 
          {
            do{
            p.x = inter.x + res*j*cos(inter.theta + (M_PI/2)) + res*i*cos(inter.theta);  //centerX + right + front
            p.y = inter.y + res*j*sin(inter.theta + (M_PI/2)) + res*i*sin(inter.theta);  //centerY + right + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell
            
            jjj1Min = std::min(jjj1Min,j);
            j = 0;
          }
          
          for(int i =0; i <= front3; i++) 
          {
            do{
            p.x = inter.x - res*j*cos(inter.theta + (M_PI/2)) + res*i*cos(inter.theta);  //centerX + left + front
            p.y = inter.y - res*j*sin(inter.theta + (M_PI/2)) + res*i*sin(inter.theta);  //centerY + left + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell
            
            jjj3Min = std::min(jjj3Min,j);
            j = 0;
          }

          for(int i =0; i <= back3; i++) 
          {
            
            do{
            p.x = inter.x + res*j*cos(inter.theta + (M_PI/2)) - res*i*cos(inter.theta);  //centerX + right + front
            p.y = inter.y + res*j*sin(inter.theta + (M_PI/2)) - res*i*sin(inter.theta);  //centerY + right + front
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell
            
            jjj2Min = std::min(j,jjj2Min);

            j = 0;
          }
          
          for(int i=0; i <= back3; i++) 
          {
            do{
            p.x = inter.x - res*j*cos(inter.theta + (M_PI/2)) - res*i*cos(inter.theta);  //centerX + left + back
            p.y = inter.y - res*j*sin(inter.theta + (M_PI/2)) - res*i*sin(inter.theta);  //centerY + left + back
            j++;
            if ((abs(p.x) > lenX) || (abs(p.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p.x, p.y})); //! means free cell
            
            jjj4Min = std::min(j,jjj4Min);
            j = 0;
          }


          

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

