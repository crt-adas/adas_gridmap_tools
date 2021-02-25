#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_utils.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_broadcaster.h>
#include <sstream>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

  ::ros::Rate loop_rate(10);
  while (::ros::ok())
  {

    auto initPos = initPoseIn();
    auto goalPose = goalPoseIn();
    PoseStamped poseArrow;
    bool ifInit;
    bool newPose = false;

    if(initPos.pose.pose.position.x != oldInitx)
    {                   
      ROS_INFO_STREAM("Initial Pose Recieved!");
      oldInitx = initPos.pose.pose.position.x;
      poseArrow.pose = initPos.pose.pose;
      ifInit = true;
      sendArrow(poseArrow, ifInit);
      newPose = true;
      MarkersOut(arrowMarkers);
    }

    if(goalPose.pose.position.x != oldGoalx)
    {                   
      ROS_INFO_STREAM("Goal Pose Recieved!");
      oldGoalx = goalPose.pose.position.x;
      poseArrow = goalPose;
      ifInit = false;
      sendArrow(poseArrow, ifInit);
      newPose = true;
      MarkersOut(arrowMarkers);

    }

    if(newPose)
    {

      visualization_msgs::Marker line_strip;
      geometry_msgs::Point p1,p2,p3,p4;
      getRecInfo(line_strip);
      double res = newMap.getResolution();
      double lenX = (newMap.getLength().x() - 1)/2;
      double lenY = (newMap.getLength().y() - 1)/2;
      double th = tf::getYaw(poseArrow.pose.orientation);

      int i1 = 0;
      int j12 = 1;
      int i2 = 0;
      
      int i3 = 0;
      int j34 = 1;
      int i4 = 0;
      
      
      int lengthR = 0;
      int lengthL = 0;
      bool rectangleOk = true;
      double rectangleArea;
      double rectangleAreaLast;
      double BiggestrectangleArea;
      geometry_msgs::Point p1o,p2o,p3o,p4o;
      bool expandRight = false;
      double count;
      do
      {

        
        do{
          p1.x = poseArrow.pose.position.x + res*j12*cos(th + (M_PI/2)) + res*i1*cos(th);  //centerX + right + front
          p1.y = poseArrow.pose.position.y + res*j12*sin(th + (M_PI/2)) + res*i1*sin(th);  //centerY + right + front
          i1++;
          if ((abs(p1.x) > lenX) || (abs(p1.y) > lenY))
            break;
        }
        while(!newMap.atPosition("elevation", {p1.x, p1.y}));
      
        do{
          p2.x = poseArrow.pose.position.x + res*j12*cos(th + (M_PI/2)) - res*i2*cos(th);  //centerX + right + back
          p2.y = poseArrow.pose.position.y + res*j12*sin(th + (M_PI/2)) - res*i2*sin(th);  //centerY + right + back
          i2++;
          if ((abs(p2.x) > lenX) || (abs(p2.y) > lenY))
            break;
        }
        while(!newMap.atPosition("elevation", {p2.x, p2.y}));
        
        

        
        do{
          p3.x = poseArrow.pose.position.x - res*j34*cos(th + (M_PI/2)) + res*i3*cos(th);  //centerX + left + front
          p3.y = poseArrow.pose.position.y - res*j34*sin(th + (M_PI/2)) + res*i3*sin(th);  //centerY + left + front
          i3++;
          if ((abs(p3.x) > lenX) || (abs(p3.y) > lenY))
            break;
        }
        while(!newMap.atPosition("elevation", {p3.x, p3.y}));
        
        do{
          p4.x = poseArrow.pose.position.x - res*j34*cos(th + (M_PI/2)) - res*i4*cos(th);  //centerX + left + back
          p4.y = poseArrow.pose.position.y - res*j34*sin(th + (M_PI/2)) - res*i4*sin(th);  //centerY + left + back
          i4++;
          if ((abs(p4.x) > lenX) || (abs(p4.y) > lenY))
            break;
        }
        while(!newMap.atPosition("elevation", {p4.x, p4.y}));
        

        
          
      
        if(i1 > i3)  // This condition keeps the rectangle shape by picking the shortest length for both sides (front)
        {
          p1.x = poseArrow.pose.position.x + res*j12*cos(th + (M_PI/2)) + res*i3*cos(th);
          p1.y = poseArrow.pose.position.y + res*j12*sin(th + (M_PI/2)) + res*i3*sin(th);
          i1 = i3;
        }else
        {
          p3.x = poseArrow.pose.position.x - res*j34*cos(th + (M_PI/2)) + res*i1*cos(th);
          p3.y = poseArrow.pose.position.y - res*j34*sin(th + (M_PI/2)) + res*i1*sin(th);
          i3 = i1;
        }
        if(i2 > i4)  // This condition keeps the rectangle shape by picking the shortest length for both sides (back) 
        {
          p2.x = poseArrow.pose.position.x + res*j12*cos(th + (M_PI/2)) - res*i4*cos(th);
          p2.y = poseArrow.pose.position.y + res*j12*sin(th + (M_PI/2)) - res*i4*sin(th);
          i2 = i4;
        }else
        {
          p4.x = poseArrow.pose.position.x - res*j34*cos(th + (M_PI/2)) - res*i2*cos(th);
          p4.y = poseArrow.pose.position.y - res*j34*sin(th + (M_PI/2)) - res*i2*sin(th);
          i4 = i2;
        }








        rectangleArea = (j12+j34) * (i1+i2); //Current area of rectangle

        if (rectangleArea > BiggestrectangleArea)
        {

          BiggestrectangleArea = rectangleArea;

          p1o = p1;
          p2o = p2;
          p3o = p3;
          p4o = p4;

        }

        
        if (!expandRight)
        {
          j12++;
          expandRight = true;
        }
        else
        {
          j34++;
          expandRight = false;
        }  
      
        if (count > 100)
          rectangleOk = false;
        
         

        rectangleAreaLast = rectangleArea;

        int i1 = 0;
        int i2 = 0;
        int i3 = 0;
        int i4 = 0;

        

        newPose = false;
        
      }
      while(!rectangleOk);

      line_strip.points.push_back(p1o);
      line_strip.points.push_back(p2o);
      line_strip.points.push_back(p4o);
      line_strip.points.push_back(p3o);
      line_strip.points.push_back(p1o);
      
      arrowMarkers->markers.push_back(line_strip);
      MarkersOut(arrowMarkers);

      count++;

    }
    


    gridMapOut(gridMapMsg);
    ::ros::spinOnce();
    loop_rate.sleep();
    
  }


  return 0;
}

