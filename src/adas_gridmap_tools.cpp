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
      
      Marker line_strip1, line_strip2;
      getRecInfo(line_strip1);
      getRecInfo(line_strip2);

      Pose2D init, goal;
      Point p1i,p2i,p3i,p4i;
      Point p1g,p2g,p3g,p4g;
      Point p1o,p2o,p3o,p4o;
      Point p1oo,p2oo,p3oo,p4oo;

      float m1, c1, m2, c2; //1 = inital, 2 = goal
      float intSectX, intSectY; 

      uintl i1 = 0, i2 = 0, i3 = 0, i4 = 0;
      uintl j1 = 0, j2 = 0, j3 = 0, j4 = 0;
      uintl j1Min = std::numeric_limits<int>::max(), j2Min = j1Min, j3Min = j1Min, j4Min = j1Min;

      uintl ii1 = 0, ii2 = 0, ii3 = 0, ii4 = 0;
      uintl jj1 = 0, jj2 = 0, jj3 = 0, jj4 = 0;
      uintl jj1Min = std::numeric_limits<int>::max(), jj2Min = jj1Min, jj3Min = jj1Min, jj4Min = jj1Min;

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
        intSectX = (c2 - c1) / (m1 - m2);
        intSectY = m1 * intSectX + c1;
        ROS_INFO_STREAM("Intersection at: " << intSectX << " ," << intSectY <<"");

        if(newMap.atPosition("elevation", {intSectX, intSectY}))
          ROS_INFO_STREAM("Intersection is not collision free!");
        else
        {
          uintl initL = sqrt(pow(intSectX - init.x, 2) +  pow(intSectY - init.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from inital pose to Intersection: " << initL*res << " m");
          uintl goalL = sqrt(pow(intSectX - goal.x, 2) +  pow(intSectY - goal.y, 2))/res ; // Distance in res units
          ROS_INFO_STREAM("Distance from goal pose to Intersection: " << goalL*res << " m");


          for(i1=0; i1 <= initL; i1++) 
          {
            do{
            p1i.x = init.x + res*j1*cos(init.theta + (M_PI/2)) + res*i1*cos(init.theta);  //centerX + right + front
            p1i.y = init.y + res*j1*sin(init.theta + (M_PI/2)) + res*i1*sin(init.theta);  //centerY + right + front
            j1++;
            if ((abs(p1i.x) > lenX) || (abs(p1i.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p1i.x, p1i.y})); //! means free cell
            
            if(j1 < j1Min)
              j1Min = j1;

            j1 = 0;
          }
          
          for(i3=0; i3 <= initL; i3++) 
          {
            ROS_INFO_STREAM("i3: " << i3 << "");
            do{
            p3i.x = init.x - res*j3*cos(init.theta + (M_PI/2)) + res*i3*cos(init.theta);  //centerX + left + front
            p3i.y = init.y - res*j3*sin(init.theta + (M_PI/2)) + res*i3*sin(init.theta);  //centerY + left + front
            j3++;
            if ((abs(p3i.x) > lenX) || (abs(p3i.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p3i.x, p3i.y})); //! means free cell
            
            if(j3 < j3Min)
              j3Min = j3;

            j3 = 0;
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



          for(ii2=0; ii2 <= goalL; ii2++) 
          {
            
            do{
            p2g.x = goal.x + res*jj2*cos(goal.theta + (M_PI/2)) - res*ii2*cos(goal.theta);  //centerX + right + front
            p2g.y = goal.y + res*jj2*sin(goal.theta + (M_PI/2)) - res*ii2*sin(goal.theta);  //centerY + right + front
            jj2++;
            if ((abs(p2g.x) > lenX) || (abs(p2g.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p2g.x, p2g.y})); //! means free cell
            
            if(jj2 < jj2Min)
              jj2Min = jj2;
             
            jj2 = 0;
          }
          
          for(ii4=0; ii4 <= goalL; ii4++) 
          {
            do{
            p4g.x = goal.x - res*jj4*cos(goal.theta + (M_PI/2)) - res*ii4*cos(goal.theta);  //centerX + left + back
            p4g.y = goal.y - res*jj4*sin(goal.theta + (M_PI/2)) - res*ii4*sin(goal.theta);  //centerY + left + back
            jj4++;
            if ((abs(p4g.x) > lenX) || (abs(p4g.y) > lenY))
              break;
            }
            while(!newMap.atPosition("elevation", {p4g.x, p4g.y})); //! means free cell
            
            if(jj4 < jj4Min)
              jj4Min = jj4;

            jj4 = 0;
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

