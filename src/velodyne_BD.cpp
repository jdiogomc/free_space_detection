#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <laser_geometry/laser_geometry.h>
/*---PointCould Includes---*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/point_cloud.h>
#include "velodyne_pointcloud/rawdata.h"
/*---LAR TK4 Includes---*/
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"
//#include "lidar_segmentation/visualization_rviz.h"
#include <colormap/colormap.h>
/*---CGAL Includes---*/
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/convex_hull_2.h>
//#include <CGAL/Polygon_2.h>
/*---Boost filesystem to get parent directory---*/
#include "boost/filesystem.hpp"
#include <boost/bind.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

#include "calibration_gui/sick_ldmrs.h"
#include "calibration_gui/common_functions.h"
#include "calibration_gui/visualization_rviz_ldmrs.h"

typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;

typedef sensor_msgs::PointCloud2::Ptr pclPtr;
typedef sensor_msgs::PointCloud2 PCL;

using namespace ros;
using namespace std;
using namespace velodyne_rawdata;


class velodyneDataHandle
{
public:
  velodyneDataHandle() {

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_points", 10,
                     &velodyneDataHandle::processScan, this,
                     ros::TransportHints().tcpNoDelay(true));

    velodyne_pub = node.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
    sphereCentroid_pub = node.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);
  }

//  void pcl2ToLaserPoints(sensor_msgs::PointCloud2::ConstPtr &pcl2, vector< vector<PointPtr> > &laserscan){
//    vector<PointPtr> scanPtr;
//    laserscan.assign(16, scanPtr);

//    pcl::PointCloud<VPoint> pcl;
//    pcl::fromROSMsg(*pcl2, pcl);

//    for(int i = 0; i<pcl.size();i++){
//      VPoint point = pcl.at(i);
//      int ringNumber = point.ring;
//      PointPtr pointXYZ;
//      pointXYZ->x = point.x;
//      pointXYZ->y = point.y;
//      pointXYZ->z = point.z;

//      laserscan[ringNumber].push_back(pointXYZ);
//    }

//  }

  void getClusters(vector<PointPtr> laserPoints, vector<ClusterPtr> clusters_nn){
    vector<PointPtr> points_filtered;
    filterPoints(laserPoints,points_filtered,0.01,200.);

    double threshold_nn = 0.20;
    nnClustering( points_filtered, threshold_nn , clusters_nn);

  }

  void processScan(const sensor_msgs::PointCloud2::ConstPtr &scanMsg){
    vector< vector<PointPtr> > laserscans;
    //pcl2ToLaserPoints(scanMsg, laserscans);


    vector<double> radius;
    vector<LidarClustersPtr> circlePoints;
    Point sphere;
    vector<geometry_msgs::Point> center;
    vector<LidarClustersPtr> clusters;

    for(int i = 0; i<laserscans.size();i++){
      vector<ClusterPtr> clusters_nn;
      getClusters(laserscans[i], clusters_nn);

      LidarClustersPtr cluster (new LidarClusters);
      cluster->Clusters = clusters_nn;
      clusters.push_back(cluster);

      LidarClustersPtr circlePs (new LidarClusters);
      vector<ClusterPtr> circleP;
      double r;
      r=find_circle(clusters_nn,circleP,i);
      int num;
      if(r!=0)
        num++;

      radius.push_back(r);
      circlePs->Clusters = circleP;
      circlePoints.push_back(circlePs);

      //publish circle centroid

      center.push_back(sphereCentroid.point);

      int circlesNumb = 0;
      if(i==laserscans.size()-1)
      {
        for(int j=0; j<laserscans.size(); j++)
        {
          if(radius[j]>0.001)
            circlesNumb++;
        }
      }

      if(i==laserscans.size()-1)
      {
        if(circlesNumb>1)
        {
          calculateSphereCentroid(center, sphereCentroid, radius);
          sphere.x=sphereCentroid.point.x;
          sphere.y=sphereCentroid.point.y;
          sphere.z=sphereCentroid.point.z;
        }
        else
        {
          sphere.x=-100;
          sphere.y=0;
          sphere.z=0;
        }
        sphereCentroid.header.stamp = ros::Time::now();
        sphereCentroid_pub.publish(sphereCentroid);
      }
    }
    vector<ClusterPtr> linePoints;
    //      Vizualize the Segmentation results

    visualization_msgs::MarkerArray targets_markers;
    targets_markers.markers = createTargetMarkers(clusters,circlePoints, sphere,radius);

    velodyne_pub.publish(targets_markers);

  }


private:
  Subscriber velodyne_scan_;
  Publisher sphereCentroid_pub;
  Publisher velodyne_pub;
  NodeHandle node;
  geometry_msgs::PointStamped sphereCentroid;

};


using namespace lidar_data_analise;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_space_detection");
  Publisher centroidPub;
  NodeHandle np;


  while(ros::ok()){
//    pclPtr mergedPcl = mergedPclHandler.getAllPcl();
//    polygonSPtr polygon = laserDataAnalise::getScanPolygon(mergedPcl);

//    /*---Publisher for the Polygon---*/
//    polygonPub.publish(*polygon);

    ros::spinOnce();
    loopRate.sleep();
  }

  //ros::spin();

  return 0;
}
