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

namespace lidar_data_analise
{

class laserDataAnalise
{

public:
  laserDataAnalise(string topicName, double rotations) {

    this->topicName = topicName;
    rotation = rotations;
    /*----Susbcribe LaserData Topic----*/
    sub = n.subscribe(topicName,1000, &laserDataAnalise::laserDataTreatment,this);
    ROS_INFO("Topic %s subscribed!",topicName.c_str());

    clustersPub = np.advertise<visualization_msgs::MarkerArray>("simple_clustering",1000);
    pclPub = np.advertise<PCL>(topicName+"_PCL",1000);
    polygonPub = np.advertise<polygonS>(topicName+"_polygon",1000);
  }

   void convertToXYZ(sensor_msgs::LaserScan scan, vector<PointPtr>& points)
    {
      int s=scan.ranges.size();
      double rot = rotation;

      for(int n=0; n<s; n++)
      {
        double angle, d, x, y, z;
        d=scan.ranges[n]*cos(rot*M_PI/180);
        z=scan.ranges[n]*sin(rot*M_PI/180);
        angle = scan.angle_min + n*scan.angle_increment;

        x=d*cos(angle);
        y=d*sin(angle);

        PointPtr point(new Point);
        point->x = x;
        point->y = y;
        point->z = z;
        point->label = n;
        point->iteration = n+1;
        point->theta = angle;
        point->range = d;
        point->cluster_id = 1;
        points.push_back(point);
      }
  }


   vector<visualization_msgs::Marker> createClutersVisualizationMarker(vector<ClusterPtr>& clusters)
   {
     static Markers marker_list;

     //Reduce the elements status, ADD to REMOVE and REMOVE to delete
     marker_list.decrement();

     class_colormap colormap("hsv",10, 1, false);

     visualization_msgs::Marker marker_ids;
     visualization_msgs::Marker marker_clusters;

     marker_ids.header.frame_id = "lms152_1";//topicName;
     marker_ids.header.stamp = ros::Time::now();

     marker_clusters.header.frame_id = "lms152_1";//topicName;
     marker_clusters.header.stamp = marker_ids.header.stamp;

     marker_ids.ns = "ids";
     marker_ids.action = visualization_msgs::Marker::ADD;

     marker_clusters.ns = "clusters";
     marker_clusters.action = visualization_msgs::Marker::ADD;

     marker_ids.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

     marker_clusters.type = visualization_msgs::Marker::SPHERE_LIST;

     marker_ids.scale.x = 0.5;
     marker_ids.scale.y = 0.5;
     marker_ids.scale.z = 0.5;

     marker_clusters.scale.x = 0.2;
     marker_clusters.scale.y = 0.2;
     marker_clusters.scale.z = 0.2;

     marker_ids.color.a = 1.0;
      marker_ids.color.r = 0.0;
      marker_ids.color.g = 0.0;
      marker_ids.color.b = 0.0;

      for ( uint i = 0 ; i< clusters.size() ; i++)  //search all clusters
      {
        ClusterPtr cluster = clusters[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster->support_points.size();h++)
        {
          geometry_msgs::Point pt;
          pt.x=cluster->support_points[h]->x;
          pt.y=cluster->support_points[h]->y;
          pt.z=0;

          marker_clusters.points.push_back(pt);
          marker_clusters.colors.push_back(color);
        }

        marker_ids.pose.position.x = cluster->centroid->x;
        marker_ids.pose.position.y = cluster->centroid->y;
        marker_ids.pose.position.z = 0.3;

        //texto
        boost::format fm("%d");
        fm	% cluster->id;

        marker_ids.text = fm.str();
        marker_ids.id = cluster->id;
        marker_list.update(marker_ids);

        marker_list.update(marker_clusters);

      } //end for

      //Remove markers that should not be transmitted
      marker_list.clean();

      //Clean the marker_vector and put new markers in it;
      return marker_list.getOutgoingMarkers();
   }

   vector<ClusterPtr> removeSmallClusters(vector<ClusterPtr> clusters, int minPoints)
   {
     vector<ClusterPtr> cleanClusters;
     int count = 0;
     for(uint i = 0; i<clusters.size(); i++){
       if(clusters[i]->support_points.size()>minPoints){
         ClusterPtr  cluster = clusters[i];
         cluster->id = count;
         cleanClusters.push_back(cluster);
         count++;
       }
     }
     return cleanClusters;
   }

   void scanToPcl(sensor_msgs::LaserScan scan, pclPtr pclOut)
   {
     laser_geometry::LaserProjection projector;
     projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, *pclOut, tf_Listener);
   }


   static polygonSPtr getScanPolygon(pclPtr scan)
   {
     polygonSPtr polygon(new(polygonS));
     pcl::PointCloud<pcl::PointXYZ> cloud;

     polygon->header = scan->header;

     geometry_msgs::Point32 point;
     point.x = 0;
     point.y = 0;
     point.z = 0;
     polygon->polygon.points.push_back(point);

     pcl::fromROSMsg(*scan, cloud);
     for(int i = 0; i<cloud.size(); i++){
       geometry_msgs::Point32 point;
       pcl::PointXYZ pointPcl;
       pointPcl = cloud.points.at(i);
       point.x = pointPcl.x;
       point.y = pointPcl.y;
       point.z = pointPcl.z;
       polygon->polygon.points.push_back(point);
     } 

     return polygon;
   }

   void laserDataTreatment(sensor_msgs::LaserScan scan)
   {
     scan.header.stamp = ros::Time::now();
//     vector<PointPtr> laserPoints;
//     convertToXYZ(scan, laserPoints);
     pclPtr scanPcl(new(PCL));
     scanToPcl(scan, scanPcl);

     polygonSPtr polygon = getScanPolygon(scanPcl);

//     vector<PointPtr> points_sorted = laserPoints;
//     sort(points_sorted.begin(),points_sorted.end(),comparePoints);

//     vector<PointPtr> points_filtered;
//     filterPoints(laserPoints,points_filtered,0.01,50.);
//     vector<PointPtr> points_filtered_sorted = points_filtered;
//     sort(points_filtered_sorted.begin(),points_filtered_sorted.end(),comparePoints);

//     vector<ClusterPtr> clusters_1;
//     double threshold_nn = 0.2;
//     nnClustering( points_filtered, threshold_nn , clusters_1);

//     vector<PointPtr> laserPoints_clean = laserPoints;
//     removeInvalidPoints(laserPoints_clean, clusters_1, 3);

//     vector<ClusterPtr> clusters_clean;
//     clusters_clean = removeSmallClusters(clusters_1,2);

     /*---Publisher for the Clusters---*/
//     visualization_msgs::MarkerArray clusters_markers;
//     clusters_markers.markers = createClutersVisualizationMarker(clusters_1);
//     clustersPub.publish(clusters_markers);

     /*---Publisher for the PointCould---*/
     pclPub.publish(*scanPcl);

     /*---Publisher for the Polygon---*/
     polygonPub.publish(*polygon);
   }

private:

  NodeHandle n;
  Subscriber sub;
  NodeHandle np;
  Publisher clustersPub;
  Publisher pclPub;
  Publisher polygonPub;
  double rotation;
  string topicName;

  tf::TransformListener tf_Listener;
  tf::TransformBroadcaster tf_Broadcaster;
};

class pointClouldFusion
{
public:
  pointClouldFusion(vector<string> topicName) {
    for(int i = 0; i<topicName.size(); i++){
      Subscriber sub;
      sub_pc.push_back(sub);
      sub_pc[i] = n.subscribe(topicName[i],1000, &pointClouldFusion::getPointCloud,this);
      ROS_INFO("Topic %s subscribed!",topicName[i].c_str());
      pclPtr newPcl;
      allPcl.push_back(newPcl);
    }
  }

void getPointCloud(PCL myPcl){
  static int ind = 0;
  allPcl[ind] = pclPtr(&myPcl);
  ind ++;
  if(ind>3) ind = 0;
}

pclPtr getAllPcl(void){
  pclPtr PclPtr = allPcl[0];
  PCL cPcl = *PclPtr;

  for(int i = 1; i< allPcl.size(); i++){
    PCL newPclOut;
    PCL oldPcl = *allPcl[i];
    pcl::concatenatePointCloud(oldPcl ,cPcl,newPclOut);
    cPcl = newPclOut;
  }

  return pclPtr(&cPcl);
}


private:
  NodeHandle n;
  tf::TransformListener tf_listener;
  vector<Subscriber> sub_pc;

  vector<pclPtr> allPcl;
};

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

}

using namespace lidar_data_analise;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_space_detection");
  Publisher polygonPub;
  NodeHandle np;

  laserDataAnalise lms151EAnalise("/lms151_D_scan",0);
  laserDataAnalise lms151DAnalise("/lms151_E_scan",0);
  laserDataAnalise ld_mrsAnalise1("/ld_rms/scan0",0);
  laserDataAnalise ld_mrsAnalise2("/ld_rms/scan1",0);
  laserDataAnalise ld_mrsAnalise3("/ld_rms/scan2",0);
  laserDataAnalise ld_mrsAnalise4("/ld_rms/scan3",0);

  Rate  loopRate(50);

  polygonPub = np.advertise<polygonS>("merged_polygon",1000);
  vector<string> topicName;
  topicName.push_back("/lms151_D_scan_PCL");
  topicName.push_back("/lms151_E_scan_PCL");
  topicName.push_back("/ld_rms/scan0_PCL");
  topicName.push_back("/ld_rms/scan1_PCL");
  topicName.push_back("/ld_rms/scan2_PCL");
  topicName.push_back("/ld_rms/scan3_PCL");

  //pointClouldFusion mergedPclHandler(topicName);

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
