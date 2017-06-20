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
///*---CGAL Includes---*/
////#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
////#include <CGAL/convex_hull_2.h>
////#include <CGAL/Polygon_2.h>
///*---Boost filesystem to get parent directory---*/
//#include "boost/filesystem.hpp"
//#include <boost/bind.hpp>
//#include "boost/filesystem/operations.hpp"
//#include "boost/filesystem/path.hpp"
//#include "boost/progress.hpp"

#include "calibration_gui/sick_ldmrs.h"
#include "calibration_gui/common_functions.h"
#include "calibration_gui/visualization_rviz_ldmrs.h"

typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;

//typedef sensor_msgs::PointCloud2::Ptr pclPtr;
typedef sensor_msgs::PointCloud2 PCL2;
typedef boost::shared_ptr< PCL2 > pcl2Ptr;

typedef pcl::PointCloud<pcl::PointXYZ> PCL;
typedef boost::shared_ptr< PCL > pclPtr;

using namespace ros;
using namespace std;
using namespace velodyne_rawdata;

int sum(vector<int> array){
  int sum = 0;
  for(int i = 0; i<array.size();i++){
    sum+=array[i];
  }
  return sum;
}

namespace lidar_data_analise
{

class laserDataAnalise
{

public:
  laserDataAnalise(string topicName) {

    this->topicName = topicName;
    /*----Susbcribe LaserData Topic----*/
    sub = n.subscribe(topicName,1000, &laserDataAnalise::laserDataTreatment,this);
    ROS_INFO("Topic %s subscribed!",topicName.c_str());

    clustersPub = np.advertise<visualization_msgs::MarkerArray>("simple_clustering",1000);
    pclPub = np.advertise<PCL2>(topicName+"_PCL",1000);
    polygonPub = np.advertise<polygonS>(topicName+"_polygon",1000);
  }

   void convertToXYZ(sensor_msgs::LaserScan scan, vector<PointPtr>& points, double rot)
    {
      int s=scan.ranges.size();

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

   void scanToPcl(sensor_msgs::LaserScan scan, pcl2Ptr pclOut)
   {
     laser_geometry::LaserProjection projector;
     projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, *pclOut, tf_Listener);
   }


   static polygonSPtr getScanPolygon(pcl2Ptr scan)
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
//     convertToXYZ(scan, laserPoints, 0);
     pcl2Ptr scanPcl(new(PCL2));
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
  bool newData;
  pointClouldFusion(vector<string> topicName, vector<string> frame_Ids) {
    nPcl = topicName.size();
    ROS_INFO("%d Topics to subscribe!",nPcl);
    frame_IDs = frame_Ids;
    newData = false;
//    received.assign(6,0);
//    pcl2Ptr newPcl(new(PCL));
//    allPcl.assign(nPcl,newPcl);
    for(int i = 0; i<topicName.size(); i++){
      Subscriber sub;
      sub_pc.push_back(sub);
      sub_pc[i] = n.subscribe(topicName[i],10, &pointClouldFusion::getPointCloud,this);
      ROS_INFO("Topic %s subscribed!",topicName[i].c_str());
      pcl2Ptr newPcl(new(PCL2));;
      allPcl.push_back(newPcl);
      received.push_back(0);
    }
  }

void getPointCloud(const PCL2::ConstPtr myPcl){
  string frame_id = myPcl->header.frame_id;
  if(newData == false){
    for(int i = 0; i<nPcl;i++){
      if(frame_IDs[i] == frame_id && received[i] != 1){
        allPcl[i] = boost::const_pointer_cast< PCL2 >(myPcl);
        received[i] = 1;
        ROS_INFO("%s",frame_id.c_str());
      }
    }
  }
  if(sum(received) == nPcl){
    fill(received.begin(), received.end(), 0);
    newData = true;
    ROS_INFO("New Set of data Ready");
  }
}

vector< pcl2Ptr > getAllPcl(void){
  return allPcl;
}


private:
  NodeHandle n;
  tf::TransformListener tf_listener;
  vector<Subscriber> sub_pc;

  vector<pcl2Ptr> allPcl;
  int nPcl;
  vector<int> received;
  vector<string> frame_IDs;
};

}


void transformPCL(tf::TransformListener listener, string oriFrame, string destFrame, pclPtr pclIn){

  tf::StampedTransform transform;

  try{
    listener.waitForTransform(oriFrame,destFrame,ros::Time(0),ros::Duration(0.1));
    listener.lookupTransform(oriFrame,destFrame,ros::Time(0),transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians arround Z axis
  transform_2.rotate (Eigen::AngleAxisf (M_PI/4, Eigen::Vector3f::UnitZ()));

  // Executing the transformation
  PCL::Ptr transformed_cloud (new PCL ());
  PCL::Ptr cloud (new PCL ());
  pcl::transformPointCloud (*pclIn, *transformed_cloud, transform_2);

}


using namespace lidar_data_analise;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_space_detection");
  Publisher polygonPub;
  NodeHandle np;
  tf::TransformListener listener;

  laserDataAnalise lms151EAnalise("/lms151_D_scan");
  laserDataAnalise lms151DAnalise("/lms151_E_scan");
  laserDataAnalise ld_mrsAnalise1("/ld_rms/scan0");
  laserDataAnalise ld_mrsAnalise2("/ld_rms/scan1");
  laserDataAnalise ld_mrsAnalise3("/ld_rms/scan2");
  laserDataAnalise ld_mrsAnalise4("/ld_rms/scan3");

  polygonPub = np.advertise<polygonS>("merged_polygon",1000);

  vector<string> topicName;
  topicName.push_back("/lms151_D_scan_PCL");
  topicName.push_back("/lms151_E_scan_PCL");
  topicName.push_back("/ld_rms/scan0_PCL");
  topicName.push_back("/ld_rms/scan1_PCL");
  topicName.push_back("/ld_rms/scan2_PCL");
  topicName.push_back("/ld_rms/scan3_PCL");
  vector<string> frame_Ids;
  frame_Ids.push_back("lms151_E");
  frame_Ids.push_back("lms151_D");
  frame_Ids.push_back("/ldmrs0");
  frame_Ids.push_back("/ldmrs1");
  frame_Ids.push_back("/ldmrs2");
  frame_Ids.push_back("/ldmrs3");

  pointClouldFusion getPcls(topicName,frame_Ids);

  Rate  loopRate(50);
  while(ros::ok()){
    if(getPcls.newData){
      vector< pcl2Ptr > allPcl2 = getPcls.getAllPcl();

      vector< PCL > allPcl;
      for(int i = 0; i<allPcl2.size();i++){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*allPcl2[i], cloud);
        pclPtr cloud_Ptr(&cloud);
//        transformPCL(listener, frame_Ids[i], "/map", cloud_Ptr);

        allPcl.push_back(cloud);
      }



      PCL mergedPcl = allPcl[0]+allPcl[1];
//      pclPtr mergedPcl_Ptr  = boost::make_shared< PCL >(mergedPcl);
//      polygonSPtr polygon = laserDataAnalise::getScanPolygon(mergedPcl_Ptr);

//      /*---Publisher for the Polygon---*/
//      polygonPub.publish(*polygon);

      getPcls.newData = false;
    }
    ros::spinOnce();
    loopRate.sleep();
  }

  //ros::spin();

  return 0;
}
