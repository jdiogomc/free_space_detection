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
/*---LAR TK4 Includes---*/
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"
#include "lidar_segmentation/visualization_rviz.h"
#include <colormap/colormap.h>
/*---Boost filesystem to get parent directory---*/
#include "boost/filesystem.hpp"
#include <boost/bind.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;

typedef sensor_msgs::PointCloud2::Ptr pclPtr;
typedef sensor_msgs::PointCloud2 PCL;

using namespace ros;
using namespace std;

int getFilesInDir(const string filesPath, vector<string> &outfiles, vector<string> &filesName){

  boost::filesystem::path full_path( boost::filesystem::initial_path<boost::filesystem::path>() );

  full_path = boost::filesystem::system_complete( boost::filesystem::path( filesPath ) );

  unsigned long file_count = 0;
  unsigned long dir_count = 0;
  unsigned long other_count = 0;
  unsigned long err_count = 0;

  if ( !boost::filesystem::exists( full_path ) )
  {
    std::cout << "\nNot found: " << filesPath << std::endl;
    return 1;
  }

  if ( boost::filesystem::is_directory( full_path ) )
  {
    boost::filesystem::directory_iterator end_iter;
    for ( boost::filesystem::directory_iterator dir_itr( full_path );
          dir_itr != end_iter;
          ++dir_itr )
    {
      try
      {
        if ( boost::filesystem::is_directory( dir_itr->status() ) )
        {
          ++dir_count;
        }
        else if ( boost::filesystem::is_regular_file( dir_itr->status() ) )
        {
          ++file_count;
          string file_name = dir_itr->path().filename().string();
          int name_len = file_name.size();
          string file_type = file_name.substr(name_len-4,10);
          if(strcmp( file_type.c_str(),".txt") == 0){
            outfiles.push_back(filesPath+"/"+file_name);
            filesName.push_back(file_name.substr(0,name_len-4));
          }
        }
        else
        {
          ++other_count;
        }

      }
      catch ( const std::exception & ex )
      {
        ++err_count;
        std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
      }
    }
  }
  else // must be a file
  {
    std::cout << "\nFound: " << full_path.filename() << "\n";
  }

  return 0;
}


Eigen::Matrix4f getTransformFromFile(string filePath){

  int nrows = 4;
  int ncols = 4;
  Eigen::MatrixX4f transform(nrows,ncols);

  const char* FilePath = filePath.c_str();
  ifstream fin (FilePath);

  if (fin.is_open())
  {
      for (int row = 0; row < nrows; row++)
          for (int col = 0; col < ncols; col++)
          {
              float item = 0.0;
              fin >> item;
              transform(row, col) = item;
          }
      fin.close();
  }
/*--- DEBUG ---
  cout << transform(0,0) << " " << transform(0,1) << " " << transform(0,2) << " " << transform(0,3) << endl;
  cout << transform(1,0) << " " << transform(1,1) << " " << transform(1,2) << " " << transform(1,3) << endl;
  cout << transform(2,0) << " " << transform(2,1) << " " << transform(2,2) << " " << transform(2,3) << endl;
  cout << transform(3,0) << " " << transform(3,1) << " " << transform(3,2) << " " << transform(3,3) << endl;
  --- DEBUG ---*/
  return transform;
}

tf::Transform getTfTransform(Eigen::MatrixX4f trans){

  tf::Transform t1;

  t1.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));

  tf::Quaternion q;
  tf::Matrix3x3 ori;
  ori.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
  ori.getRotation(q);

  t1.setRotation(q);

  return t1;
}

double degToRad(double deg){
  double rad = deg*M_PI/180;
  return rad;
}

tf::Transform getTf(double x, double y, double z, double r, double p, double yy){

  tf::Transform t1;
  t1.setOrigin(tf::Vector3(x,y,z));
  tf::Quaternion q;
  q.setRPY(degToRad(r),degToRad(p),degToRad(yy));
  t1.setRotation(q);

  return t1;
}


void readCalibrationFiles(string filesPath, vector<tf::Transform> &deviceFrames, vector<string> &deviceNames){

  vector<string> files;
  getFilesInDir(filesPath, files, deviceNames);

  tf::Transform ld_tf;
  bool ld_push = false;
  tf::Transform transform;

  for(int i = 0; i<files.size(); i++){


    transform = getTfTransform( getTransformFromFile(files[i]) );

    if(deviceNames[i] == "ldmrs"){
      deviceNames[i] = "ldmrs0";
      deviceFrames.push_back(transform*getTf(0, 0, 0, 0, -1.6, 0));
      ld_tf = transform;
      ld_push = true;

    }else{
      deviceFrames.push_back(transform);
    }

  }

  if(ld_push){
    deviceNames.push_back("ldmrs1");
    deviceNames.push_back("ldmrs2");
    deviceNames.push_back("ldmrs3");

    deviceFrames.push_back(ld_tf*getTf(0, 0, 0, 0, -0.8, 0));
    deviceFrames.push_back(ld_tf*getTf(0, 0, 0, 0, 0.8, 0));
    deviceFrames.push_back(ld_tf*getTf(0, 0, 0, 0, 1.6, 0));
    //deviceFrames.push_back(ld_tf);
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "device_frame_publisher");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  vector<tf::Transform> deviceFrames;
  vector<string> deviceNames;
  readCalibrationFiles("/home/diogo/catkin_ws/src/free_space_detection/calibration_data",
                       deviceFrames, deviceNames);
  //deviceNames[0] = "/lms152_1";


//  deviceFrames.push_back(getTf(0, 0, 0, 0, -1.6, 0));
//  deviceFrames.push_back(getTf(0, 0, 0, 0, -0.8, 0));
//  deviceFrames.push_back(getTf(0, 0, 0, 0, 0.8, 0));
//  deviceFrames.push_back(getTf(0, 0, 0, 0, 1.6, 0));

//  deviceNames.push_back("ldmrs0");
//  deviceNames.push_back("ldmrs1");
//  deviceNames.push_back("ldmrs2");
//  deviceNames.push_back("ldmrs3");
  tf::Transform LD_tf = deviceFrames[deviceFrames.size()-1]*getTf(0, 0, 0, 0, -1.6, 0);

//  deviceFrames.push_back(getTf(0, 0, 0, 0, 0, 0));
//  deviceNames.push_back("lms151_E");

//  for(int i = 0; i < deviceNames.size();i++){
//    if(deviceNames[i]=="ldmrs0")
//  }
  string ref_sensor;
  if(argc > 1){
    ref_sensor = argv[0];
  }else{
    ref_sensor = "lms151_E";
  }


  ros::Rate loop_rate(50);
  while(ros::ok()){
    br.sendTransform(tf::StampedTransform(LD_tf.inverse(), ros::Time::now(),"map",ref_sensor));
    for(int i = 0; i<deviceNames.size(); i++){
      tf::Transform T = deviceFrames[i];//*LD_tf.inverse();
      string name = deviceNames[i];
      br.sendTransform(tf::StampedTransform(T, ros::Time::now(),ref_sensor,name));
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  ROS_INFO("Hello world!");
}
