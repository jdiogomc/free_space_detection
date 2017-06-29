/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
\file  free_space_detection.h
\brief Global include file
\author Diogo Correia
\date   June, 2017
*/

#ifndef _FREE_SPACE_H_
#define _FREE_SPACE_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
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
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
/*---PointCould Includes---*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include "velodyne_pointcloud/rawdata.h"
/*---LAR TK4 Includes---*/
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"
#include <colormap/colormap.h>


/*---DEFINES---*/
#define RED -125
#define GREEN 125
#define BLACK 100
#define WHITE 0
#define UNKWON 50
#define YELLOW -50

typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;

typedef sensor_msgs::PointCloud2 PCL2;
typedef boost::shared_ptr< PCL2 > pcl2Ptr;

typedef pcl::PointCloud<pcl::PointXYZ> PCL;
typedef boost::shared_ptr< PCL > pclPtr;

using namespace ros;
using namespace std;
using namespace velodyne_rawdata;

class LidarClusters
{
public:
    vector<ClusterPtr> Clusters;
};

typedef boost::shared_ptr<LidarClusters> LidarClustersPtr;

/**
 * \class Markers
 * Class to handle the visualization markers
 * \author Jorge Almeida in mtt
 *
 */

class Markers
{
    public:
        void update(visualization_msgs::Marker& marker)
        {
            for(uint i=0;i<markers.size();++i)
                if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
                {
                    markers.erase(markers.begin()+i);
                    break;
                }

            markers.push_back(marker);
        }

        void decrement(void)
        {
            for(uint i=0;i<markers.size();++i)
            {
                switch(markers[i].action)
                {
                    case visualization_msgs::Marker::ADD:
                        markers[i].action = visualization_msgs::Marker::DELETE;
                        break;
                    case visualization_msgs::Marker::DELETE:
                        markers[i].action = -1;
                        break;
                }
            }
        }

        void clean(void)
        {
            vector<visualization_msgs::Marker> new_markers;

            for(uint i=0;i<markers.size();++i)
                if(markers[i].action!=-1)
                    new_markers.push_back(markers[i]);

            markers=new_markers;
        }

        vector<visualization_msgs::Marker> getOutgoingMarkers(void)
        {
            vector<visualization_msgs::Marker> m_out(markers);
            return markers;
        }

    private:

        vector<visualization_msgs::Marker> markers;
};

/*---Prototipos---*/
int sum(vector<int> array);
double degToRad(double deg);


geometry_msgs::Point xyzTortp(geometry_msgs::Point point);
geometry_msgs::Point rtpToxyz(geometry_msgs::Point point);
bool sortByY(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs);
void sortPcl(pclPtr in_pcl, pclPtr pclOut);
void azimuteFilter(pclPtr in_pcl, pclPtr pclOut, bool nearest);
void removeGround(pclPtr in_pcl, double angle);
vector<double> linspace(double min, double max, int n);
tf::Transform getTf(double x, double y, double z, double r, double p, double yy);
double pointsDist(pcl::PointXYZ center1, pcl::PointXYZ center3);
bool hasIdx(vector<int> array, int idx);
void euDistSort(pclPtr in_pcl, pclPtr pclOut);
void euDistSort2(pclPtr in_pcl, pclPtr pclOut);


pcl::PointXYZ createPointAllognLine(pcl::PointXYZ ini_point, pcl::PointXYZ last_point, double dist);
#endif
