/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
   \file  free_space_detection.cpp
   \brief Algorithm for subscribing to lidar data and compute the free space
   \author Diogo Correia
   \date   June, 2017
 */


#include "free_space_detection/free_space_detection.h"

/**
@brief Sums all the points of an array
@param[in] Array to sum the values
@return int The sum of all the elemenets in the array
*/
int sum(vector<int> array){
  int sum = 0;
  for(int i = 0; i<array.size();i++){
    sum+=array[i];
  }
  return sum;
}

/**
@brief Converts an angle from degrees to radians
@param[in] Angle in degrees
@return double Angles in radians
*/
double degToRad(double deg){
  double rad = deg*M_PI/180;
  return rad;
}

namespace lidar_data_analise
{

/**
@brief Class to handle the incoming laser data from the LIDAR sensors
*/
class laserDataAnalise
{

public:
  laserDataAnalise(string topicName, string frame_id) {

    this->topicName = topicName;
    this->frameId = frame_id;
    /*----Susbcribe LaserData Topic----*/
    sub = n.subscribe(topicName,1000, &laserDataAnalise::laserDataTreatment,this);
    ROS_INFO("Topic %s subscribed!",topicName.c_str());

    clustersPub = np.advertise<visualization_msgs::MarkerArray>("simple_clustering",1000);
    pclPub = np.advertise<PCL2>(topicName+"_PCL",1000);
    polygonPub = np.advertise<polygonS>(topicName+"_polygon",1000);

    scanPcl = pcl2Ptr(new PCL2 );
  }

  /**
  @brief Converts the laser scan point to an array of points
  @param[in] Laser scan of the laser data
  @param[out] Array of points to save the converted points
  @param[in] Angle between the scan plane and the horizontal plane
  @return void
  */
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

   /**
   @brief Creats visualization markers to display the clusters resultant from the laser scan
   @param[in] Array of clusters
   @return vector<visualization_msgs::Marker> Array with the visualization markers
   */
   vector<visualization_msgs::Marker> createClutersVisualizationMarker(vector<ClusterPtr>& clusters)
   {
     static Markers marker_list;

     //Reduce the elements status, ADD to REMOVE and REMOVE to delete
     marker_list.decrement();

     class_colormap colormap("hsv",10, 1, false);

     visualization_msgs::Marker marker_ids;
     visualization_msgs::Marker marker_clusters;

     marker_ids.header.frame_id = frameId;//topicName;
     marker_ids.header.stamp = ros::Time::now();

     marker_clusters.header.frame_id = frameId;//topicName;
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

   /**
   @brief Removes clusters with less than a minimum of points
   @param[in] Array with all the clusters
   @param[in] Minimum points to keep the clusters
   @return vector<ClusterPtr> Array with the clusters after filtering
   */
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

   /**
   @brief Converts a LaserScan message to a PointCloud2 message
   @param[in] LaserScan message with the data
   @param[out] Point cloud pointer to hold the data
   @return void
   */
   void scanToPcl(sensor_msgs::LaserScan scan, pcl2Ptr pclOut)
   {
     laser_geometry::LaserProjection projector;
     projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, *pclOut, tf_Listener);
   }

   /**
   @brief Creates a polygon message from a point cloud
   @param[in] Pointer for the point cloud data
   @return polygonSPtr Pointer to the polygon message
   */
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

   /**
   @brief Creates a polygon message from a point cloud
   @param[in] Pointer for  the point cloud data
   @param[in] Id of the reference frame for the polygon message
   @return polygonSPtr Pointer to the polygon message
   */
   static polygonSPtr getScanPolygon(pclPtr cloud_ptr, string frame_id)
   {
     polygonSPtr polygon(new(polygonS));
     pcl::PointCloud<pcl::PointXYZ> cloud(*cloud_ptr);
     std_msgs::Header h;
     h.frame_id = frame_id;
     polygon->header = h;

//     geometry_msgs::Point32 point;
//     point.x = 0;
//     point.y = 0;
//     point.z = 0;
//     polygon->polygon.points.push_back(point);

//     pcl::fromROSMsg(*scan, cloud);
     for(int i = 0; i<cloud.size(); i++){
       geometry_msgs::Point32 point;
       pcl::PointXYZ pointPcl;
       pointPcl = cloud.points.at(i);
       point.x = pointPcl.x;
       point.y = pointPcl.y;
       point.z = 0;//pointPcl.z;
       polygon->polygon.points.push_back(point);
     }

     return polygon;
   }

   /**
   @brief Callback used to handle the incoming laser data
   @param[in] Incoming laser scan
   @return void
   */
   void laserDataTreatment(sensor_msgs::LaserScan scan)
   {
     //scan.header.stamp = ros::Time::now();
     scanToPcl(scan, scanPcl);

     polygonSPtr polygon = getScanPolygon(scanPcl);

     /*---Publisher for the PointCould---*/
     pclPub.publish(*scanPcl);

     /*---Publisher for the Polygon---*/
     polygonPub.publish(*polygon);
   }

   /**
   @brief Gets the transformation between tow frames and applies it to the points of a point cloud
   @param[out] Point cloud pointer to assing the points
   @return bool Returns true if there if data in the scan point cloud
   */
   bool getPcl(pcl2Ptr cloud_in_ptr){
     if(scanPcl->width>0){
       *cloud_in_ptr = *scanPcl;
       return true;
     }
     return false;
   }

   /**
   @brief Gets the transformation between tow frames and applies it to the points of a point cloud
   @param[in] Destination frame
   @param[in] Point cloud to apply the transformation
   @param[out] Point cloud with the transformation applied
   @return void
   */
   void transformPCL(string destFrame,  pclPtr pclIn, pclPtr pclOut)
   {

     tf::StampedTransform transform;
     string oriFrame = frameId;
     try{
       tf_Listener.waitForTransform(destFrame,oriFrame,ros::Time(0),ros::Duration(0.1));
       tf_Listener.lookupTransform(destFrame,oriFrame,ros::Time(0),transform);
     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
     }

     Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
     tf::transformTFToEigen (transform, transform_2);

     pcl::transformPointCloud (*pclIn, *pclOut, transform_2);
   }

   /**
   @brief Aplies a rigid body transformation to the points of a point cloud
   @param[in] Transformation to apply
   @param[in] Point cloud to apply the transformation
   @return geometry_msgs::Point Point in spheric coordinates
   */
   static void transformPCL(tf::Transform transform,  pclPtr pclIn)
   {

     Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
     tf::transformTFToEigen (transform, transform_2);

     pcl::transformPointCloud (*pclIn, *pclIn, transform_2);

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
  string frameId;

  tf::TransformListener tf_Listener;
  tf::TransformBroadcaster tf_Broadcaster;

  pcl2Ptr scanPcl;
};
}

/**
@brief Converts a point from cartezian coordinates to spheric coordinates
@param[in] Point in cartezian coordinates
@return geometry_msgs::Point Point in spheric coordinates
*/
geometry_msgs::Point xyzTortp(geometry_msgs::Point point){
  double X = point.x;
  double Y = point.y;
  double Z = point.z;

  double radius = sqrt((double)(double)pow(X,2) + (double)pow(Y,2) + (double)pow(Z,2));
  double theta = atan2(Y, X);
  double phi = acos((double)(Z / radius));

  if(theta<0)
    theta += M_PI*2;

  geometry_msgs::Point point_s;

  point_s.x = radius;
  point_s.y = theta;
  point_s.z = phi;

  return point_s;
}

/**
@brief Converts a point from spheric coordinates to cartezian coordinates
@param[in] Point in spheric coordinates
@return geometry_msgs::Point Point in cartezian coordinates
*/
geometry_msgs::Point rtpToxyz(geometry_msgs::Point point){
  double radius = point.x;
  double theta = point.y;
  double phi = point.z;

  double X = (double)((double)cos(theta) * (double)sin(phi) * (double)radius);
  double Y = (double)((double)sin(theta) * (double)sin(phi) * (double)radius);
  double Z = (double)((double)cos(phi) * (double)radius);

  geometry_msgs::Point point_s;

  point_s.x = X;
  point_s.y = Y;
  point_s.z = Z;

  return point_s;
}

/**
@brief Sorting function used to sort the point cloud by the point's second element (the azimute angle)
@param[in] First point to compare
@param[in] Second point to compare
@return bool
*/
bool sortByY(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs) { return lhs.y > rhs.y; }

/**
@brief Sorts a point cloud points usign a specific sorting function
@param[in] Point cloud with the points to sort
@param[out] Point cloud with the sorted points
@return void
*/
void sortPcl(pclPtr in_pcl, pclPtr pclOut)
{
  vector<double> points;
  vector<geometry_msgs::Point> points_g;

  for(int i = 0; i<in_pcl->points.size();i++){
    geometry_msgs::Point point;
    point.x = in_pcl->points[i].x;
    point.y = in_pcl->points[i].y;
    point.z = in_pcl->points[i].z;
    geometry_msgs::Point point_s = xyzTortp(point);
    points_g.push_back(point_s);
    points.push_back(point_s.y);
  }

  sort(points_g.begin(), points_g.end(), sortByY);

//  cout << "Points sorted:" << endl;
//  for(int i = 0; i<points_g.size();i++){
//    cout << points_g[i].x << ", " << points_g[i].y << ", " << points_g[i].z << endl;
//  }
//  cout << "END"<<endl;

  pclOut->width  = points.size();
  pclOut->height = 1;
  pclOut->points.resize (pclOut->width * pclOut->height);

  for(int i = 0; i<in_pcl->points.size();i++){
    geometry_msgs::Point point_c = rtpToxyz(points_g[i]);
    pclOut->points[i].x = point_c.x;
    pclOut->points[i].y = point_c.y;
    pclOut->points[i].z = point_c.z;
  }
}

/**
@brief Filters the merged point cloud by the point's azimute angle
@param[in] Point cloud with the points to filter
@param[out] Point cloud with the filtered points
@param[in] Indicates if is to keep the point nearst to the sensor or the farest
@return void
*/
void azimuteFilter(pclPtr in_pcl, pclPtr pclOut, bool nearest){

  double anglePerc = degToRad(0.6);

  vector<geometry_msgs::Point> points_s;

  for(int i = 0; i<in_pcl->points.size();i++){
    geometry_msgs::Point point;
    point.x = in_pcl->points[i].x;
    point.y = in_pcl->points[i].y;
    point.z = in_pcl->points[i].z;
    geometry_msgs::Point point_s = xyzTortp(point);
    points_s.push_back(point_s);
  }

  vector<geometry_msgs::Point> points_s_clean;
  points_s_clean.push_back(points_s[0]);
  double angle2 = points_s[1].y;
  for(int i = 1; i<points_s.size(); i++){
    double angle1 = points_s[i].y; double r1 = points_s[i].x; geometry_msgs::Point point1 = points_s[i];
    double r2 = points_s_clean.back().x; geometry_msgs::Point point2 = points_s_clean.back();
    if(abs(angle1-angle2)<=anglePerc){
      if(nearest){
        if(r1<r2){
          points_s_clean.back() = point1;
        }
      }else{
        if(r1>r2){
          points_s_clean.back() = point1;
        }
      }
    }else{
      points_s_clean.push_back(point1);
      angle2 = points_s_clean.back().y;
//      points_s_clean.push_back(point2);
    }

    pclOut->width  = points_s_clean.size();
    pclOut->height = 1;
    pclOut->points.resize (pclOut->width * pclOut->height);

    for(int i = 0; i<pclOut->points.size();i++){
      geometry_msgs::Point point_c = rtpToxyz(points_s_clean[i]);
      pclOut->points[i].x = point_c.x;
      pclOut->points[i].y = point_c.y;
      pclOut->points[i].z = point_c.z;
    }
  }

//  cout << "Before: " << in_pcl->points.size() << endl;
//  cout << "After: "<< pclOut->points.size() << endl;
}


/**
@brief Function used to remove the grounf detections from the Sick LD-MRS sensor
@param[in] Point cloud with the scan data
@param[in] Angle of the scan plane
@param[in] Vertical distance from the sensor to the ground
@return void
*/
void removeGround(pclPtr in_pcl, double angle, double distance)
{

  double dist = abs(distance/sin(degToRad(angle)));
for(int i = 0; i<in_pcl->points.size(); i++){
  geometry_msgs::Point point;
  point.x = in_pcl->points[i].x;
  point.y = in_pcl->points[i].y;
  point.z = in_pcl->points[i].z;
  geometry_msgs::Point point_s = xyzTortp(point);
  double range = point_s.x;
//  if(abs(range - dist)<=2)
//  cout << "Range: " << range << endl;
  if(range > dist-3)
    in_pcl->points.erase(in_pcl->points.begin() + i);
}

}

/**
@brief Creates an array with a specific number of points equaly spaced allong a minimum and a maximum
@param[in] Array lowest value
@param[in] Array upper value
@param[in] Number of points of the array
@return vector<double>
*/
vector<double> linspace(double min, double max, int n)
{
 vector<double> result;
 // vector iterator
 int iterator = 0;

for (int i = 0; i <= n-2; i++)
 {
 double temp = min + i*(max-min)/(floor((double)n) - 1);
 result.insert(result.begin() + iterator, temp);
 iterator += 1;
 }

//iterator += 1;

result.insert(result.begin() + iterator, max);
 return result;
}

/**
@brief Sorts the point cloud points by their distance to the closest neighbor
@param[in] Point cloud to sort
@param[out] Point cloud sorted
@return void
*/
tf::Transform getTf(double x, double y, double z, double r, double p, double yy){

  tf::Transform t1;
  t1.setOrigin(tf::Vector3(x,y,z));
  tf::Quaternion q;
  q.setRPY(degToRad(r),degToRad(p),degToRad(yy));
  t1.setRotation(q);

  return t1;
}

/**
@brief Calculates the euclidian distance between two points
@param[in] First point
@param[out] Second point
@return double Returns the euclidian distance beteewn the two points
*/
double pointsDist(pcl::PointXYZ center1, pcl::PointXYZ center3){

  double eu_dist, x_dist, y_dist, z_dist;
  x_dist = center1.x - center3.x;
  y_dist = center1.y - center3.y;
  z_dist = center1.z - center3.z;
  eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );

  return eu_dist;
}

/**
@brief Checks if an array contains a value
@param[in] Array to check for the value
@param[in] Value to check for
@return bool Returns true if the array has the value
*/
bool hasIdx(vector<int> array, int idx){
  for(int i = 0; i<array.size();i++){
    if(array[i]==idx){
      return true;
    }
  }
  return false;
}

/**
@brief Sorts the point cloud points by their distance to the closest neighbor
@param[in] Point cloud to sort
@param[out] Point cloud sorted
@return void
*/
void euDistSort(pclPtr in_pcl, pclPtr pclOut){

  pclOut->width = in_pcl->width;
  pclOut->height = in_pcl->height;
  pclOut->points.resize(pclOut->width * pclOut->height);

  int t_point = 0;
  vector<int> indices;
  pclOut->points[0] = in_pcl->points[0];
  indices.push_back(0);
  for(int i = 1; i<in_pcl->points.size(); i++){
    double min_dist = 10000;
    int idx;
    for(int j = 0; j<in_pcl->points.size(); j++){
      if(!hasIdx(indices, j)){
        double dist = pointsDist(in_pcl->points[t_point], in_pcl->points[j]);
        if(min_dist>dist){
          min_dist =  dist;
          idx = j;
        }
      }
    }
    t_point = idx;
    pclOut->points[i] = in_pcl->points[idx];
    indices.push_back(idx);

  }
}

/**
@brief Sorts the point cloud points by their distance to the closest neighbor
@param[in] Point cloud to sort
@param[out] Point cloud sorted
@return void
*/
void euDistSort2(pclPtr in_pcl, pclPtr pclOut){

  pclOut->width = in_pcl->width;
  pclOut->height = in_pcl->height;
  pclOut->points.resize(pclOut->width * pclOut->height);

  int t_point = 0;
  int p_point = -1;
  pclOut->points[0] = in_pcl->points[0];
  for(int i = 1; i<in_pcl->points.size(); i++){
    double min_dist = 10000;
    int idx;
    for(int j = 0; j<in_pcl->points.size(); j++){
      if(j != t_point && j != p_point){
        double dist = pointsDist(in_pcl->points[t_point], in_pcl->points[j]);
        if(min_dist>dist){
          min_dist =  dist;
          idx = j;
        }
      }
    }
    p_point = t_point;
    t_point = idx;
    pclOut->points[i] = in_pcl->points[idx];
  }
}


/**
@brief Class containing functions to create and publish an ocupation grid from point cloud data
*/
class ocupGrid
{
public:
  ocupGrid(string frameId, double xMin, double xMax, double yMin, double yMax, double cellResol) {
    this->cellResolution = cellResol;
    this->xMin = xMin; this->yMin = yMin; this->xMax = xMax; this->yMax = yMax;
    this->xCells = (int) ((xMax-xMin)/cellResolution);
    this->yCells = (int) ((yMax-yMin)/cellResolution);

    grid = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    initGrid(frameId);

    ocGrid.assign(xCells * yCells,UNKWON);

    gridPub = np.advertise<nav_msgs::OccupancyGrid>("ocupancy_grid",1000);
  }

  /**
  @brief Assigns assign ocupation grid size acording to a point cloud extreme points
  @param[in] Ocupation point cloud
  @return void
  */
  void getGridSize(pclPtr inPcl){
    for(int i = 0; i<inPcl->points.size();i++){
      if(xMin > inPcl->points[i].x){
        xMin = inPcl->points[i].x;
      }
      if(xMax < inPcl->points[i].x){
        xMax = inPcl->points[i].x;
      }
      if(yMin > inPcl->points[i].y){
        yMin = inPcl->points[i].y;
      }
      if(yMax < inPcl->points[i].y){
        yMax = inPcl->points[i].y;
      }
    }
  }

  /**
  @brief Updates the ocupation grid parameters
  @param[in] x coordinate of origin of the grid
  @param[in] y coordinate of origin of the grid
  @return void
  */
  void updateGrid(double originX, double originY){
    grid->header.seq++;
    grid->header.stamp.sec = ros::Time::now().sec;
    grid->header.stamp.nsec = ros::Time::now().nsec;
    grid->info.map_load_time = ros::Time::now();
    grid->info.resolution = cellResolution;
    grid->info.width = xCells;
    grid->info.height = yCells;
    grid->info.origin.position.x = originX;
    grid->info.origin.position.y = originY;
    grid->data = ocGrid;
  }

  /**
  @brief Assigns values to the grid's cells according to the ocupation point cloud
  @param[in] Ocupation point cloud
  @param[in] Value to assign to the UNKWON cells
  @return void
  */
  void populateMap(pclPtr inPcl, int color){
    populateMap(inPcl,color, 0.0, 0.0);
  }

  /**
  @brief Assigns values to the grid's cells according to the ocupation point cloud
  @param[in] Ocupation point cloud
  @param[in] Value to assign to the UNKWON cells
  @param[in] x coordinate of origin of the point cloud
  @param[in] y coordinate of origin of the point cloud
  @return void
  */
  void populateMap(pclPtr inPcl, int color, double xOrigin, double yOrigin){
    for(int i = 0; i<inPcl->points.size(); i++){

      geometry_msgs::Point point;
      point.x = inPcl->points[i].x; double x = point.x; x+=xOrigin;
      point.y = inPcl->points[i].y; double y = point.y; y+=yOrigin;
      point.z = 0;

      if(x<xMax && x>xMin && y>yMin && y<yMax){
        int xCell = (int) ((x-xMin)/cellResolution);
        int yCell = (int) ((y-yMin)/cellResolution);

        int idx = yCell*xCells + xCell;
        ocGrid[idx] = 100;
      }

      geometry_msgs::Point point_s = xyzTortp(point);

      for(double k = cellResolution; k<point_s.x; k+=cellResolution){
         geometry_msgs::Point point;
         point.x = k;
         point.y = point_s.y;
         point.z = point_s.z;
         geometry_msgs::Point point_c = rtpToxyz(point);

         double x = point_c.x; x+=xOrigin;
         double y = point_c.y; y+=yOrigin;

         if(x<xMax && x>xMin && y>yMin && y<yMax){
           int xCell = (int) ((x-xMin)/cellResolution);
           int yCell = (int) ((y-yMin)/cellResolution);

           int idx = yCell*xCells + xCell;
           if(ocGrid[idx]==UNKWON){
            ocGrid[idx] = color;
           }else if(ocGrid[idx] == RED && color != RED){
             ocGrid[idx] = YELLOW;
           }
         }
      }
    }
  }

  /**
  @brief Returns the ocupeation grid matrix
  @return vector<signed char>
  */
  vector<signed char> getGrid(){
    return ocGrid;
  }


  /**
  @brief Resets all the cells to a specific value
  @param[in] Value to assign to the cells
  @return void
  */
  void resetGrid(int color){
    ocGrid.assign(xCells * yCells,color);
  }

  /**
  @brief Assignes a matrix to the grid setting all the values and size equal to the matrix
  @param[in] Pointer to the matrix
  @return void
  */
  void assingGrid(vector<signed char> *grid){
    ocGrid = *grid;
  }

  /**
  @brief Publishes the ocupation grid message
  @return void
  */
  void publish(){
    gridPub.publish(*grid);
  }

  /**
  @brief Changes the value of a cell were a given points is in
  @param[in] Value to assign to the cell
  @param[in] x coordinate of the point
  @param[in] y coordinate of the point
  @param[in] Discard ocupied cells
  @return void
  */
  void setValue(int color, double x, double y, bool override){
    if(x<xMax && x>xMin && y>yMin && y<yMax){
      int xCell = (int) ((x-xMin)/cellResolution);
      int yCell = (int) ((y-yMin)/cellResolution);

      int idx = yCell*xCells + xCell;
      if(ocGrid[idx]==50 || override)
       ocGrid[idx] = color;
    }
  }

private:
  nav_msgs::OccupancyGridPtr grid;
  double cellResolution;
  double xMin, yMin, xMax, yMax;
  int xCells;
  int yCells;
  vector<signed char> ocGrid;
  ros::NodeHandle np;
  ros::Publisher gridPub;

  /**
  @brief Initiates the ocupation grid
  @param[in] Name of the reference frame for the grid
  @return void
  */
  void initGrid(string frameId){
    grid->header.seq = 1;
    grid->header.frame_id = frameId;
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 1;
    grid->info.origin.orientation.x = 0;
    grid->info.origin.orientation.y = 0;
    grid->info.origin.orientation.z = 0;
  }
};

/**
@brief Creates a point cloud with a set of points allong a line given two points
@param[in] Initial point
@param[in] Final point
@param[in] Distance between points
@return pcl::PointXYZ
*/
pcl::PointXYZ createPointAllognLine(pcl::PointXYZ ini_point, pcl::PointXYZ last_point, double dist){
  double x1 = ini_point.x; double y1 = ini_point.y;
  double x2 = last_point.x; double y2 = last_point.y;

  double vx = x2 - x1;
  double vy = y2 - y1;
  double mag = sqrt(vx*vx + vy*vy);

  vx /= mag;
  vy /= mag;

  pcl::PointXYZ new_point;
  new_point.x = ((double)x1 + vx * (dist));
  new_point.y = ((double)y1 + vy * (dist));
  new_point.z = 0;

  return new_point;
}


using namespace lidar_data_analise;

/**
   @brief Main function to compute the free space
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_space_detection");

  Publisher polygonPub;
  Publisher uNavLPolygonPub;
  Publisher uNavRPolygonPub;
  Publisher mergedPclPub;
  Publisher uNavPclPub;
  Publisher gridPub;
  NodeHandle np;

  bool includeUnav = true;
  if(np.getParam("includeUnav",includeUnav)){
    if(includeUnav){
      ROS_INFO("Using Nav Area!");
    }else {
      ROS_INFO("Not Using Nav Area!");
    }
  }else{
    ROS_WARN("Param 'includeUnav' not found!");
    ROS_INFO("Using Nav Area!");
  }

  vector< boost::shared_ptr< laserDataAnalise > > laserDataHandle;

  laserDataAnalise lms151DAnalise("/lms151_D_scan", "lms151_D");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&lms151DAnalise));
  laserDataAnalise lms151EAnalise("/lms151_E_scan","lms151_E");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&lms151EAnalise));
  laserDataAnalise ld_mrsAnalise1("/ld_rms/scan0","/ldmrs0");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&ld_mrsAnalise1));
  laserDataAnalise ld_mrsAnalise2("/ld_rms/scan1","/ldmrs1");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&ld_mrsAnalise2));
  laserDataAnalise ld_mrsAnalise3("/ld_rms/scan2","/ldmrs2");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&ld_mrsAnalise3));
  laserDataAnalise ld_mrsAnalise4("/ld_rms/scan3","/ldmrs3");
  laserDataHandle.push_back(boost::shared_ptr< laserDataAnalise >(&ld_mrsAnalise4));

  polygonPub = np.advertise<polygonS>("merged_polygon",1000);
  uNavLPolygonPub = np.advertise<polygonS>("unavl_polygon",1000);
  uNavRPolygonPub = np.advertise<polygonS>("unavr_polygon",1000);
  mergedPclPub = np.advertise<PCL2>("merged_pcl",1000);
  uNavPclPub = np.advertise<PCL2>("unav_pcl",1000);
  gridPub = np.advertise<nav_msgs::OccupancyGrid>("ocupancy_grid",1000);

  vector<int> include;
  include.assign(laserDataHandle.size(),1);
  include[2] = 0; include[3] = 0;
  string includeScans;
  if(np.getParam("IncludeScans",includeScans)){
    if(includeScans.size() == include.size()){
     for(int i = 0; i< includeScans.size();i++){
       char a = includeScans[i];
       include[i] = atoi(&a);
     }
    }else{
      ROS_WARN("Erro in Param 'IncludeScans'");
    }
  }else{
    ROS_WARN("'IncludeScans' Param not found!");
  }

  if(include[0]==1){
    ROS_INFO("Merging scans from: Sick LMS Right!");
  }
  if(include[1]==1){
    ROS_INFO("Merging scans from: Sick LMS Left!");
  }
  if(include[2]==1){
    ROS_INFO("Merging scans from: Sick LD-MRS 0!");
  }
  if(include[3]==1){
    ROS_INFO("Merging scans from: Sick LD-MRS 1!");
  }
  if(include[4]==1){
    ROS_INFO("Merging scans from: Sick LD-MRS 2!");
  }
  if(include[5]==1){
    ROS_INFO("Merging scans from: Sick LD-MRS 3!");
  }

  vector<int> received;
  received.assign(laserDataHandle.size(),0);

  vector< pclPtr > allPcl;
  pclPtr myPcl(new PCL);
  allPcl.assign(laserDataHandle.size(), myPcl);

  pclPtr unavArea_clean(new PCL);
  pclPtr unavAreaL(new PCL);
  pclPtr unavAreaL_t(new PCL);
  pclPtr unavAreaD(new PCL);
  pclPtr unavAreaD_t(new PCL);
  pclPtr car(new PCL);
  if(includeUnav){

    int pointsSize = 500;
    unavAreaL->width = pointsSize;
    unavAreaL->height = 1;
    unavAreaL->points.resize(unavAreaL->width * unavAreaL->height);

    unavAreaD->width = pointsSize;
    unavAreaD->height = 1;
    unavAreaD->points.resize(unavAreaL->width * unavAreaL->height);

    vector<double> angle = linspace(0, M_PI*2, pointsSize);
    double radius = 4.5;

    for(int i = 0; i< pointsSize; i++){
      double x = radius * cos(angle[i]);
      double y = radius * sin(angle[i]);
      if(y>3.95){
        y = 3.95;
      }

      unavAreaL->points[i].x = x;
      unavAreaL->points[i].y = y;
      unavAreaL->points[i].z = 0;
      unavAreaD->points[i].x = x;
      unavAreaD->points[i].y = -y;
      unavAreaD->points[i].z = 0;
    }

    pcl::copyPointCloud(*unavAreaL, *unavAreaL_t);
    pcl::copyPointCloud(*unavAreaD, *unavAreaD_t);

    laserDataAnalise::transformPCL(getTf(-1.5, -(0.65+radius), 0, 0, 0, 0),  unavAreaL_t);
    laserDataAnalise::transformPCL(getTf(-1.5, 0.65+radius, 0, 0, 0, 0),  unavAreaD_t);

    pclPtr unavArea(new PCL);
    *unavArea += *unavAreaD_t;
    *unavArea += *unavAreaL_t;

    pclPtr unavArea_filtered2 (new PCL);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (unavArea);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*unavArea_filtered2);

    unavArea_clean = unavArea_filtered2;

    pclPtr unavAreaD_sorted(new PCL);
    euDistSort2(unavAreaD_t, unavAreaD_sorted);
    pclPtr unavAreaL_sorted(new PCL);
    euDistSort2(unavAreaL_t, unavAreaL_sorted);

    vector<double> side = linspace(0.2, -3.275, 100);
    vector<double> front = linspace(-0.85, 0.85, 50);

    car->width = 100*2+50*2;
    car->height = 1;
    car->points.resize(car->width * car->height);
    int count = 0;
    for(int i = 0; i< side.size(); i++){
      car->points[count].x = side[i];
      car->points[count].y = 0.85;
      car->points[count].z = 0;
      count++;
    }
    for(int i = 0; i< front.size(); i++){
      car->points[count].x = 0.2;
      car->points[count].y = front[i];
      car->points[count].z = 0;
      count++;
    }
    for(int i = 0; i< side.size(); i++){
      car->points[count].x = side[i];
      car->points[count].y = -0.85;
      car->points[count].z = 0;
      count++;
    }
    for(int i = 0; i< front.size(); i++){
      car->points[count].x = -3.275;
      car->points[count].y = front[i];
      car->points[count].z = 0;
      count++;
    }

  }


  double cellResolution = 0.2;
  double xMin = -50; double yMin = -50; double xMax = 50, yMax = 50;

  ocupGrid ocGrid("/map",xMin,xMax,yMin,yMax,cellResolution);

  ocGrid.populateMap(unavAreaL,RED,-1.55,-5.15);
  ocGrid.populateMap(unavAreaD,RED,-1.55,5.15);
  ocGrid.populateMap(car,WHITE);
  ocGrid.setValue(RED,-1.5,-5.15,false);
  ocGrid.setValue(RED,-1.5,5.15,false);

  vector<signed char> map = ocGrid.getGrid();

  Rate  loopRate(50);
  while(ros::ok()){

    for(int i = 0; i<laserDataHandle.size();i++){
      pcl2Ptr cloud_in_ptr(new PCL2);
      bool newData = laserDataHandle[i]->getPcl(cloud_in_ptr);

      if(newData){
        pclPtr cloud(new PCL);
        pcl::fromROSMsg(*cloud_in_ptr, *cloud);
        if(cloud->width>0){
          pclPtr cloud_trans(new PCL);
          laserDataHandle[i]->transformPCL("/map",cloud,cloud_trans);

          allPcl[i]=cloud_trans;
          received[i] = 1;
        }
      }
    }

    pclPtr ldPcl(new PCL);
    pclPtr mergedPcl(new PCL);
    pclPtr mergedPcl_clean(new PCL);


    int arr[] = {-1.6,-0.8,0.8,1.6};
    vector<double> angles(arr, arr+4);
//    angles.insert(angles.end(), { 1, 2, 3, 4, 5, 6 });
    for(int i = 2; i<6; i++){
      if(received[i] == 1 && include[i] == 1){
        pclPtr pcl = allPcl[i];
        if(i == 2 || i == 3)
          removeGround(pcl, angles[i-2],285);

        *mergedPcl += *pcl;
//        *ldPcl += *pcl;
      }
    }

//    if(ldPcl->points.size()>0){
//      removeGround(ldPcl, 0);
//      *mergedPcl += *ldPcl;
//    }

    for(int i = 0; i<2; i++){
      if(received[i] == 1 && include[i] == 1){
        received[i] = 0;
        pclPtr pcl = allPcl[i];
        *mergedPcl += *pcl;
      }
    }

    if(unavArea_clean->points.size()>0){
      PCL2 mergedPcl2;
      pcl::toROSMsg(*unavArea_clean,mergedPcl2);
      mergedPcl2.header.frame_id = "/map";
      uNavPclPub.publish(mergedPcl2);

      polygonSPtr polygonD = laserDataAnalise::getScanPolygon(unavAreaD_t,"/map");
      polygonSPtr polygonL = laserDataAnalise::getScanPolygon(unavAreaL_t,"/map");

      /*---Publisher for the Polygon---*/
      uNavRPolygonPub.publish(*polygonD);
      uNavLPolygonPub.publish(*polygonL);
    }


    if(mergedPcl->points.size()>0){

      pclPtr mergedPcl_sorted(new PCL);
      sortPcl(mergedPcl, mergedPcl_sorted);

      // Build the condition
      pcl::ConditionOr<pcl::PointXYZ>::Ptr range_and1 (new pcl::ConditionOr<pcl::PointXYZ> ());
      range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.9)));
      range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -0.9)));
      range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, -3.5)));
      range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.2)));

      pclPtr mergedPcl_filtered (new PCL);
      // Build the filter
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem(true);
      condrem.setCondition (range_and1);
      condrem.setInputCloud (mergedPcl_sorted);
      condrem.filter (*mergedPcl_filtered);

      azimuteFilter(mergedPcl_filtered, mergedPcl_clean, true );
    }

    if(mergedPcl_clean->points.size()>0){
      pclPtr polyPcl(new PCL);
      for(int i = 0; i<mergedPcl_clean->points.size(); i++){
        pcl::PointXYZ point1 = mergedPcl_clean->points[i]; point1.z = 0;
        pcl::PointXYZ point2;
        if(i<mergedPcl_clean->points.size()-1){
          point2 = mergedPcl_clean->points[i+1]; point2.z = 0;
        }else{
          point2 = mergedPcl_clean->points[0]; point2.z = 0;
        }
        polyPcl->points.push_back(point1);
        double lenght = pointsDist(point1, point2);
        for(double k = 0.2; k<lenght; k+=0.2){
          pcl::PointXYZ point  = createPointAllognLine(point1,point2, k);
          polyPcl->points.push_back(point);
        }
      }

      polygonSPtr polygon = laserDataAnalise::getScanPolygon(mergedPcl_clean,"/map");

      /*---Publisher for the Polygon---*/
      polygonPub.publish(*polygon);

      PCL2 mergedPcl2;
      pcl::toROSMsg(*polyPcl,mergedPcl2);
      mergedPcl2.header.frame_id = "/map";
      mergedPclPub.publish(mergedPcl2);

      ocGrid.assingGrid(&map);
      ocGrid.populateMap(polyPcl,GREEN);

      double originX = xMin;
      double originY = yMin;
      ocGrid.updateGrid(originX,originY);

      ocGrid.publish();
    }

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
