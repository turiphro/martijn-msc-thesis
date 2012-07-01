#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../io/sfm_reader.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <path>" << endl;
  cout << " where path is the path to an nvm, ply, out or txt file." << endl;
  exit(1);
}


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* sfm_ptr)
{
  SfMReader* sfm = (SfMReader*) sfm_ptr;
  int id = event.getPointIndex();
  if (id == -1)
    return;

  float x, y, z;
  event.getPoint (x, y, z);

  if (sfm->poses.size() > id &&
      sfm->poses.at(id).x == x &&
      sfm->poses.at(id).y == y &&
      sfm->poses.at(id).z == z) {
    // update camera view
    cout << "it's a camera!" << endl;
    sfm->selectPointsForCamera(id);
  } else {
    cout << "it's a point!" << endl;
    sfm->selectCamerasForPoint(id);
  }

  sfm->updated = true;
}


/* main */
int main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  SfMReader sfm(argv[1]);

  /* visualise:
   *  CloudViewer (simple): http://pointclouds.org/documentation/tutorials/cloud_viewer.php
   *  PCLVisualizer (adv):  http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
   *   \__ has line and sphere draw functions; lots of info down at page
   *
   * functions:
   * - draw cameras
   * - click camera: colour visible points
   * - click point:  colour corresponding cameras
   * - view from camera
   */

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(&sfm.points);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses(&sfm.poses);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  //    poses_colours(poses, 255, 255, 0);

  pcl::visualization::PCLVisualizer viewer("Cloud viewer");
  viewer.registerPointPickingCallback(pp_callback, &sfm);

  viewer.addPointCloud(points, "points");
  viewer.addPointCloud(poses, "poses");
  viewer.setBackgroundColor(0,0,0);
  viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "points");
  viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "poses");
  //viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();


  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);

    if (sfm.updated) {
      cout << "Updated!" << endl;
      viewer.updatePointCloud(points, "points");
      viewer.updatePointCloud(poses, "poses"); 
      sfm.updated = false;
    }
  }


  // more drawing: use viewer.getRenderWindow() and use VTK

  return 0;
}


