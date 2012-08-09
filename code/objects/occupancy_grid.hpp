/* Occupancy grid; useful for carving */

/* Implementation: (TODO)
 * - own
 *    can use PCL VoxelGrid (now used as downsample filter):
 *    http://docs.pointclouds.org/trunk/classpcl_1_1_voxel_grid.html
 * - Octree PCL:        [looks binary; leafnodes are occupancy;
 *                       more general than octomap acc. to forum]
 *    http://docs.pointclouds.org/trunk/classpcl_1_1octree_1_1_octree_point_cloud_occupancy.html
 *    http://www.pcl-users.org/Volume-estimation-td2807985.html
 * - OctoMap library:   [occupied, free, unknown; probabilistic possible]
 *    incl. viewer (octovis)
 *    http://octomap.sourceforge.net/
 *    http://octomap.sourceforge.net/doxygen/
 *    \_ carving: insertRay (uses integrateMissOnRay) for freeing
 *                a ray of voxels
 *    \_ visualisation: example code; castRay
 *    http://www.pcl-users.org/OctoMap-and-PCL-Question-td3850053.html
 */

#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <opencv2/core/core.hpp>
#include "../io/sfm_reader.hpp"
#include "../io/sequence_capture.hpp"
#include "../objects/listhelpers.cpp"

using namespace std;
using namespace pcl;
using namespace octomap;


typedef struct projected_voxel
{
  double x, y;
  double r;
  double distance;
  
} projected_voxel;


bool projected_voxel_comp(projected_voxel a, projected_voxel b);

class OccupancyGrid
{
  public:
    OcTree* tree;
    SfMReader* sfm;

    OccupancyGrid(string path, string imagespath="", int resolution=250);
    ~OccupancyGrid();
    bool load(string path);
    bool carve(bool exportUnknowns=false,
               bool exportOccupied=true,
               int method=0,
               double param1=0.1);
    bool carveSingleRay(double ignoredBorderSize=0.1,
                        bool exportUnknowns=false,
                        bool exportOccupied=true);
    bool carveSingleRayInvisible(double occluderProbAddition=0.1,
                                 bool exportOccupied=true);
    bool save(string filename, bool binary=false);
    void graphcut(double gamma=0.5);
    void extentVisibilityLists(double threshold=0.2);
    double reprojectMatch(Mat* img1, Mat* img2, 
                          camera* cam1, camera* cam2,
                          PointXYZRGB point,
                          PointXYZRGB pose,
                          double voxel_size,
                          string method="L2",
                          bool showImgs=false);
    double patchDistance(Mat* patch1, Mat* patch2, string method="L2");
    void projectVoxel(PointXYZRGB* point,
                      double voxel_size, camera* cam,
                      PointXYZRGB* centre, double* r);

    void pcl2octomap(PointXYZRGB pcl, point3d& octomap);
    void octomap2pcl(point3d octomap, PointXYZRGB& pcl);

    void visualise(Mat& output, camera* cam,
                   bool redraw=true,
                   string method="circle", double alpha=0.5);
    bool visualisePose(Mat& output, int poseID,
                       string method="circle", double alpha=0.5);
    void visualisePath(vector<Mat>* output,
                       string method="circle", double alpha=0.5);
    void visualisePath(vector<Mat>* output, vector<camera>* path,
                       string method="circle", double alpha=0.5);

};

#endif
