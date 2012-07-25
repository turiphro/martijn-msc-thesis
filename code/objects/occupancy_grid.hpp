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

using namespace std;
using namespace pcl;
using namespace octomap;

class OccupancyGrid
{
  private:
    OcTree* tree;
    SfMReader* sfm;

  public:
    OccupancyGrid(string path, string imagespath="", int resolution=250);
    ~OccupancyGrid();
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
    void pcl2octomap(PointXYZRGB pcl, point3d& octomap);
    void octomap2pcl(point3d octomap, PointXYZRGB& pcl);

};

#endif
