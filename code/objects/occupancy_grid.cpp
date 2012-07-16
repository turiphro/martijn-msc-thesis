#include "occupancy_grid.hpp"

OccupancyGrid::OccupancyGrid(string path, int resolution)
{
  sfm = new SfMReader(path);

  // subdivide longest axis into given resolution
  Scalar min, max, axes;
  double max_axis_length;
  sfm->getExtrema(min, max);
  axes = max - min;
  max_axis_length = axes[0];
  if (axes[1] > max_axis_length) max_axis_length = axes[1];
  if (axes[2] > max_axis_length) max_axis_length = axes[2];

  tree = new OcTree(max_axis_length / resolution);
}

OccupancyGrid::~OccupancyGrid()
{
  delete sfm;
}

/* General Carving function.
 * If exportUnknowns is set to true, the final result will
 * be altered such that the unknowns are set to occupied.
 * If exportOccupied is set to false, the final result will
 * be altered such that the occupied voxels are set to
 * unknown.
 */
bool OccupancyGrid::carve(bool exportUnknowns,
                          bool exportOccupied,
                          int method)
{
  if (sfm->poses.size() == 0) {
    cerr << "[!!] No camera poses known!" << endl;
    return false;
  }
  if (sfm->visible.size() < sfm->poses.size()) {
    cerr << "[!!] No or not enough visibility information!" << endl;
    return false;
  }
  
  switch (method) {
    case 0:
      return carveSingleRay(exportUnknowns, exportOccupied);
      break;
    default:
      cerr << "[!!] Error: unknown carving method (" << method << ")" << endl;
      return false;
  }

}

bool OccupancyGrid::carveSingleRay(bool exportUnknowns,
                                   bool exportOccupied)
{
  point3d origin, end;
  map<int,visibility>* vismap;
  map<int,visibility>::iterator it;
  int frame;
  // for every point ..
  for (int p=0; p<sfm->points.size(); p++) {
    // .. carve for sequence of camera poses
    vismap = &sfm->visible.at(p);
    for (it = vismap->begin(); it!=vismap->end(); it++) {
      frame = (*it).first;
      pcl2octomap(sfm->poses.at(frame), origin);
      pcl2octomap(sfm->points.at(p), end);
      tree->insertRay(origin, end);
    }
  }
  // change unknowns or occupied voxels for desired output
  if (exportUnknowns || !exportOccupied) {
    // iterate through leaf nodes and alter state if necessary
    for (OcTree::leaf_iterator it = tree->begin_leafs(),
         end=tree->end_leafs(); it!=end; it++) {
      if (exportUnknowns
          && !tree->isNodeAtThreshold(*it)
          && !tree->isNodeOccupied(*it)) {
        it->setValue(tree->getOccupancyThresLog());
      } else if (!exportOccupied
          && tree->isNodeOccupied(*it)) {
        it->setValue(tree->getOccupancyThresLog()-1); // TODO: rem hack
      }

    }
  }
  // update up-tree
  tree->updateInnerOccupancy();
  return true;
}

bool OccupancyGrid::save(string filename, bool binary)
{
  if (binary)
    return tree->writeBinary(filename);
  else
    return tree->write(filename);
}

void OccupancyGrid::pcl2octomap(PointXYZRGB& pcl, point3d& octomap)
{
  octomap.x() = pcl.x;
  octomap.y() = pcl.y;
  octomap.z() = pcl.z;
}

void OccupancyGrid::octomap2pcl(point3d& octomap, PointXYZRGB& pcl)
{
  pcl.x = octomap.x();
  pcl.y = octomap.y();
  pcl.z = octomap.z();
  pcl.r = 150;
  pcl.g = 150;
  pcl.b = 150;
}
