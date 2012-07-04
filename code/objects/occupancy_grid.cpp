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
  if (max(axes[1] > max_axis_length)) max_axis_length = axes[1];
  if (max(axes[2] > max_axis_length)) max_axis_length = axes[2];

  tree = new OcTree(max_axis_length / resolution);
}

OccupancyGrid::~OccupancyGrid()
{
  delete sfm;
}

bool OccupancyGrid::carve()
{
  if (sfm->poses.size() == 0) {
    cerr << "[!!] No camera poses known!" << endl;
    return false;
  }
  if (sfm->visible.size() < sfm->poses.size()) {
    cerr << "[!!] No or not enough visibility information!" << endl;
    return false;
  }
  
  point3d origin, end;
  map<int,visibility>* vismap;
  map<int,visibility>::iterator it;
  int frame;
  // for every point ..
  for (int p=0; p<sfm->points.size(); p++) {
    // .. carve for sequence of camera poses
    // TODO: make something better than just carving rays
    vismap = &sfm->visible.at(p);
    for (it = vismap->begin(); it!=vismap->end(); it++) {
      frame = (*it).first;
      pcl2octomap(sfm->poses.at(frame), origin);
      pcl2octomap(sfm->points.at(p), end);
      tree->insertRay(origin, end);
    }
  }
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
