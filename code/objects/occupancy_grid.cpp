#include "occupancy_grid.hpp"

OccupancyGrid::OccupancyGrid(string path, string imagespath, int resolution)
{
  sfm = new SfMReader(path, imagespath);

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
                          int method,
                          double param1)
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
      return carveSingleRay(param1, exportUnknowns, exportOccupied);
      break;
    case 1:
      return carveSingleRayInvisible(param1, exportOccupied);
      break;
    default:
      cerr << "[!!] Error: unknown carving method (" << method << ")" << endl;
      return false;
  }

}

/* Carve using rays from camera poses to points
 * (ray sets every hitting voxel to free and point
 * voxel to occupied; the others are unknown and
 * therefore probably occluders.
 * Most simple and fast method (only one ray per
 * camera-point pair, forall points: forall cameras)
 * If exportUnknowns is set, only the unknown voxels
 * that reproject in at least one camera view are set
 * to occupied threshold. To ignore points projected
 * near camera view borders, set ignoredBorderSize != 0.0.
 */
bool OccupancyGrid::carveSingleRay(double ignoredBorderSize,
                                   bool exportUnknowns,
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
  PointXYZRGB centre;
  Size subwindow_size;
  sfm->getWindowSize(subwindow_size);
  subwindow_size.height = (1.0 - ignoredBorderSize) * subwindow_size.height;
  subwindow_size.width  = (1.0 - ignoredBorderSize) * subwindow_size.width;
  if (exportUnknowns || !exportOccupied) {
    // update up-tree
    tree->updateInnerOccupancy();
    // iterate through leaf nodes and alter state if necessary
    for (OcTree::leaf_iterator it = tree->begin_leafs(),
         end=tree->end_leafs(); it!=end; it++) {
      if (exportUnknowns
          && !tree->isNodeAtThreshold(*it)
          && !tree->isNodeOccupied(*it)) {
        /* Only set this unknown state node to occupied if the
         * centre is inside at least one camera view
         * (or don't initialise nodes that are not visible
         * earlier on)
         * Note: this is the bottle neck of the carve0 algorithm
         *       O(n^3) with n = resolution
         */
        // TODO: this hugely decreases speed (for higher resolution)
        octomap2pcl(it.getCoordinate(), centre);
        for (int c=0; c<sfm->cameras.size(); c++) {
          if (sfm->reprojectsInsideImage(&centre,
                    &sfm->cameras.at(c),
                    subwindow_size))
          {
            it->setValue(tree->getOccupancyThresLog());
            break;
          }
        }
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

bool OccupancyGrid::carveSingleRayInvisible(double occluderProbAddition,
                                            bool exportOccupied)
{
  point3d origin, end;
  map<int,visibility>* vismap;
  map<int,visibility>::iterator it;
  int frame;
  KeyRay ray;
  OcTreeNode* node;
  // find window size
  Size window_size;
  sfm->getWindowSize(window_size);

  // for every camera pose ..
  cout << "Carving.." << endl;
  for (int c=0; c<sfm->poses.size(); c++) {
    cout << "\b\b\b\b\b\b\b" << c+1 << "/" << sfm->poses.size();
    cout.flush();
    sfm->selectPointsForCamera(c);
    // .. check, for each point, if it is either visible
    //    or - if not - at least lies inside the camera window
    for (int p=0; p<sfm->points.size(); p++) {
      if (sfm->points_curr_visibility.at(p)) {
        // if it is visible (thus inside camera window):
        // carve ray (binary)
        // TODO: or: decrease occluder probability on the ray
        pcl2octomap(sfm->poses.at(c), origin);
        pcl2octomap(sfm->points.at(p), end);
        if (exportOccupied) {
          // mark end point as occupied
          tree->insertRay(origin, end);
        } else {
          // don't mark end point as occupied
          // unfortunately, this method is protected:
          //tree->integrateMissOnRay(origin, end);
          // therefore we hack:
          node = tree->search(end);
          double value;
          if (node)
            value = node->getValue();
          tree->insertRay(origin, end);
          if (node)
            node->setValue(value);
        }
      } else if (sfm->reprojectsInsideImage(p, c, window_size)) {
        // if it is not visible but lies inside camera window:
        // increase occluder probability on the ray
        pcl2octomap(sfm->poses.at(c), origin);
        pcl2octomap(sfm->points.at(p), end);
        // get voxels we hit in between
        ray.reset();
        tree->computeRayKeys(origin, end, ray);
        for (KeyRay::const_iterator it = ray.begin();
             it != ray.end(); it++) {
          node = tree->search(*it);
          // if a voxel is not carved away yet, increase occupancy probability
          if (node && exp(node->getValue()) > tree->getProbMiss()) {
            node->setValue( log( exp(node->getValue()) + occluderProbAddition) );
          }
        }
      }
    } // end for ev point
  } // end for ev camera
  cout << endl;

  /*
  bool exportUnknowns = true;
  // change occupied voxels for desired output
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
  */

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

void OccupancyGrid::pcl2octomap(PointXYZRGB pcl, point3d& octomap)
{
  octomap.x() = pcl.x;
  octomap.y() = pcl.y;
  octomap.z() = pcl.z;
}

void OccupancyGrid::octomap2pcl(point3d octomap, PointXYZRGB& pcl)
{
  pcl.x = octomap.x();
  pcl.y = octomap.y();
  pcl.z = octomap.z();
  pcl.r = 150;
  pcl.g = 150;
  pcl.b = 150;
}
