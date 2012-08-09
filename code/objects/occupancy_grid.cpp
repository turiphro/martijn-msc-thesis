#include "occupancy_grid.hpp"

bool projected_voxel_comp(projected_voxel a, projected_voxel b)
{
  return a.distance < b.distance;
}

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


// Delete current octree and load in another
// Note: still using original sfm file and image directory!
bool OccupancyGrid::load(string path)
{
  delete tree;
  AbstractOcTree* abstract = AbstractOcTree::read(path);
  tree = dynamic_cast<OcTree*>(abstract);
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
  
  cout << "Carving.." << endl;
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
  cout << " a. carve" << endl;
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
  cout << " b. visualise" << endl;
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
  for (int c=0; c<sfm->poses.size(); c++) {
    cout << "\r >> " << c+1 << "/" << sfm->poses.size();
    cout.flush();
    sfm->selectPointsForCamera(c);
    // .. check, for each point, if it is either visible
    //    or - if not - at least lies inside the camera window
    for (int p=0; p<sfm->points.size(); p++) {
      if (sfm->points_curr_visible.at(p)) {
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


void OccupancyGrid::graphcut(double gamma)
{

  // TODO

  cout << "TODO: graphcut" << endl;

}

/* extent the visibility lists in sfmreader using
 * reprojection and distance between patches around 
 * the projected pixels
 */
void OccupancyGrid::extentVisibilityLists(double threshold)
{
  cout << "Extending visibility lists.." << endl;
  Size window_size;
  sfm->getWindowSize(window_size);
  double voxel_size = tree->getResolution();

  // load all images (memory intensive!)
  if (sfm->imagespath == "") {
    cerr << "[!!] Error: image path not given" << endl;
    return;
  }
  vector<Mat> images;
  images.push_back(Mat());
  SequenceCapture sc(sfm->imagespath);
  while (sc.read(images.back()))
    try {
      images.push_back(Mat());
    } catch (exception& e) {
      cerr << "[!!] Error loading images: " << e.what() << endl;
    }
  images.pop_back();
  //cout << " >> " << images.size() << " images in memory" << endl;
  
  // for each point ..
  ListHelper LH;
  bool visible[sfm->poses.size()];
  bool invisible[sfm->poses.size()];
  for (int p=0; p<sfm->points.size(); p++) {
    cout << "\r >> " << p+1 << "/" << sfm->points.size();
    cout.flush();
    for (int c=0; c<sfm->poses.size(); c++) {
      visible[c] = sfm->visible.at(p).find(c) != sfm->visible.at(p).end();
      invisible[c] = (!visible[c] && sfm->reprojectsInsideImage(p, c, window_size));
    }

    // .. follow in sequence
    double distance;
    double dist_cam_bw, dist_cam_fw;
    int item, nearest, nearest_bw, nearest_fw;
    item = LH.bool_find_first(visible, sfm->poses.size());
    
    for (int i=0; i<sfm->poses.size(); i++) {
      // if point reprojects inside this image, but was marked invisible:
      if (invisible[i]) {
        // 1. find nearest camera in which it was visible ..
        //    (heuristic: search nearest in both sides, take euclidean nearest)
        nearest_fw = LH.bool_find_next(visible, sfm->poses.size(), i);
        nearest_bw = LH.bool_find_prev(visible, sfm->poses.size(), i);
        if (nearest_fw < 0 && nearest_bw < 0)
          continue; // point never visible (this should never happen)
        else if (nearest_fw < 0)
          nearest = nearest_bw;
        else if (nearest_bw < 0)
          nearest = nearest_fw;
        else {
          dist_cam_bw = pow(sfm->poses.at(nearest_bw).x - sfm->poses.at(i).x,2)
                      + pow(sfm->poses.at(nearest_bw).y - sfm->poses.at(i).y,2);
          dist_cam_fw = pow(sfm->poses.at(nearest_fw).x - sfm->poses.at(i).x,2)
                      + pow(sfm->poses.at(nearest_fw).y - sfm->poses.at(i).y,2);
          nearest = (dist_cam_fw>dist_cam_bw) ? nearest_bw : nearest_fw;
        }
        // 2. calculate reprojection match measure
        distance = reprojectMatch(&images.at(i),
                                  &images.at(nearest),
                                  &sfm->cameras.at(i),
                                  &sfm->cameras.at(nearest),
                                  sfm->points.at(p),
                                  sfm->poses.at(i),
                                  voxel_size,
                                  "L2", false);
        // 3. if the reprojection patches seem to match,
        //    add 'invisible' camera pose to visibility list!
        if (distance < threshold) {
          PointXYZRGB reproj;
          sfm->reproject(&sfm->points.at(p), &sfm->cameras.at(i), &reproj);
          sfm->distortPointR1(&reproj, &sfm->cameras.at(i));
          sfm->visible.at(p).insert(pair<int,visibility>(i,visibility()));
          sfm->visible.at(p).find(i)->second.index = -1; // i don't know
          sfm->visible.at(p).find(i)->second.x = reproj.x;
          sfm->visible.at(p).find(i)->second.y = reproj.y;
        }
      }
    }

  }
  cout << endl;
}

double OccupancyGrid::reprojectMatch(Mat* img1, Mat* img2, camera* cam1, camera* cam2, PointXYZRGB point, PointXYZRGB pose, double voxel_size, string method, bool showImgs)
{

  // find patch centres
  PointXYZRGB centr1, centr2;
  double patch_size;
  projectVoxel(&point, voxel_size, cam1, &centr1, &patch_size);
  sfm->reproject(&point, cam2, &centr2);
  sfm->distortPointR1(&centr2, cam2);
  

  int x1_start = int(centr1.x - patch_size + img1->size().width/2.0);
  int x1_stop  = int(centr1.x + patch_size + img1->size().width/2.0);
  int y1_start = int(centr1.y - patch_size + img1->size().height/2.0);
  int y1_stop  = int(centr1.y + patch_size + img1->size().height/2.0);
  int x2_start = int(centr2.x - patch_size + img2->size().width/2.0);
  int x2_stop  = x2_start + (x1_stop - x1_start);
  int y2_start = int(centr2.y - patch_size + img2->size().height/2.0);
  int y2_stop  = y2_start + (y1_stop - y1_start);

  // check if inside image and not near edge
  if (x1_start <= 0 || x1_stop > img1->size().width
   || y1_start <= 0 || y1_stop > img1->size().height
   || x2_start <= 0 || x2_stop > img2->size().width
   || y2_start <= 0 || y2_stop > img2->size().height)
    return -1;

  // calculate distance between patches
  Mat patch1 = (*img1)(Range(y1_start, y1_stop), Range(x1_start, x1_stop));
  Mat patch2 = (*img2)(Range(y2_start, y2_stop), Range(x2_start, x2_stop));
  double distance = patchDistance(&patch1, &patch2, method);

  if (showImgs && distance >= 0) { 
    // /*
    cout << "D = " << distance << endl;
    
    namedWindow("IMG1", CV_WINDOW_NORMAL);
    namedWindow("IMG2", CV_WINDOW_NORMAL);
    Mat image1 = img1->clone();
    Mat image2 = img2->clone();
    rectangle(image1, Point(x1_start, y1_start), Point(x1_stop, y1_stop), Scalar(255,0,0), 4);
    rectangle(image2, Point(x2_start, y2_start), Point(x2_stop, y2_stop), Scalar(255,0,0), 4);
    circle(image1,
      Point((x1_start+x1_stop)/2, (y1_start+y1_stop)/2),
      5, Scalar(0,0,255), 4);
    circle(image2,
      Point((x2_start+x2_stop)/2, (y2_start+y2_stop)/2),
      5, Scalar(0,0,255), 4);
    imshow("IMG1", image1);
    imshow("IMG2", image2);
    while (32 != waitKey(0))
      ; // wait for space
    destroyWindow("IMG1");
    destroyWindow("IMG2");
    // */

  }

  return distance;

}

// Normalised average r,g,b, intensity difference
// Note: assuming images are represented by 8bit unsigned integers
double OccupancyGrid::patchDistance(Mat* patch1, Mat* patch2, string method)
{
  double distance = 0;
  double r, g, b;
  for (int x=1; x<=patch1->size().width; x++) {
    for (int y=1; y<=patch1->size().height; y++) {
      r = patch1->at<Vec3b>(x, y)[0] - patch2->at<Vec3b>(x, y)[0];
      g = patch1->at<Vec3b>(x, y)[1] - patch2->at<Vec3b>(x, y)[1];
      b = patch1->at<Vec3b>(x, y)[2] - patch2->at<Vec3b>(x, y)[2];
      
      distance += pow( double(r) / pow(2,8), 2 );
      distance += pow( double(g) / pow(2,8), 2 );
      distance += pow( double(b) / pow(2,8), 2 );
      //distance += abs( double(img1->at<unsigned int>(x1, y1) - img2->at<unsigned int>(x2, y2)) / pow(2,32) );
    }
  }

  return sqrt( distance / (3*patch1->size().width*patch1->size().height) );
  
}


/* determine patch size (approximation):
 * project point with distance voxelSize/2 from point inside img1
 * and decide patch size based on distance from projected point
 * Note: assuming same patch size for both images for easy comparing
 * (small camera displacement assumption)
 * Outputs: centre (projected point), r (projected voxel size)
 */
void OccupancyGrid::projectVoxel(PointXYZRGB* point, double voxel_size, camera* cam, PointXYZRGB* centre, double* r)
{
  PointXYZRGB pose;
  pose.x = cam->t.at<double>(0);
  pose.y = cam->t.at<double>(1);
  pose.z = cam->t.at<double>(2);
  // project point to get centre output
  sfm->reproject(point, cam, centre);
  sfm->distortPointR1(centre, cam);

  PointXYZRGB offset_vect, pointpose;
  // offset_point = point + offset_vect
  //              = point + 0.5*voxel_size * ( (point - pose) X [1 0 0]' )
  pointpose.x = point->x - pose.x;
  pointpose.y = point->y - pose.y;
  pointpose.z = point->z - pose.z;
  double offset_vect_length = sqrt(pow(pointpose.z,2)+pow(pointpose.y,2));
  offset_vect.x = 0;
  
  offset_vect.y = pointpose.z *  0.5*voxel_size / offset_vect_length;
  offset_vect.z = pointpose.y * -0.5*voxel_size / offset_vect_length;
  PointXYZRGB offset_point;
  offset_point.x = point->x + offset_vect.x;
  offset_point.y = point->y + offset_vect.y;
  offset_point.z = point->z + offset_vect.z;

  PointXYZRGB offset_proj;
  sfm->reproject(&offset_point, cam, &offset_proj);
  sfm->distortPointR1(&offset_proj, cam);

  // calculate approximated size of projected voxel
  *r = sqrt(pow(centre->x - offset_proj.x, 2)
          + pow(centre->y - offset_proj.y, 2));
}


void OccupancyGrid::visualise(Mat& output, camera* cam, bool redraw, string method, double alpha)
{

  Mat annotated = output.clone();
  vector<projected_voxel> voxels;

  // reproject all voxels into image
  PointXYZRGB centre, projected, pose;
  pose.x = cam->t.at<double>(0);
  pose.y = cam->t.at<double>(1);
  pose.z = cam->t.at<double>(2);
  Size window_size;
  sfm->getWindowSize(window_size);
  double voxel_size = tree->getResolution();
  double r, distance;
  for (OcTree::leaf_iterator it = tree->begin_leafs(),
       end=tree->end_leafs(); it!=end; it++) {
    if (tree->isNodeOccupied(*it)) {
      octomap2pcl(it.getCoordinate(), centre);
      if (sfm->reprojectsInsideImage(&centre, cam, window_size, &projected)) {
        // if visible, save to list
        // TODO: calc distance
        sfm->distortPointR1(&projected, cam);
        projectVoxel(&centre, voxel_size, cam, &projected, &r);
        distance = sqrt(pow(centre.x - pose.x, 2)
                      + pow(centre.y - pose.y, 2)
                      + pow(centre.z - pose.z, 2));
        voxels.push_back(projected_voxel());
        voxels.back().x = projected.x + annotated.size().width/2.0;
        voxels.back().y = projected.y + annotated.size().height/2.0;
        voxels.back().r = r;
        voxels.back().distance = distance;
      }
    }
  }

  // sort list based on distance
  sort(voxels.begin(), voxels.end(), projected_voxel_comp);

  // draw list on annotated image from far to near
  // (alternatively, one could check for occlusions
  //  for each voxel during list creation above)
  Scalar colour1(255,0,0);
  Scalar colour2(0,0,255);
  Scalar colour;
  double cmin = log(voxels.front().distance);
  double cmax = log(voxels.back().distance);
  double rate;
  for (int i=voxels.size()-1; i>=0; i--) {
    rate = (log(voxels[i].distance) - cmin) / (cmax - cmin);
    colour = Scalar(
           rate * colour1[0] + (1-rate) * colour2[0],
           rate * colour1[1] + (1-rate) * colour2[1],
           rate * colour1[2] + (1-rate) * colour2[2]);
    if (method == "circle")
      circle(annotated, Point(voxels[i].x, voxels[i].y), voxels[i].r, colour, -1);
    else if (method == "rectangle")
      rectangle(annotated, Point(voxels[i].x - voxels[i].r,
                                 voxels[i].y - voxels[i].r),
                           Point(voxels[i].x + voxels[i].r,
                                 voxels[i].y + voxels[i].r),
                           colour, -1);
    else
      cerr << "[!!] Error: unknown visualisation method: " << method << endl;

  }


  // combine original and annotated images
  addWeighted(output, alpha, annotated, 1-alpha, 0, output);

}


bool OccupancyGrid::visualisePose(Mat& output, int poseID, string method, double alpha)
{
  // use order as given by sequence capture (sorted directory listing)
  // (we need to correct the cameraID for this)
  string filename = sfm->sc->images.at(poseID);
  int camID = sfm->getImageID(filename);
  if (poseID < 0) {
    cerr << "[!!] Error: cannot find image file " << filename << endl;
    return false;
  }
  sfm->sc->setPosition(poseID);
  sfm->sc->read(output);
  visualise(output, &sfm->cameras.at(camID), false, method, alpha);
  return true;
}


// Visualise (project) voxels onto images in the camera pose path
// (output vector should be empty)
void OccupancyGrid::visualisePath(vector<Mat>* output, string method, double alpha)
{
  output->clear();

  for (int i=0; i < sfm->poses.size(); i++) {
    output->push_back(Mat());
    visualisePose(output->at(i), i, method, alpha);
  }

}


// Visualise (project) voxels onto images in vector output for given path
void OccupancyGrid::visualisePath(vector<Mat>* output, vector<camera>* path, string method, double alpha)
{
  if (path->size() > 0 && path->size() != output->size()) {
    cerr << "[!!] Error: output is supposed to be the same size";
    cerr << " as path, or empty (OccupancyGrid::visualisePath)" << endl;
    return;
  }

  for (int i=0; i<path->size(); i++) {
    if (output->size() <= i) {
      output->push_back(Mat());
      visualise(output->at(i), &path->at(i), true, method, alpha);
    } else {
      visualise(output->at(i), &path->at(i), false, method, alpha);
    }
  }

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
