#include "sfm_reader.hpp"

/* Outputs:
 *
 *                 cameras  points   visibility     sift in:
 * VisualSFM: NVM     x       x          x          .sift files
 *            PLY     -       x          -
 * Bundler:   OUT     x       x          x          .tar.gz files
 *            PLY     -       x          -
 * Voodoo:    TXT     x       x          -          .key files
 *                                                  (active export)
 * [PCL]:     PCD     -       x          -          -
 *            
 *                                                  (might want to use
 *                                                  SIFT descriptors too)
 */

SfMReader::SfMReader()
{
  SfMReader("");
}

SfMReader::SfMReader(string path, string imagespath)
{
  this->path = path;
  this->imagespath = imagespath;
  window_size = Size(0,0);
  read();
}

SfMReader::~SfMReader()
{
}


bool SfMReader::read()
{
  if (path == "")
    return false;

  return read(path);
}

bool SfMReader::read(string path)
{
  if (path.find_last_of(".") == string::npos)
    return false;

  // save filename if not saved earlier
  if (this->path == "")
    this->path = path;

  // call appropiate function for file extension
  ext = path.substr(path.find_last_of(".") + 1);

  if (ext == "nvm")
    return readNVM(); // VisualSFM 
  else if (ext == "out")
    return readOUT(); // Bundler
  else if (ext == "ply")
    return readPLY(); // VisualSFM/Bundler
  else if (ext == "txt")
    return readTXT(); // Voodoo
  else
    cerr << "[!!] (SfMReader::read) unknown file format: " << ext << endl;

  return true;
}

bool SfMReader::readNVM()
{
  /* output from: VisualSFM
   * contains: cameras, points, visibility
   * note: only reading first model
   * format:
   *  http://www.cs.washington.edu/homes/ccwu/vsfm/doc.html#nvm
   *  0: <header>          (3 lines)
   *  1: <cameras>         (1 line each)
   *  2: <header>          (2 lines)
   *  3: <points pos+col>  (1 line each)  + <visibility ijxy>  <-- many
   *  4: <header>          (9+ lines)
   */

  ifstream file(path.c_str());
  if (!file.is_open())
    return false;

  int camera_count, point_count;

  int state = 0; // for states see comment above
  string line;
  while (file.good()) {
    getline(file, line);
    stringstream sline(line);
    string filename;
    Mat q(4, 1, CV_64FC1);
    switch (state) {
      case 0: // header
        if (integerLine(line, 1)) {
          sline >> camera_count;
          state = 1;
        }
        break;
      case 1: // cameras
        if (camera_count-- == 0) {
          state = 2;
          break;
        }
        // new camera
        double cx, cy, cz;  // pos: position
        double fx,fy,fz;    // focal: lookat
        double vx,vy,vz;    // view: up vector
        double fovy;        // fovy: FoV in y direction
        double focal, q1,q2,q3,q4, radial;
        sline >> filename >> focal >> q1>>q2>>q3>>q4 >> cx >> cy >> cz >> radial;
        q.at<double>(0) = q1; q.at<double>(1) = q2;
        q.at<double>(2) = q3; q.at<double>(3) = q4;

        // add new camera to list of cameras
        cameras.push_back(camera());
        cameras.back().t = Mat(3, 1, CV_64FC1);
        // apparently, t = -1 * t in nvm; we need to correct this:
        cameras.back().t.at<double>(0) = -1 * cx;
        cameras.back().t.at<double>(1) = -1 * cy;
        cameras.back().t.at<double>(2) = -1 * cz;
        cameras.back().R = Mat(3, 3, CV_64FC1);
        quaternion2matrix(q, cameras.back().R);
        cameras.back().focal = focal;
        cameras.back().radial[0] = radial;
        cameras.back().radial[1] = 0;
        image_filenames.push_back(filename);
        // also add camera pose to poses point cloud:
        poses.push_back(PointXYZRGB(255,255,0));
        poses.back().x = cx;
        poses.back().y = cy;
        poses.back().z = cz;
        break;
      case 2: // header
        if (integerLine(line, 1)) {
          sline >> point_count;
          state = 3;
        }
        break;
      case 3: // points
        if (point_count-- == 0) {
          state = 4;
          break;
        }
        
        // add new point to point cloud
        float x,y,z;
        int r,g,b;
        sline >> x >> y >> z;
        sline >> r >> g >> b;
        points.push_back(PointXYZRGB(r,g,b));
        points.back().x = x;
        points.back().y = y;
        points.back().z = z;

        // add visibility to visible
        visible.push_back(map<int,visibility>());
        int vis_count, frame, index;
        sline >> vis_count;
        while (vis_count--) {
          sline >> frame >> index >> x >> y;
          visible.back().insert(pair<int,visibility>(frame,visibility()));
          visible.back().find(frame)->second.index = index;
          visible.back().find(frame)->second.x = x;
          visible.back().find(frame)->second.y = y;
        }
        
        break;
      case 4: // header
        break;
    }
  }
  file.close();

  cout << poses.size() << " poses read!" << endl;
  cout << points.size() << " points read!" << endl;

  return true;
}

bool SfMReader::readOUT()
{
  /* output from: Bundler
   * contains: cameras, points, visibility
   * format:
   *  http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html#S6
   *  0: <header>          (2 lines)
   *  1: <cameras>         (5 lines each)
   *  2: <points pos+col>  (3 lines each)  + <visibility ijxy>
   */

  ifstream file(path.c_str());
  if (!file.is_open())
    return false;

  int camera_count, point_count;

  int state = 0; // for states see comment above
  string line;
  while (file.good()) {
    getline(file, line);
    stringstream sline(line);
    string filename;
    switch (state) {
      case 0: // header
        if (integerLine(line, 2)) {
          sline >> camera_count >> point_count;
          state = 1;
        }
        break;
      case 1: // cameras
        if (camera_count-- == 0) {
          state = 2;
          sline.flush();
          sline << line;
          // no break! use this line
        } else {
          // new camera
          double cx, cy, cz;  // pos: position
          double focal, r00,r01,r02,r10,r11,r12,r20,r21,r22;
          double radial1, radial2;
          sline >> focal >> radial1 >> radial2;
          getline(file, line);
          sline.str(line);
          sline >> r00 >> r10 >> r20;
          getline(file, line);
          sline.str(line);
          sline >> r01 >> r11 >> r21;
          getline(file, line);
          sline.str(line);
          sline >> r02 >> r12 >> r22;
          getline(file, line);
          sline.str(line);
          sline >> cx >> cy >> cz;

          // add new camera to list of cameras
          cameras.push_back(camera());
          cameras.back().t = Mat(3, 1, CV_64FC1);
          cameras.back().t.at<double>(0) = cx;
          cameras.back().t.at<double>(1) = cy;
          cameras.back().t.at<double>(2) = cz;
          cameras.back().R = Mat(3, 3, CV_64FC1);
          cameras.back().R.at<double>(0,0) = r00;
          cameras.back().R.at<double>(0,1) = r01;
          cameras.back().R.at<double>(0,2) = r02;
          cameras.back().R.at<double>(1,0) = r10;
          cameras.back().R.at<double>(1,1) = r11;
          cameras.back().R.at<double>(1,2) = r12;
          cameras.back().R.at<double>(2,0) = r20;
          cameras.back().R.at<double>(2,1) = r21;
          cameras.back().R.at<double>(2,2) = r22;
          cameras.back().focal = focal;
          cameras.back().radial[0] = radial1;
          cameras.back().radial[1] = radial2;
          // also add camera pose to poses point cloud:
          poses.push_back(PointXYZRGB(255,255,0));
          poses.back().x = cx;
          poses.back().y = cy;
          poses.back().z = cz;
          break;
        }
      case 2: // points
        if (point_count-- == 0) {
          state = 3;
          break;
        }
        
        // add new point to point cloud
        float x,y,z;
        int r,g,b;
        sline >> x >> y >> z;
        getline(file, line);
        sline.str(line);
        sline >> r >> g >> b;

        points.push_back(PointXYZRGB(r,g,b));
        points.back().x = x;
        points.back().y = y;
        points.back().z = z;

        // add visibility to visible
        visible.push_back(map<int,visibility>());
        int vis_count, frame, index;
        getline(file, line);
        sline.str(line);
        sline >> vis_count;
        while (vis_count-- > 0) {
          sline >> frame >> index >> x >> y;
          visible.back().insert(pair<int,visibility>(frame,visibility()));
          visible.back().find(frame)->second.index = index;
          visible.back().find(frame)->second.x = x;
          visible.back().find(frame)->second.y = y;
        }
        
        break;
      case 3: // end
        break;
    }
  }
  file.close();

  cout << poses.size() << " poses read!" << endl;
  cout << points.size() << " points read!" << endl;

  return true;
}

bool SfMReader::readPLY()
{
  /* output from: VisualSFM, Bundler
   * contains: 
   * format:
   *  http://local.wasp.uwa.edu.au/~pbourke/dataformats/ply/
   *  0: <header>          (10+ lines)     list of columns of table
   *  1: <points pos+col>  (1 line each)   NO visibility (no dynamic list length)
   *                                       and NO camera poses
   */

  // pcl::PLYReader?
  // http://docs.pointclouds.org/trunk/classpcl_1_1_p_l_y_reader.html

  ifstream file(path.c_str());
  if (!file.is_open())
    return false;

  int state = 0; // for states see comment above
  string line;
  while (file.good()) {
    getline(file, line);
    stringstream sline(line);
    string filename;
    switch (state) {
      case 0: // header
        if (line.find("end_header") == 0) {
          state = 1;
        }
        break;
      case 1: // points
        if (line == "") {
          state = 2;
          break;
        }
        
        // add new point to point cloud
        float x,y,z;
        int r,g,b;
        // ! assuming X Y Z R G B [foo]
        sline >> x >> y >> z >> r >> g >> b;
        points.push_back(PointXYZRGB(r,g,b));
        points.back().x = x;
        points.back().y = y;
        points.back().z = z;
        break;
      case 2: // end
        break;
    }
  }
  file.close();

  cout << points.size() << " points read!" << endl;

  return true;
  
}

bool SfMReader::readTXT()
{
  /* output from: voodoo (probably)
   * contains:
   * format:
   *  (documented in the files itself)
   *  0: <header>          (39+ lines)    documentation of file format
   *  1: <cameras>         (2 lines each)
   *  2: <header>          (3 lines)
   *  3: <points pos>      (1 line each)   NO visibility, NO colour
   *  4: <empty lines>
   */

  ifstream file(path.c_str());
  if (!file.is_open())
    return false;

  int state = 0; // for states see comment above
  string line;
  while (file.good()) {
    getline(file, line);
    stringstream sline(line);
    string filename;
    switch (state) {
      case 0: // header
        if (line == "#timeindex 1") {
          state = 1;
        }
        break;
      case 1: // cameras
        if (line == "#") {
          state = 2;
          break;
        } else if (line.at(0) == '#') {
          break;
        }
        // new camera
        double cx, cy, cz;  // pos: position
        sline >> cx >> cy >> cz;
        // TODO: convert and use other variables

        // add new camera to list of cameras
        /*
        cameras.push_back(visualization::Camera());
        cameras.back().pos[0] = cx;
        cameras.back().pos[1] = cy;
        cameras.back().pos[2] = cz;
        */
        // also add camera pose to poses point cloud:
        poses.push_back(PointXYZRGB(255,255,0));
        poses.back().x = cx;
        poses.back().y = cy;
        poses.back().z = cz;
        break;
      case 2: // header
        if (line.at(0) != '#') {
          state = 3;
        }
        break;
      case 3: // points
        if (line == "") {
          state = 4;
          break;
        }
        
        // add new point to point cloud
        float x,y,z;
        sline >> x >> y >> z;
        points.push_back(PointXYZRGB(150,150,150));
        points.back().x = x;
        points.back().y = y;
        points.back().z = z;
        break;
      case 4: // header
        break;
    }
  }
  file.close();

  cout << poses.size() << " poses read!" << endl;
  cout << points.size() << " points read!" << endl;

  return true;
}

bool SfMReader::readPCD()
{
  /* output from: [PCL programs]
   * contains:
   * format:
   *  http://pointclouds.org/documentation/tutorials/pcd_file_format.php
   */

  // TODO: pcl::PCDReader?

  cout << "TODO: readPCD" << endl;
  return false;
}

/* this getter is used to prevent SfMReader from using
 * the slow SequenceCapture for calculating the window size
 * only, unless strictly necessary
 */
void SfMReader::getWindowSize(Size& size)
{
  if (this->window_size.width == 0) {
    // determine window size
    if (imagespath == "") {
      cerr << "[!!] Error: no image path specified for SfMReader::getWindowSize" << endl;
      return;
    }
    Mat frame;
    SequenceCapture sq(imagespath);
    sq.read(frame);
    this->window_size = frame.size();
  }
  // set window size
  size = this->window_size;
}

bool SfMReader::integerLine(string line, int count)
{
  int number = 0;
  int state = 1;
  for (string::iterator c = line.begin(); c != line.end(); c++) {
    switch (state) {
      case 1: // spaces
        if (isspace(*c)) {
          state = 1;
        } else if (isdigit(*c)) {
          state = 2;
          number++;
        } else {
          return false;
        }
        break;
      case 2: // digit
        if (isspace(*c)) {
          state = 1;
        } else if (isdigit(*c)) {
          state = 2;
        } else {
          return false;
        }
        break;
    }
  }

  if (number == count)
    return true;
  else
    return false;
}


bool SfMReader::selectPointsForCamera(int id,
                                      Scalar colourCamera,
                                      Scalar colourSelectedCamera,
                                      Scalar colourSelectedPoint)
{
  if (poses.size() < id)
    return false;

  // colour and save points visible from given camera
  resetPointColours();
  line_ends.clear();
  curr_visible_keypoints.clear();
  points_curr_visibility.clear();
  for (int i=0; i<visible.size(); i++) {
    points_curr_visibility.push_back( visible.at(i).find(id) != visible.at(i).end() );
    if (points_curr_visibility.back()) {
      points.at(i).r = colourSelectedPoint[2];
      points.at(i).g = colourSelectedPoint[1];
      points.at(i).b = colourSelectedPoint[0];
      line_ends.push_back(&points.at(i));
      curr_visible_keypoints.push_back(&visible.at(i).find(id)->second);
    }
  }

  // colour all camera poses
  for (int i=0; i<poses.size(); i++) {
    poses.at(i).r = colourCamera[2];
    poses.at(i).g = colourCamera[1];
    poses.at(i).b = colourCamera[0];
  }

  // recolour given camera
  poses.at(id).r = colourSelectedCamera[2];
  poses.at(id).g = colourSelectedCamera[1];
  poses.at(id).b = colourSelectedCamera[0];
  // save pointer to given camera
  line_start = &poses.at(id);

  return true;
}


bool SfMReader::selectCamerasForPoint(int id,
                                      Scalar colourCamera,
                                      Scalar colourSelectedCamera,
                                      Scalar colourSelectedPoint)
{
  if (points.size() < id)
    return false;

  // colour given point
  resetPointColours();
  points.at(id).r = colourSelectedPoint[2];
  points.at(id).g = colourSelectedPoint[1];
  points.at(id).b = colourSelectedPoint[0];
  // save pointer to given point
  line_start = &points.at(id);

  // colour all camera poses
  for (int i=0; i<poses.size(); i++) {
    poses.at(i).r = colourCamera[2];
    poses.at(i).g = colourCamera[1];
    poses.at(i).b = colourCamera[0];
  }

  // recolour and save poses for given point
  line_ends.clear();
  curr_visible_keypoints.clear();
  points_curr_visibility.clear();
  map<int,visibility>* vismap = &visible.at(id);
  map<int,visibility>::iterator it;
  int frame;
  for (it = vismap->begin(); it!=vismap->end(); it++) {
    frame = (*it).first;
    poses.at(frame).r = colourSelectedCamera[2];
    poses.at(frame).g = colourSelectedCamera[1];
    poses.at(frame).b = colourSelectedCamera[0];
    line_ends.push_back(&poses.at(frame));
  }

  return true;
}


void SfMReader::resetPointColours()
{
  if (points_original.size() == 0) {
    // first time call: backup point cloud
    for (int i=0; i<points.size(); i++)
      points_original.push_back(points.at(i));
  } else {
    // reset points, get from point cloud backup
    // (this is, indeed, a lot of work)
    for (int i=0; i<points.size(); i++) {
      points.at(i).r = points_original.at(i).r;
      points.at(i).g = points_original.at(i).g;
      points.at(i).b = points_original.at(i).b;
    }
  }
}


void SfMReader::getExtrema(Scalar& min, Scalar& max)
{
  PointXYZRGB min_pt, max_pt;
  getMinMax3D(points, min_pt, max_pt);
  min[0] = min_pt.x;
  min[1] = min_pt.y;
  min[2] = min_pt.z;
  max[0] = max_pt.x;
  max[1] = max_pt.y;
  max[2] = max_pt.z;
}

/* reprojects point using camera parameters
 * note: this will return the undistorted location
 * (re-distort before displaying in an image)
 */
void SfMReader::reproject(PointXYZRGB* point, camera* cam, PointXYZRGB* projected)
{
  Mat K = Mat::zeros(3, 3, CV_64FC1);
  K.at<double>(0,0) = cam->focal;
  K.at<double>(1,1) = cam->focal;
  K.at<double>(2,2) = 1;
  Mat v(3, 1, CV_64FC1);
  Mat x(3, 1, CV_64FC1);
  x.at<double>(0) = point->x;
  x.at<double>(1) = point->y;
  x.at<double>(2) = point->z;
  v = K * cam->R * (x + cam->t);
  projected->x = v.at<double>(0)/v.at<double>(2);
  projected->y = v.at<double>(1)/v.at<double>(2);
  projected->z = 0;
  projected->r = point->r;
  projected->g = point->g;
  projected->b = point->b;
}

bool SfMReader::reprojectsInsideImage(int pointID, int camID, Size size)
{
  if (pointID<0 || pointID>=points.size()
    || camID<0 || camID>=cameras.size()) {
    cerr << "[!!] Error: invalid pointID or camID (in SfMReader::reprojectsInsideImage)" << endl;
    return false;
  }

  // TODO: would be nicer to find size ourselves

  return reprojectsInsideImage(&points.at(pointID),
                               &cameras.at(camID),
                               size);
}

bool SfMReader::reprojectsInsideImage(PointXYZRGB* point, camera* cam, Size size)
{
  PointXYZRGB projected;
  reproject(point, cam, &projected);
  return (projected.x >= size.width/-2.0
       && projected.x <= size.width/2.0
       && projected.y >= size.height/-2.0
       && projected.y <= size.height/2.0);
}


void SfMReader::quaternion2matrix(Mat& q, Mat& R)
{
  double a = q.at<double>(0);
  double b = q.at<double>(1);
  double c = q.at<double>(2);
  double d = q.at<double>(3);
  // conversion based on the description on
  // http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Conversion_to_and_from_the_matrix_representation
  R.at<double>(0,0) = a*a + b*b - c*c - d*d;
  R.at<double>(1,0) = 2*b*c + 2*a*d;
  R.at<double>(2,0) = 2*b*d - 2*a*c;
  R.at<double>(0,1) = 2*b*c - 2*a*d;
  R.at<double>(1,1) = a*a - b*b + c*c - d*d;
  R.at<double>(2,1) = 2*c*d + 2*a*b;
  R.at<double>(0,2) = 2*b*d + 2*a*c;
  R.at<double>(1,2) = 2*c*d - 2*a*b;
  R.at<double>(2,2) = a*a - b*b - c*c + d*d;
}
