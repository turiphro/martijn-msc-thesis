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

SfMReader::SfMReader(string path)
{
  this->path = path;
  updated = false;
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
  string ext = path.substr(path.find_last_of(".") + 1);

  if (ext == "nvm")
    return readNVM(); // VisualSFM 
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
        double focal_l, q1,q2,q3,q4, radial_dist;
        sline >> filename >> focal_l >> q1>>q2>>q3>>q4 >> cx >> cy >> cz >> radial_dist;
        // TODO: convert quaternion to focal look-at
        //       solve later on (may want to use io::ply::camera;
        //       some compile errors, library broken)
        // visualization::Camera can easily be used to set the visualizer view

        // add new camera to list of cameras
        cameras.push_back(visualization::Camera());
        cameras.back().pos[0] = cx;
        cameras.back().pos[1] = cy;
        cameras.back().pos[2] = cz;
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
   *  <header>          (2 lines)
   *  <cameras>         (5 lines each)
   *  <points pos+col>  (3 lines each)  + <visibility ijxy>
   */

}

bool SfMReader::readPLY()
{
  /* output from: VisualSFM, Bundler
   * contains: 
   * format:
   *  http://local.wasp.uwa.edu.au/~pbourke/dataformats/ply/
   *  <header>          (10+ lines)     list of columns of table
   *  <points pos+col>  (1 line each)   NO visibility (no dynamic list length)
   */

  // pcl::PLYReader?
  // http://docs.pointclouds.org/trunk/classpcl_1_1_p_l_y_reader.html
  
}

bool SfMReader::readTXT()
{
  /* output from: voodoo (hopefully)
   * contains:
   * format:
   *  (documented in the files itself)
   *  <header>          (39+ lines)     documentation of file format
   *  <cameras>         (2 lines each)
   *  <header>          (3 lines)
   *  <points pos>      (1 line each)   NO visibility
   */

  cout << "TODO: readTXT" << endl;
  return false;
}

bool SfMReader::readPCD()
{
  /* output from: [PCL programs]
   * contains:
   * format:
   *  http://pointclouds.org/documentation/tutorials/pcd_file_format.php
   */

  // pcl::PCDReader?

  cout << "TODO: readPCD" << endl;
  return false;
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



