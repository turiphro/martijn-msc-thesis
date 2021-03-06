#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../io/sfm_reader.hpp"
#include "../io/sequence_capture.hpp"

using namespace std;
using namespace pcl;
using namespace cv;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <sfm> [<images>]" << endl;
  cout << " where sfm is the path to an nvm, ply, out or txt file," << endl;
  cout << " and images is the path to the corresponding directory with images." << endl;
  exit(1);
}

typedef struct {
  SfMReader* sfm;
  visualization::PCLVisualizer* viewer;
  SequenceCapture* sc;
  bool updated;
  int camID;
  int pointID;
  bool lineDraw;
  bool cameraWindow;
} callbackObject;


/* drawing functions */

void drawLines(visualization::PCLVisualizer& viewer, SfMReader* sfm, int maxLines = 250)
{
  // faster drawing: http://stackoverflow.com/questions/2140796/draw-a-multiple-lines-set-with-vtk

  viewer.removeAllShapes();
  stringstream name("");
  if (sfm->line_ends.size() > maxLines) {
    cout << "Too many lines (" << sfm->line_ends.size() << ") to draw! ";
    cout << "Only drawing first " << maxLines << " lines." << endl;
  }
  for (int i=0; i<min((int)sfm->line_ends.size(), maxLines); i++) {
    name.str("line");
    name << i;
    viewer.addLine(*sfm->line_start, *sfm->line_ends.at(i), 0,1,0, name.str());
  }
}

void showCameraView(callbackObject* options, string window_name)
{
  if (options->cameraWindow) {
    // camera window enabled
    if (options->camID >= 0) {
      // camera selected
      Mat frame;

      // get camera image
      if (options->sfm->ext == "nvm") {
        /* VisualSfM (nvm) does not save the cameras in the order
         * in which the images appear in a sorted directory listing.
         * Therefore we need to do some tricks to get the right
         * image. I am very sorry for the messy code this requires
         * here and in the used classes.
         */
        string img_filename = options->sfm->image_filenames.at(options->camID);
        options->sc->setPosition(img_filename);
      } else {
        // all other formats are fine
        options->sc->setPosition(options->camID);
      }
      options->sc->read(frame);

      // draw keypoints
      Size s = frame.size();
      double mx, my, r, focal;
      for (vector<visibility*>::iterator it = options->sfm->curr_visible_keypoints.begin();
           it != options->sfm->curr_visible_keypoints.end(); it++) {
        mx = (*it)->x;
        my = (*it)->y;
        /* Optionally, one could undistort the the measurements 
         * (instead of redistorting the reprojected points) with:
         * focal = options->sfm->cameras.at(options->camID).focal;
         * r = (mx*mx + my*my) / (focal*focal) * options->sfm->cameras.at(options->camID).radial[0];
         * mx = (1 + r) * (*it)->x;
         * my = (1 + r) * (*it)->y;
         */
        mx += s.width/2.0;
        my += s.height/2.0;
        circle(frame, Point(mx, my), 2, Scalar(255,100,0), 2);
      }

      // draw reprojected keypoints
      PointXYZRGB projected;
      int visible_in = 0, visible_out = 0;
      int invisible_in = 0, invisible_out = 0;
      double x, y, gamma;
      for (int i=0; i<options->sfm->points.size(); i++) {
        options->sfm->reproject(&options->sfm->points.at(i),
                                &options->sfm->cameras.at(options->camID),
                                &projected);
        // redo distortion for reprojected keypoints
        gamma = options->sfm->cameras.at(options->camID).focal;
        options->sfm->distortPointR1(&projected, &options->sfm->cameras.at(options->camID));
        x = projected.x + s.width/2.0;
        y = projected.y + s.height/2.0;

        if (x > 0 && x < s.width && y > 0 && y < s.height) {
          if (options->sfm->points_curr_visible.at(i)) {
            circle(frame, Point(x, y), 3, Scalar(0,255,0), 2);
            visible_in++;
          } else {
            circle(frame, Point(x, y), 3, Scalar(0,0,255), 1);
            invisible_in++;
          }
        } else {
          if (options->sfm->points_curr_visible.at(i)) {
            visible_out++;
          } else {
            invisible_out++;
          }
        }
      }
      cout << "Showing " << visible_in << "/" << (visible_in + visible_out);
      cout << " (" << (100.0*visible_in/(visible_in + visible_out)) << "%)";
      cout << " visible and " << invisible_in << "/" << (invisible_in + invisible_out);
      cout << " (" << (100.0*invisible_in/(invisible_in + invisible_out)) << "%)";
      cout << " invisible points." << endl;

      // show result
      imshow(window_name, frame);
    }
  }
}


/* callback functions */

void pp_callback(const visualization::PointPickingEvent& event, void* options_ptr)
{
  callbackObject* options = (callbackObject*) options_ptr;

  int id = event.getPointIndex();
  if (id == -1)
    return;

  float x, y, z;
  event.getPoint (x, y, z);

  if (options->sfm->poses.size() > id &&
      options->sfm->poses.at(id).x == x &&
      options->sfm->poses.at(id).y == y &&
      options->sfm->poses.at(id).z == z) {
    // update camera view
    cout << ">> Clicked on camera at (" << x << ", " << y << ", " << z << ")!" << endl;
    options->sfm->selectPointsForCamera(id);
    options->camID = id;
    options->pointID = -1;
    showCameraView(options, "camera view");
  } else {
    cout << ">> Clicked on point at (" << x << ", " << y << ", " << z << ")!" << endl;
    options->sfm->selectCamerasForPoint(id);
    options->camID = -1;
    options->pointID = id;
  }

  options->updated = true;
}

void kb_callback(const visualization::KeyboardEvent& event, void* options_ptr)
{
  callbackObject* options = (callbackObject*) options_ptr;

  if (!event.keyDown())
    return;

  if (event.getKeyCode() == '?') {
    cout << "Keyboard shortcuts: (press h for PCL shortcuts)" << endl;
    cout << " b/w:   set background colour to black/white" << endl;
    cout << " d:     line draw on/off" << endl;
    cout << " n/p:   select next/prev camera or point" << endl;
    cout << " f/l:   select first/last camera or point" << endl;
    cout << " i:     camera window enable/disable (shows image for current camera)" << endl;
  } else if (event.getKeyCode() == 'q') {
    exit(0);
  /** visuals **/
  } else if (event.getKeyCode() == 'b') {
    options->viewer->setBackgroundColor(0,0,0);
  } else if (event.getKeyCode() == 'w') {
    options->viewer->setBackgroundColor(1,1,1);
  } else if (event.getKeyCode() == 'd') {
    options->lineDraw = !options->lineDraw;
    options->updated = true;
  } else if (event.getKeyCode() == 'n') { // select next camera/point
    if (options->camID >= 0) {
      // select next camera
      options->camID = min(options->camID + 1,
                           (int)options->sfm->poses.size() - 1);
      options->sfm->selectPointsForCamera(options->camID);
      showCameraView(options, "camera view");
    } else if (options->pointID >= 0) {
      // select next point
      options->pointID = min(options->pointID + 1,
                             (int)options->sfm->points.size() - 1);
      options->sfm->selectCamerasForPoint(options->pointID);
    }
    options->updated = true;
  } else if (event.getKeyCode() == 'p') { // select prev camera/point
    if (options->camID >= 0) {
      // select prev camera
      options->camID = max(options->camID - 1, 0);
      options->sfm->selectPointsForCamera(options->camID);
      showCameraView(options, "camera view");
    } else if (options->pointID >= 0) {
      // select prev point
      options->pointID = max(options->pointID - 1, 0);
      options->sfm->selectCamerasForPoint(options->pointID);
    }
    options->updated = true;
  } else if (event.getKeyCode() == 'f') { // select first camera/point (begin)
    if (options->camID >= 0) {
      // select first camera
      options->camID = 0;
      options->sfm->selectPointsForCamera(options->camID);
      showCameraView(options, "camera view");
    } else if (options->pointID >= 0) {
      // select first point
      options->pointID = 0;
      options->sfm->selectCamerasForPoint(options->pointID);
    }
    options->updated = true;
  } else if (event.getKeyCode() == 'l') { // select last camera/point (end)
    if (options->camID >= 0) {
      // select last camera
      options->camID = options->sfm->poses.size() - 1;
      options->sfm->selectPointsForCamera(options->camID);
      showCameraView(options, "camera view");
    } else if (options->pointID >= 0) {
      // select last point
      options->pointID = options->sfm->points.size() - 1;
      options->sfm->selectCamerasForPoint(options->pointID);
    }
    options->updated = true;
  /** extra windows **/
  } else if (event.getKeyCode() == 'i') { // camera window
    options->cameraWindow = !options->cameraWindow;
    if (options->sc == NULL) {
      cout << "[!!] Warning: no image path specified" << endl;
      options->cameraWindow = false;
    } else if (options->sfm->poses.size() == 0) {
      cout << "[!!] Warning: no camera poses known (not in file)" << endl;
    } else if (options->cameraWindow) {
      namedWindow("camera view", CV_WINDOW_NORMAL);
      showCameraView(options, "camera view");
    } else {
      destroyWindow("camera view");
    }
  }


}




/* main */
int main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  SfMReader* sfm;
  if (argc==2)
    sfm = new SfMReader(argv[1]);
  else
    sfm = new SfMReader(argv[1], argv[2]);
  visualization::PCLVisualizer viewer("Cloud viewer");

  // set options
  callbackObject options;
  options.sfm = sfm;
  options.viewer = &viewer;
  options.updated = false;
  options.camID = -1;
  options.pointID = -1;
  options.lineDraw = false;
  options.cameraWindow = false;
  options.sc = (argc>=3) ? new SequenceCapture(argv[2]) : NULL;

  PointCloud<PointXYZRGB>::Ptr points(&sfm->points);
  PointCloud<PointXYZRGB>::Ptr poses(&sfm->poses);

  viewer.registerPointPickingCallback(pp_callback, &options);
  viewer.registerKeyboardCallback(kb_callback, &options);

  viewer.addPointCloud(points, "points");
  viewer.addPointCloud(poses, "poses");
  viewer.setBackgroundColor(0,0,0);
  viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "points");
  viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "poses");
  //viewer.addCoordinateSystem (1.0);  // draw origin
  viewer.initCameraParameters ();

  int msg_nr = 1;
  if (sfm->visible.size() == 0)
    viewer.addText("No visibility info", 10, 10 + (10 * msg_nr++), 0.8,0,1);
  if (sfm->poses.size() == 0)
    viewer.addText("No camera poses", 10, 10 + (10 * msg_nr++), 1,1,0);


  int opencvKey;
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);   // needed for pcl visualizer to work
    opencvKey = waitKey(1); // needed for highgui windows to work
    if (opencvKey != -1)
      // pass opencv keyboard events to pcl keyboard handler
      // (works for user defined shortcuts only)
      kb_callback(visualization::KeyboardEvent(true, "", opencvKey,
                                               false, false, false), &options);

    if (options.updated) {
      viewer.updatePointCloud(points, "points");
      viewer.updatePointCloud(poses, "poses"); 
      if (options.lineDraw)
        drawLines(viewer, sfm, 500);
      else
        viewer.removeAllShapes();
      options.updated = false;
    }
  }

  viewer.close();

  // more drawing: use viewer.getRenderWindow() and use VTK

  return 0;
}


