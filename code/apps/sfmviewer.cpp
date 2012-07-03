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

typedef struct {
  SfMReader* sfm;
  pcl::visualization::PCLVisualizer* viewer;
  bool updated;
  bool lineDraw;
} callbackObject;


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* options_ptr)
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
    cout << "it's a camera at (" << x << ", " << y << ", " << z << ")!" << endl;
    options->sfm->selectPointsForCamera(id);
  } else {
    cout << "it's a point at (" << x << ", " << y << ", " << z << ")!" << endl;
    options->sfm->selectCamerasForPoint(id);
  }

  options->updated = true;
}

void kb_callback(const pcl::visualization::KeyboardEvent& event, void* options_ptr)
{
  callbackObject* options = (callbackObject*) options_ptr;

  if (!event.keyDown())
    return;

  if (event.getKeyCode() == '?') {
    cout << "Keyboard shortcuts: (press h for PCL shortcuts)" << endl;
    cout << " l:     line draw on/off" << endl;
    cout << " b/w:   set background colour to black/white" << endl;
  } else if (event.getKeyCode() == 'l') {
    options->updated = true;
    options->lineDraw = !options->lineDraw;
  } else if (event.getKeyCode() == 'b') {
    options->viewer->setBackgroundColor(0,0,0);
  } else if (event.getKeyCode() == 'w') {
    options->viewer->setBackgroundColor(1,1,1);
  }


}


/* drawing functions */

void drawLines(pcl::visualization::PCLVisualizer& viewer, SfMReader& sfm, int maxLines = 250)
{
  viewer.removeAllShapes();
  stringstream name("");
  if (sfm.line_ends.size() > maxLines) {
    cout << "Too many lines (" << sfm.line_ends.size() << ") to draw! ";
    cout << "Only drawing first " << maxLines << " lines." << endl;
  }
  for (int i=0; i<min((int)sfm.line_ends.size(), maxLines); i++) {
    name.str("line");
    name << i;
    viewer.addLine(*sfm.line_start, *sfm.line_ends.at(i), 0,1,0, name.str());
  }
}



/* main */
int main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  SfMReader sfm(argv[1]);
  pcl::visualization::PCLVisualizer viewer("Cloud viewer");

  // set options
  callbackObject options;
  options.sfm = &sfm;
  options.viewer = &viewer;
  options.updated = false;
  options.lineDraw = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(&sfm.points);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses(&sfm.poses);

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
  if (sfm.visible.size() == 0)
    viewer.addText("No visibility info", 10, 10 + (10 * msg_nr++), 0.8,0,1);
  if (sfm.poses.size() == 0)
    viewer.addText("No camera poses", 10, 10 + (10 * msg_nr++), 1,1,0);


  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);

    if (options.updated) {
      viewer.updatePointCloud(points, "points");
      viewer.updatePointCloud(poses, "poses"); 
      if (options.lineDraw)
        drawLines(viewer, sfm);
      else
        viewer.removeAllShapes();
      options.updated = false;
    }
  }

  //PCL 1.6+: viewer.close();

  // more drawing: use viewer.getRenderWindow() and use VTK

  return 0;
}


