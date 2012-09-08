/* reader for output of SfM programs; extracts camera poses and points */
/* note: would be nice to split into seperate readers */

#ifndef SFMREADER_H
#define SFMREADER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>

#include "sequence_capture.hpp"

using namespace std;
using namespace pcl;
using namespace cv;

typedef struct visibility
{
  int index;
  double x;
  double y;
} visibility;

typedef struct camera
{
  Mat R;
  Mat t;
  double focal;
  double radial[2];
} camera;

class SfMReader
{

  private:

    bool readNVM();
    bool readOUT();
    bool readPLY();
    bool readTXT();
    bool readPCD();
    bool integerLine(string line, int count=1);

  public:
    string path;
    string imagespath;
    Size window_size;
    string ext; // file type
    SequenceCapture* sc;
    PointCloud<PointXYZRGB> points, points_original;
    PointCloud<PointXYZRGB> poses;
    vector<camera> cameras;
    vector<map<int,visibility> > visible; 
    PointXYZRGB* line_start;
    vector<PointXYZRGB*> line_ends;
    vector<visibility*> curr_visible_keypoints;
    vector<bool> points_curr_visible;
    vector<bool> points_curr_invisible;
    vector<string> image_filenames; // used for nvm only

    /* Possible other camera representations:
     * - visualization::Camera  f, t,    fovy, lookat, up
     *    \_ has nice cvtWindowCoordinates function
     * - io::ply::camera        
     * - CameraParams           f, t, R, ratio, ppx, ppy
     */

    SfMReader();
    SfMReader(string path, string imagespath="");
    ~SfMReader();
    bool read();
    bool read(string path);
    void getWindowSize(Size& size);
    int getImageID(string filename);
    bool selectPointsForCamera(int id,
                               bool calcInvisible=true,
                               Scalar colourCamera = Scalar(150,0,255),
                               Scalar colourSelectedCamera = Scalar(0,255,0),
                               Scalar colourVisiblePoint = Scalar(0,255,0),
                               Scalar colourInvisiblePoint = Scalar(0,0,255));
    bool selectCamerasForPoint(int id,
                               Scalar colourCamera = Scalar(150,0,255),
                               Scalar colourSelectedCamera = Scalar(0,255,0),
                               Scalar colourSelectedPoint = Scalar(0,255,0));

    void resetPointColours();
    void getExtrema(Scalar& min, Scalar& max);
    void reproject(PointXYZRGB* point, camera* cam, PointXYZRGB* projected, bool* front=NULL);
    bool reprojectsInsideImage(int pointID, int camID, Size size, PointXYZRGB* projected=NULL);
    bool reprojectsInsideImage(PointXYZRGB* point, camera* cam, Size size, PointXYZRGB* projected=NULL);
    void distortPointR1(PointXYZRGB* point, camera* cam);
    void quaternion2matrix(Mat& q, Mat& R);

};

#endif
