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
//#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>


using namespace std;
using namespace pcl;
using namespace cv;

typedef struct visibility
{
  int index;
  double x;
  double y;
} visibility;

class SfMReader
{

  private:
    string path;

    bool readNVM();
    bool readOUT();
    bool readPLY();
    bool readTXT();
    bool readPCD();
    bool integerLine(string line, int count=1);

  public:
    PointCloud<PointXYZRGB> points, points_original;
    PointCloud<PointXYZRGB> poses;
    vector<visualization::Camera> cameras;
    vector<map<int,visibility> > visible; 
    PointXYZRGB* line_start;
    vector<PointXYZRGB*> line_ends;
    //vector<io::ply::camera>

    SfMReader();
    SfMReader(string path);
    ~SfMReader();
    bool read();
    bool read(string path);
    bool selectPointsForCamera(int id,
                               Scalar colourCamera = Scalar(0,255,255),
                               Scalar colourSelectedCamera = Scalar(0,0,255),
                               Scalar colourSelectedPoint = Scalar(0,0,255));
    bool selectCamerasForPoint(int id,
                               Scalar colourCamera = Scalar(0,255,255),
                               Scalar colourSelectedCamera = Scalar(0,0,255),
                               Scalar colourSelectedPoint = Scalar(0,0,255));

    void resetPointColours();

};

#endif
