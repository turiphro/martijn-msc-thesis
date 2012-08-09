/* wrapper around cv::VideoCapture to add directories with images */

#ifndef SEQUENCECAPTURE_H
#define SEQUENCECAPTURE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <boost/algorithm/string.hpp>    


using namespace std;
using namespace cv;


class SequenceCapture
{

  private:
    VideoCapture* videoCapture;
    int position;
    enum {CAMERA, VIDEO, IMAGES} type;
    
  public:
    string path;
    vector<string> images; // for directory with images only

    SequenceCapture(int device);
    SequenceCapture(string path);
    ~SequenceCapture();
    bool isOpened();
    bool isBounded();
    bool read(Mat& image);
    SequenceCapture& operator>>(Mat& image);
    bool setPosition(int position);
    bool setPosition(string filename); // for directory with images only
    int  getPosition();
    int  getFrameCount();
    
};


#endif
