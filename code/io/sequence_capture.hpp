/* wrapper around cv::VideoCapture to add directories with images */

#ifndef SEQUENCECAPTURE_H
#define SEQUENCECAPTURE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>

using namespace std;
using namespace cv;


class SequenceCapture
{

  private:
    VideoCapture* videoCapture;
    string path;
    vector<string> images;
    int position;
    enum {CAMERA, VIDEO, IMAGES} type;
    
  public:

    SequenceCapture(int device);
    SequenceCapture(string path);
    ~SequenceCapture();
    bool isOpened();
    bool isBounded();
    bool read(Mat& image);
    SequenceCapture& operator>>(Mat& image);
    bool setPosition(int position);
    int  getPosition();
    int  getFrameCount();
    
};


#endif
