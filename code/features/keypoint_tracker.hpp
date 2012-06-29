/* keypoint tracking class - saves pointers to keypoints possibly representing the same point */

#ifndef KEYPOINTTRACKER_H
#define KEYPOINTTRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>
#include <vector>

#include "../io/sequence_capture.hpp"
#include "feature_sequence.hpp"

using namespace std;
using namespace cv;

class KeypointTracker
{
  
  private:
    //enum type {NN};
    vector<vector<KeyPoint> *>* kpp;
    vector<Mat *>* kdp;
    int kp_ref, kp_lastseen, kp_firstseen;
    int frame_ref, frame_lastseen, frame_firstseen;
    int frame_lasttested, frame_firsttested;
    int* track;
    double* distinctiveness;

  public:
    KeypointTracker(vector<vector<KeyPoint> *>* kpp, vector<Mat *>* kdp, int kp_ref, int frame_ref=0, int type=0);
    ~KeypointTracker();
    int bestMatch(int position1, int position2, double maxRatio=0.85);
    void trackAll();
    void trackUntil(int position);
    int getBestMatch(int position);
    void plotDistinctiveness(Mat& output, int frame_curr=-1);
    void plot(Mat& output, double *data, int count, int frame_curr=-1, Scalar colour=Scalar(0,0,255), bool reDraw=true, Size size=Size(300,80));

};

#endif
