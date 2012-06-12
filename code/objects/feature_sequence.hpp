/* feature extractor for whole sequence */

#ifndef FEATURESEQUENCE_H
#define FEATURESEQUENCE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "../io/sequence_capture.hpp"

using namespace std;
using namespace cv;

class FeatureSequence
{

  private:
    SequenceCapture* sc;
    Ptr<FeatureDetector> fd;
    Ptr<DescriptorExtractor> de;
    vector<vector<KeyPoint> *> kp;
    vector<Mat *> kd;

  public:
    FeatureSequence(SequenceCapture* sc,
                    const string& pointDetector="FAST",
                    const string& edgeDetector="CANNY");
    ~FeatureSequence();
    bool calculateFeaturesNextFrame();
    bool calculateFeaturesOf(int frame);
    bool calculateFeatures();
    bool getFeaturesOf(int frame,
                       vector<KeyPoint>& keypoints);
                       // TODO: add edges as image and/or list of connected KeyPoints
    bool visualiseFeaturesOf(int frame, Mat& output, Scalar colour=Scalar(255,0,0));

    // TODO: extend with tracking / visibilityTraces (linking over sequence)
    // bool getFeatureSequence(feature, sequence)

};

#endif
