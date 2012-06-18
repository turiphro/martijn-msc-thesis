/* feature extractor for whole sequence */

#ifndef FEATURESEQUENCE_H
#define FEATURESEQUENCE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
    string edgeDetector;
    // TODO: graph of interest points and edges

  public:
    vector<vector<KeyPoint> *> kp;
    vector<Mat *> kd;
    vector<Mat *> edges;

    FeatureSequence(SequenceCapture* sc,
                    // SIFT, SURF, ORB, FAST, STAR, MSER,
                    // GFTT, HARRIS, Dense, SimpleBlob
                    const string& pointDetector="SIFT",
                    // SIFT, SURF, ORB, BRIEF
                    const string& pointDescriptor="SIFT",
                    // CANNY, HARRIS
                    const string& edgeDetector="CANNY");
    ~FeatureSequence();
    void clearCache();
    int getNearestKeypoint(Point2f point, int position);
    bool calculateFeaturesNextFrame();
    bool calculateFeaturesOf(int frame);
    bool calculateFeatures();

    bool getFeaturesOf(int frame,
                       vector<KeyPoint>& keypoints,
                       Mat& descriptors,
                       Mat& edges);

    bool visualiseFeaturesOf(int position,
                             Mat& output,
                             bool richDraw=false,
                             Scalar colourPoints=Scalar(0,255,255),
                             Scalar colourEdges=Scalar(0,0,255));

    bool visualiseFeatures(Mat& frame,
                           Mat& output, 
                           vector<KeyPoint>* keypoints,
                           Mat& frame_edges,
                           bool richDraw=false,
                           Scalar colourPoints=Scalar(0,255,255),
                           Scalar colourEdges=Scalar(0,0,255));

    bool visualiseMatchesOf(int position1,
                            int position2,
                            Mat& output,
                            bool flowDraw=true, // false: two images draw
                            bool richDraw=false,
                            bool redrawImg=true,
                            Scalar colourMatches=Scalar(0,255,255),
                            Scalar colourSingles=Scalar(150,0,150));

    void drawFlowVectors(Mat& output,
                         vector<KeyPoint>* kp1,
                         vector<KeyPoint>* kp2,
                         vector<DMatch>* matches,
                         int minDistance=40,
                         Scalar colourMatches=Scalar(0,255,255),
                         Scalar colourSingles=Scalar(0,255,255),
                         bool redrawKeypoints=true,
                         bool richDraw=false);


    // TODO: extend with tracking / visibilityTraces (linking over sequence)
    // bool getFeatureSequence(feature, sequence)

};

#endif
