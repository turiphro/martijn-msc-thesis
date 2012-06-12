#include "feature_sequence.hpp"

FeatureSequence::FeatureSequence(SequenceCapture* sc,
                                 const string& pointDetector,
                                 const string& pointDescriptor,
                                 const string& edgeDetector)
{
  this->sc = sc;
  // better parameter control? make less general objects
  fd = FeatureDetector::create(pointDetector);
  de = DescriptorExtractor::create(pointDescriptor);
  this->edgeDetector = edgeDetector;

  int frame_count = sc->getFrameCount();
  if (frame_count==-1)
    frame_count = 1; // infinite stream: 'one frame' only
  while (frame_count--) {
    kp.push_back(new vector<KeyPoint>);
    kd.push_back(new Mat());
    edges.push_back(new Mat());
  }
}

FeatureSequence::~FeatureSequence()
{
  while (kp.size()) {
    delete kp.back();
    kp.pop_back();
    delete kd.back();
    kd.pop_back();
    delete edges.back();
    edges.pop_back();
  }
}

bool FeatureSequence::calculateFeaturesNextFrame()
{
  int position = sc->getPosition();
  Mat frame;
  if (!sc->read(frame))
    return false;

  // interest points
  fd->detect(frame, *kp.at(position));
  SIFT foo; // this prevents segfault somehow (which makes no sense)
  de->compute(frame, *kp.at(position), *kd.at(position));

  // edges
  Mat frame_bw;
  cvtColor(frame, frame_bw, CV_RGB2GRAY);
  if (edgeDetector.compare("HARRIS")==0) {
    cornerHarris(frame_bw, *edges.at(position), 2, 3, 0.04);
    // TODO: fix type of frame_bw
  } else {
    if (edgeDetector.compare("CANNY")!=0)
      cout << "[!!] Using CANNY instead of " << edgeDetector << endl;
    GaussianBlur(frame_bw, frame_bw, Size(0,0), 2);
    Canny(frame_bw, *edges.at(position), 50, 100);
  }

  // TODO: make point/edge graph

  return true;
}

bool FeatureSequence::calculateFeaturesOf(int frame)
{
  sc->setPosition(frame);
  return calculateFeaturesNextFrame();
}

// calculate all features for all frames
bool FeatureSequence::calculateFeatures()
{
  // this makes no sense for camera streams
  if (!sc->isOpened() || !sc->isBounded())
    return false;

  sc->setPosition(0);
  while (calculateFeaturesNextFrame())
    ;
  return true;
}

bool FeatureSequence::getFeaturesOf(int frame,
                                    vector<KeyPoint>& keypoints,
                                    Mat& edges)
{
  keypoints = *kp.at(frame);
  edges = *this->edges.at(frame);
}

bool FeatureSequence::visualiseFeaturesOf(int position, Mat& output, Scalar colourPoints, Scalar colourEdges)
{
  sc->setPosition(position);
  Mat frame;
  if (!sc->read(frame))
    return false;

  // overlay edges on image
  Mat frame_bw, frame_zero;
  frame_bw = *edges.at(position);
  //(*edges.at(position)).convertTo(frame_bw, frame.type());
  vector<Mat> channels;
  channels.push_back(frame_bw*colourEdges[0]/255.0);
  channels.push_back(frame_bw*colourEdges[1]/255.0);
  channels.push_back(frame_bw*colourEdges[2]/255.0);
  merge(channels, frame_bw);
  
  double alpha = 0.3, beta = 1-alpha;
  addWeighted(frame, alpha, frame_bw, beta, 0, frame);

  // draw interest points
  drawKeypoints(frame, *kp.at(position), output, colourPoints, DrawMatchesFlags::DEFAULT);
  return true;
}


