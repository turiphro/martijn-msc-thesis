#include "feature_sequence.hpp"

FeatureSequence::FeatureSequence(SequenceCapture* sc,
                                 const string& pointDetector,
                                 const string& edgeDetector)
{
  this->sc = sc;
  fd = FeatureDetector::create(pointDetector);
  de = DescriptorExtractor::create(pointDetector);
  int frame_count = sc->getFrameCount();
  if (frame_count==-1)
    frame_count = 1; // infinite stream: 'one frame' only
  while (frame_count--) {
    kp.push_back(new vector<KeyPoint>);
    kd.push_back(new Mat());
  }
}

FeatureSequence::~FeatureSequence()
{
  while (kp.size()) {
    delete kp.back();
    kp.pop_back();
    delete kd.back();
    kd.pop_back();
  }
}

bool FeatureSequence::calculateFeaturesNextFrame()
{
  int position = sc->getPosition();
  Mat frame;
  if (!sc->read(frame))
    return false;

  fd->detect(frame, *kp.at(position));
  /*
  cout << "pointer " << position << ": " << kp.at(position) << endl;
  cout << "size: " << (*kp.at(position)).size() << endl;
  vector<KeyPoint> keypoints = *kp.at(position);
  de->compute(frame, keypoints, descriptors);
  //de->compute(frame, *kp.at(position), descriptors);
  //de->compute(frame, *kp.at(position), *kd.at(position));
  */

  // TODO: remove this hack (but: DescriptorExtractor gives segfaults!)
  SIFT sift;
  sift(frame, Mat(), *kp.at(position), *kd.at(position), true);
  
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
                                    vector<KeyPoint>& keypoints)
{
  keypoints = *kp.at(frame);
}

bool FeatureSequence::visualiseFeaturesOf(int position, Mat& output, Scalar colour)
{
  sc->setPosition(position);
  Mat frame;
  if (!sc->read(frame))
    return false;

  drawKeypoints(frame, *kp.at(position), output, colour, DrawMatchesFlags::DEFAULT);
  return true;
}


